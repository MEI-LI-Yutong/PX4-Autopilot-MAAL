/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "RotorWindDragSystem.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <chrono>
#include <iostream>

using namespace custom;

GZ_ADD_PLUGIN(
	RotorWindDragSystem,
	gz::sim::System,
	RotorWindDragSystem::ISystemConfigure,
	RotorWindDragSystem::ISystemPreUpdate
)

void RotorWindDragSystem::Configure(const gz::sim::Entity &_entity,
				    const std::shared_ptr<const sdf::Element> &_sdf,
				    gz::sim::EntityComponentManager &_ecm,
				    gz::sim::EventManager &/*_eventMgr*/)
{
	_model_entity = _entity;

	if (_sdf) {
		_model_name = _sdf->Get<std::string>("model_name", _model_name).first;
		_base_link_name = _sdf->Get<std::string>("base_link", _base_link_name).first;
		_drag_coeff = _sdf->Get<double>("drag_coeff", _drag_coeff).first;
		_debug_interval_s = _sdf->Get<double>("debug_interval", _debug_interval_s).first;
		_wind_topic = _sdf->Get<std::string>("wind_topic", _wind_topic).first;
		_wind_stale_sec = _sdf->Get<double>("wind_stale_sec", _wind_stale_sec).first;
		_motor_speed_topic = _sdf->Get<std::string>("motor_speed_topic", _motor_speed_topic).first;
		_motor_speed_stale_sec = _sdf->Get<double>("motor_speed_stale_sec", _motor_speed_stale_sec).first;

		for (auto rotor_elem = _sdf->FindElement("rotor_link"); rotor_elem;
		     rotor_elem = rotor_elem->GetNextElement("rotor_link")) {
			_rotor_link_names.emplace_back(rotor_elem->Get<std::string>());
		}
	}

	if (_rotor_link_names.empty()) {
		_rotor_link_names = {"rotor_0", "rotor_1", "rotor_2", "rotor_3"};
	}

	if (_model_name.empty()) {
		const auto name_comp = _ecm.Component<gz::sim::components::Name>(_model_entity);
		if (name_comp) {
			_model_name = name_comp->Data();
		}
	}

	_configured = ResolveEntities(_ecm);
	std::cerr << "RotorWindDragSystem loaded: model=" << (_model_name.empty() ? "<entity>" : _model_name)
		  << ", base_link=" << _base_link_name
		  << ", rotors=" << _rotor_link_names.size()
		  << ", drag_coeff=" << _drag_coeff << std::endl;

	if (!_wind_topic.empty()) {
		_wind_topic_subscribed = _node.Subscribe(_wind_topic, &RotorWindDragSystem::OnWindMsg, this);
		if (!_wind_topic_subscribed) {
			std::cerr << "RotorWindDragSystem: failed to subscribe wind topic: " << _wind_topic << std::endl;
		}
	}

	if (_motor_speed_topic.empty() && !_model_name.empty()) {
		_motor_speed_topic = "/" + _model_name + "/command/motor_speed";
	}

	if (!_motor_speed_topic.empty()) {
		_motor_speed_subscribed = _node.Subscribe(_motor_speed_topic, &RotorWindDragSystem::OnMotorSpeedMsg, this);
		if (!_motor_speed_subscribed) {
			std::cerr << "RotorWindDragSystem: failed to subscribe motor speed topic: " << _motor_speed_topic << std::endl;
		} else {
			std::cerr << "RotorWindDragSystem: subscribed motor speed topic: " << _motor_speed_topic << std::endl;
		}
	}

	if (_drag_coeff <= 0.0) {
		std::cerr << "RotorWindDragSystem: drag_coeff <= 0, force will be zero." << std::endl;
	}
}

void RotorWindDragSystem::PreUpdate(const gz::sim::UpdateInfo &_info,
				    gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	if (!_configured && !ResolveEntities(_ecm)) {
		return;
	}

	gz::math::Vector3d wind{};
	bool wind_from_topic = false;
	if (_wind_topic_subscribed) {
		std::lock_guard<std::mutex> lock(_wind_mutex);
		const double now_s = std::chrono::duration<double>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
		if (_wind_from_topic_time_s >= 0.0 &&
		    (now_s - _wind_from_topic_time_s) <= _wind_stale_sec) {
			wind = _wind_from_topic;
			wind_from_topic = true;
		}
	}

	if (!wind_from_topic) {
		const auto wind_vel_comp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(_wind_entity);
		if (!wind_vel_comp) {
			if (!_warned_missing_wind) {
				std::cerr << "RotorWindDragSystem: missing wind velocity component." << std::endl;
				_warned_missing_wind = true;
			}
			return;
		}
		wind = wind_vel_comp->Data();
	}

	const double wind_speed = wind.Length();
	const double rotor_speed = ComputeAverageRotorSpeed(_ecm);

	if (_debug_interval_s > 0.0) {
		const double now_s = std::chrono::duration<double>(_info.simTime).count();
		if (_last_debug_time_s < 0.0 || (now_s - _last_debug_time_s) >= _debug_interval_s) {
			std::cerr << "RotorWindDragSystem: wind=" << wind_speed
				  << " m/s, rotor_avg=" << rotor_speed
				  << " rad/s, drag_coeff=" << _drag_coeff
				  << ", wind_src=" << (wind_from_topic ? "topic" : "component")
				  << ", rotor_src=" << (_motor_speed_subscribed ? "topic" : "link")
				  << std::endl;
			_last_debug_time_s = now_s;
		}
	}

	if (wind_speed <= 1e-6 || rotor_speed <= 1e-6) {
		return;
	}

	const gz::math::Vector3d drag_dir = -wind.Normalized();
	const double drag_mag = _drag_coeff * wind_speed * rotor_speed;
	const gz::math::Vector3d drag_force = drag_dir * drag_mag;

	_base_link.AddWorldForce(_ecm, drag_force);
}

bool RotorWindDragSystem::ResolveEntities(gz::sim::EntityComponentManager &_ecm)
{
	if (_model_entity == gz::sim::kNullEntity) {
		if (!_model_name.empty()) {
			_model_entity = _ecm.EntityByComponents(
				gz::sim::components::Model(),
				gz::sim::components::Name(_model_name));
		}
		if (_model_entity == gz::sim::kNullEntity) {
			if (!_warned_missing_model) {
				std::cerr << "RotorWindDragSystem: model not found: " << _model_name << std::endl;
				_warned_missing_model = true;
			}
			return false;
		}
	}

	gz::sim::Model model(_model_entity);
	const gz::sim::Entity base_link_entity = model.LinkByName(_ecm, _base_link_name);
	_base_link = gz::sim::Link(base_link_entity);

	_rotor_links.clear();
	_rotor_links.reserve(_rotor_link_names.size());
	for (const auto &name : _rotor_link_names) {
		const gz::sim::Entity link_entity = model.LinkByName(_ecm, name);
		const gz::sim::Link link(link_entity);
		if (link.Valid(_ecm)) {
			_rotor_links.push_back(link);
		}
	}

	if (!_base_link.Valid(_ecm) || _rotor_links.empty()) {
		return false;
	}

	if (_wind_entity == gz::sim::kNullEntity) {
		_wind_entity = _ecm.EntityByComponents(gz::sim::components::Wind());
	}

	if (_wind_entity == gz::sim::kNullEntity && !_warned_missing_wind) {
		std::cerr << "RotorWindDragSystem: wind entity not found." << std::endl;
		_warned_missing_wind = true;
	}

	return _wind_entity != gz::sim::kNullEntity;
}

void RotorWindDragSystem::OnWindMsg(const gz::msgs::Vector3d &_msg)
{
	std::lock_guard<std::mutex> lock(_wind_mutex);
	_wind_from_topic = gz::math::Vector3d(_msg.x(), _msg.y(), _msg.z());
	_wind_from_topic_time_s = std::chrono::duration<double>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

void RotorWindDragSystem::OnMotorSpeedMsg(const gz::msgs::Actuators &_msg)
{
	std::lock_guard<std::mutex> lock(_motor_mutex);
	_motor_speed_from_topic.clear();
	_motor_speed_from_topic.reserve(static_cast<size_t>(_msg.velocity_size()));
	for (int i = 0; i < _msg.velocity_size(); ++i) {
		_motor_speed_from_topic.push_back(_msg.velocity(i));
	}
	_motor_speed_time_s = std::chrono::duration<double>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

double RotorWindDragSystem::ComputeAverageRotorSpeed(const gz::sim::EntityComponentManager &_ecm) const
{
	if (_motor_speed_subscribed) {
		std::lock_guard<std::mutex> lock(_motor_mutex);
		const double now_s = std::chrono::duration<double>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
		if (_motor_speed_time_s >= 0.0 &&
		    (now_s - _motor_speed_time_s) <= _motor_speed_stale_sec &&
		    !_motor_speed_from_topic.empty()) {
			double sum = 0.0;
			for (const double v : _motor_speed_from_topic) {
				sum += v;
			}
			return sum / static_cast<double>(_motor_speed_from_topic.size());
		}
	}

	double sum = 0.0;
	size_t count = 0;

	for (const auto &link : _rotor_links) {
		const auto angular_vel = link.WorldAngularVelocity(_ecm);
		if (angular_vel.has_value()) {
			sum += angular_vel.value().Length();
			++count;
		}
	}

	if (count == 0) {
		return 0.0;
	}

	return sum / static_cast<double>(count);
}
