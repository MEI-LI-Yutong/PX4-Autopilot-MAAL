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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>

using namespace custom;

namespace
{
std::string DefaultDragTablePath()
{
	const std::string file_path = __FILE__;
	const auto pos = file_path.find_last_of('/');
	if (pos == std::string::npos) {
		return "data/figure20_drag.csv";
	}
	return file_path.substr(0, pos) + "/data/figure20_drag.csv";
}

std::string ResolveDragTablePath(const std::string &path)
{
	if (path.empty()) {
		return DefaultDragTablePath();
	}
	if (!path.empty() && path.front() == '/') {
		return path;
	}
	const std::string base = DefaultDragTablePath();
	const auto pos = base.find_last_of('/');
	if (pos == std::string::npos) {
		return path;
	}
	return base.substr(0, pos) + "/" + path;
}
} // namespace

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
		_drag_table_path = _sdf->Get<std::string>("drag_table", _drag_table_path).first;
		_rho = _sdf->Get<double>("rho", _rho).first;
		_k_scale = _sdf->Get<double>("k_scale", _k_scale).first;
		if (_sdf->HasElement("drag_coeff")) {
			_k_scale = _sdf->Get<double>("drag_coeff", _k_scale).first;
		}
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
	_drag_table_path = ResolveDragTablePath(_drag_table_path);
	_drag_table_loaded = LoadDragTable(_drag_table_path);
	std::cerr << "RotorWindDragSystem loaded: model=" << (_model_name.empty() ? "<entity>" : _model_name)
		  << ", base_link=" << _base_link_name
		  << ", rotors=" << _rotor_link_names.size()
		  << ", rho=" << _rho
		  << ", k_scale=" << _k_scale
		  << ", drag_table=" << _drag_table_path
		  << ", drag_table_loaded=" << (_drag_table_loaded ? "true" : "false")
		  << std::endl;

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

	if (_k_scale <= 0.0) {
		std::cerr << "RotorWindDragSystem: k_scale <= 0, force will be zero." << std::endl;
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

	gz::math::Vector3d wind_body = wind;
	const auto pose_opt = _base_link.WorldPose(_ecm);
	if (pose_opt.has_value()) {
		const auto &pose = pose_opt.value();
		wind_body = pose.Rot().Inverse().RotateVector(wind);
	}

	const double wind_speed = wind.Length();
	std::vector<double> eta_rpm;
	const double eta_bar = ComputeEtaBar(_ecm, eta_rpm);
	const double gamma_rad = std::atan2(wind_body.X(), wind_body.Z());
	const double gamma_deg = gamma_rad * 180.0 / M_PI;
	const double k_lookup = _drag_table_loaded ? LookupDragK(eta_bar, gamma_deg) : 0.0;
	const double k_eff = k_lookup * _k_scale;
	const double drag_mag_debug = _rho * k_eff * wind_speed;

	if (_debug_interval_s > 0.0) {
		const double now_s = std::chrono::duration<double>(_info.simTime).count();
		if (_last_debug_time_s < 0.0 || (now_s - _last_debug_time_s) >= _debug_interval_s) {
			std::cerr << "RotorWindDragSystem: wind=" << wind_speed
				  << " m/s, wind_world=(" << wind.X() << "," << wind.Y() << "," << wind.Z() << ")"
				  << ", wind_body=(" << wind_body.X() << "," << wind_body.Y() << "," << wind_body.Z() << ")"
				  << ", gamma_deg=" << gamma_deg
				  << ", eta_rpm=[";
			for (size_t i = 0; i < eta_rpm.size(); ++i) {
				std::cerr << eta_rpm[i] << (i + 1 < eta_rpm.size() ? "," : "");
			}
			std::cerr << "], eta_bar=" << eta_bar
				  << ", k_lookup=" << k_lookup
				  << ", k_scale=" << _k_scale
				  << ", k_eff=" << k_eff
				  << ", rho=" << _rho
				  << ", drag_mag=" << drag_mag_debug
				  << ", wind_src=" << (wind_from_topic ? "topic" : "component")
				  << ", rotor_src=" << (_motor_speed_subscribed ? "topic" : "link")
				  << std::endl;
			_last_debug_time_s = now_s;
		}
	}

	if (wind_speed <= 1e-6 || eta_bar <= 1e-6 || std::fabs(k_eff) <= 1e-9) {
		return;
	}

	const gz::math::Vector3d drag_dir = -wind.Normalized();
	const double drag_mag = _rho * k_eff * wind_speed;
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

double RotorWindDragSystem::ComputeEtaBar(const gz::sim::EntityComponentManager &_ecm,
					  std::vector<double> &eta_rpm_out) const
{
	eta_rpm_out.clear();

	if (_motor_speed_subscribed) {
		std::lock_guard<std::mutex> lock(_motor_mutex);
		const double now_s = std::chrono::duration<double>(
			std::chrono::steady_clock::now().time_since_epoch()).count();
		if (_motor_speed_time_s >= 0.0 &&
		    (now_s - _motor_speed_time_s) <= _motor_speed_stale_sec &&
		    !_motor_speed_from_topic.empty()) {
			for (const double v : _motor_speed_from_topic) {
				const double rpm = v * 60.0 / (2.0 * M_PI);
				eta_rpm_out.push_back(rpm);
			}
			const double sum_sq = std::accumulate(eta_rpm_out.begin(), eta_rpm_out.end(), 0.0,
				[](double acc, double value) { return acc + value * value; });
			return 0.5 * std::sqrt(sum_sq);
		}
	}

	double sum_sq = 0.0;
	size_t count = 0;

	for (const auto &link : _rotor_links) {
		const auto angular_vel = link.WorldAngularVelocity(_ecm);
		if (angular_vel.has_value()) {
			const double rpm = angular_vel.value().Length() * 60.0 / (2.0 * M_PI);
			eta_rpm_out.push_back(rpm);
			sum_sq += rpm * rpm;
			++count;
		}
	}

	if (count == 0) {
		return 0.0;
	}

	return 0.5 * std::sqrt(sum_sq);
}

bool RotorWindDragSystem::LoadDragTable(const std::string &path)
{
	_drag_table.clear();
	_rpm_grid.clear();
	_pitch_grid.clear();

	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "RotorWindDragSystem: failed to open drag table: " << path << std::endl;
		return false;
	}

	std::string line;
	bool is_header = true;
	while (std::getline(file, line)) {
		if (is_header) {
			is_header = false;
			continue;
		}
		if (line.empty()) {
			continue;
		}

		std::stringstream ss(line);
		std::string rpm_str;
		std::string pitch_str;
		std::string drag_str;

		if (!std::getline(ss, rpm_str, ',')) {
			continue;
		}
		if (!std::getline(ss, pitch_str, ',')) {
			continue;
		}
		if (!std::getline(ss, drag_str, ',')) {
			continue;
		}

		const double rpm = std::stod(rpm_str);
		const double pitch = std::stod(pitch_str);
		const double drag = std::stod(drag_str);

		_drag_table[{rpm, pitch}] = drag;
		_rpm_grid.push_back(rpm);
		_pitch_grid.push_back(pitch);
	}

	if (_drag_table.empty()) {
		std::cerr << "RotorWindDragSystem: drag table empty: " << path << std::endl;
		return false;
	}

	std::sort(_rpm_grid.begin(), _rpm_grid.end());
	_rpm_grid.erase(std::unique(_rpm_grid.begin(), _rpm_grid.end()), _rpm_grid.end());
	std::sort(_pitch_grid.begin(), _pitch_grid.end());
	_pitch_grid.erase(std::unique(_pitch_grid.begin(), _pitch_grid.end()), _pitch_grid.end());

	return !_rpm_grid.empty() && !_pitch_grid.empty();
}

double RotorWindDragSystem::LookupDragK(double rpm, double gamma_deg) const
{
	if (_rpm_grid.empty() || _pitch_grid.empty()) {
		return 0.0;
	}

	const double rpm_clamped = std::clamp(rpm, _rpm_grid.front(), _rpm_grid.back());
	const double pitch_clamped = std::clamp(gamma_deg, _pitch_grid.front(), _pitch_grid.back());

	auto rpm_it = std::lower_bound(_rpm_grid.begin(), _rpm_grid.end(), rpm_clamped);
	auto pitch_it = std::lower_bound(_pitch_grid.begin(), _pitch_grid.end(), pitch_clamped);

	double rpm1 = (rpm_it == _rpm_grid.begin()) ? *rpm_it : *(rpm_it - 1);
	double rpm2 = (rpm_it == _rpm_grid.end()) ? _rpm_grid.back() : *rpm_it;
	double pitch1 = (pitch_it == _pitch_grid.begin()) ? *pitch_it : *(pitch_it - 1);
	double pitch2 = (pitch_it == _pitch_grid.end()) ? _pitch_grid.back() : *pitch_it;

	if (std::fabs(rpm1 - rpm2) <= 1e-9 && std::fabs(pitch1 - pitch2) <= 1e-9) {
		auto it = _drag_table.find({rpm1, pitch1});
		return it != _drag_table.end() ? it->second : 0.0;
	}

	const double q11 = _drag_table.count({rpm1, pitch1}) ? _drag_table.at({rpm1, pitch1}) : 0.0;
	const double q12 = _drag_table.count({rpm1, pitch2}) ? _drag_table.at({rpm1, pitch2}) : 0.0;
	const double q21 = _drag_table.count({rpm2, pitch1}) ? _drag_table.at({rpm2, pitch1}) : 0.0;
	const double q22 = _drag_table.count({rpm2, pitch2}) ? _drag_table.at({rpm2, pitch2}) : 0.0;

	const double rpm_diff = rpm2 - rpm1;
	const double pitch_diff = pitch2 - pitch1;
	const double rpm_den = std::fabs(rpm_diff) > 1e-9 ? rpm_diff : 1.0;
	const double pitch_den = std::fabs(pitch_diff) > 1e-9 ? pitch_diff : 1.0;

	const double t = (rpm_clamped - rpm1) / rpm_den;
	const double u = (pitch_clamped - pitch1) / pitch_den;

	const double q1 = q11 + t * (q21 - q11);
	const double q2 = q12 + t * (q22 - q12);
	return q1 + u * (q2 - q1);
}
