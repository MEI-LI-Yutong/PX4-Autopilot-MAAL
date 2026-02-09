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
		_base_link_name = _sdf->Get<std::string>("base_link", _base_link_name).first;
		_drag_coeff = _sdf->Get<double>("drag_coeff", _drag_coeff).first;

		for (auto rotor_elem = _sdf->FindElement("rotor_link"); rotor_elem;
		     rotor_elem = rotor_elem->GetNextElement("rotor_link")) {
			_rotor_link_names.emplace_back(rotor_elem->Get<std::string>());
		}
	}

	if (_rotor_link_names.empty()) {
		_rotor_link_names = {"rotor_0", "rotor_1", "rotor_2", "rotor_3"};
	}

	_configured = ResolveEntities(_ecm);
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

	const auto wind_vel_comp = _ecm.Component<gz::sim::components::WorldLinearVelocity>(_wind_entity);
	if (!wind_vel_comp) {
		return;
	}

	const gz::math::Vector3d wind = wind_vel_comp->Data();
	const double wind_speed = wind.Length();
	if (wind_speed <= 1e-6) {
		return;
	}

	const double rotor_speed = ComputeAverageRotorSpeed(_ecm);
	if (rotor_speed <= 1e-6) {
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
		return false;
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

	return _wind_entity != gz::sim::kNullEntity;
}

double RotorWindDragSystem::ComputeAverageRotorSpeed(const gz::sim::EntityComponentManager &_ecm) const
{
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
