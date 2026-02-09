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

#pragma once

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <sdf/sdf.hh>

#include <string>
#include <vector>

namespace custom
{
class RotorWindDragSystem :
	public gz::sim::System,
	public gz::sim::ISystemConfigure,
	public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &/*_eventMgr*/) final;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) final;

private:
	bool ResolveEntities(gz::sim::EntityComponentManager &_ecm);
	double ComputeAverageRotorSpeed(const gz::sim::EntityComponentManager &_ecm) const;

	gz::sim::Entity _model_entity{gz::sim::kNullEntity};
	gz::sim::Link _base_link;
	std::vector<std::string> _rotor_link_names;
	std::vector<gz::sim::Link> _rotor_links;

	gz::sim::Entity _wind_entity{gz::sim::kNullEntity};

	std::string _base_link_name{"base_link"};
	double _drag_coeff{0.02};
	bool _configured{false};
};
} // namespace custom
