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
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Wind.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/vector3d.pb.h>
#include <sdf/sdf.hh>

#include <mutex>
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
	void OnWindMsg(const gz::msgs::Vector3d &_msg);
	void OnMotorSpeedMsg(const gz::msgs::Actuators &_msg);

	gz::sim::Entity _model_entity{gz::sim::kNullEntity};
	gz::sim::Link _base_link;
	std::vector<std::string> _rotor_link_names;
	std::vector<gz::sim::Link> _rotor_links;

	gz::sim::Entity _wind_entity{gz::sim::kNullEntity};

	std::string _base_link_name{"base_link"};
	std::string _model_name{};
	std::string _wind_topic{"/world/windy/wind_gust"};
	std::string _motor_speed_topic{};
	double _drag_coeff{0.02};
	double _wind_stale_sec{1.0};
	double _motor_speed_stale_sec{1.0};
	bool _configured{false};
	double _debug_interval_s{1.0};
	double _last_debug_time_s{-1.0};
	bool _warned_missing_wind{false};
	bool _warned_missing_model{false};

	gz::transport::Node _node;
	mutable std::mutex _wind_mutex;
	gz::math::Vector3d _wind_from_topic{};
	double _wind_from_topic_time_s{-1.0};
	bool _wind_topic_subscribed{false};

	mutable std::mutex _motor_mutex;
	std::vector<double> _motor_speed_from_topic{};
	double _motor_speed_time_s{-1.0};
	bool _motor_speed_subscribed{false};
};
} // namespace custom
