/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>

#include <sdf/sdf.hh>

#include <cstddef>
#include <string>
#include <vector>

namespace custom {

class BodyAeroWrenchSystem : public gz::sim::System,
                             public gz::sim::ISystemConfigure,
                             public gz::sim::ISystemPreUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &eventMgr) final;

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) final;

private:
  struct WrenchPoint {
    double alpha_deg{};
    double beta_deg{};
    gz::math::Vector3d force_per_q{};
    gz::math::Vector3d moment_per_q{};
  };

  bool LoadTable(const std::string &path);
  bool ResolveLink(gz::sim::EntityComponentManager &ecm);
  WrenchPoint Interpolate(double alpha_deg, double beta_deg) const;
  static std::string DefaultTablePath();

  gz::sim::Entity _model_entity{gz::sim::kNullEntity};
  gz::sim::Entity _wind_entity{gz::sim::kNullEntity};
  gz::sim::Link _link{gz::sim::kNullEntity};
  std::string _link_name{"base_link"};
  std::string _table_path{};
  std::vector<WrenchPoint> _points{};

  double _rho{1.225};
  double _force_scale{1.0};
  double _moment_scale{1.0};
  double _minimum_airspeed{0.1};
  double _alpha_min_deg{-26.0};
  double _alpha_max_deg{26.0};
  double _beta_min_deg{-45.0};
  double _beta_max_deg{45.0};
  double _alpha_distance_scale_deg{10.0};
  double _beta_distance_scale_deg{10.0};
  std::size_t _neighbor_count{6};
  bool _configured{false};
  bool _warned_outside_envelope{false};
};

} // namespace custom
