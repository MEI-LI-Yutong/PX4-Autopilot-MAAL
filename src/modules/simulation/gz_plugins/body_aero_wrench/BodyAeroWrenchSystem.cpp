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

#include "BodyAeroWrenchSystem.hpp"

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Wind.hh>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

GZ_ADD_PLUGIN(custom::BodyAeroWrenchSystem, gz::sim::System,
              custom::BodyAeroWrenchSystem::ISystemConfigure,
              custom::BodyAeroWrenchSystem::ISystemPreUpdate)

namespace custom {

std::string BodyAeroWrenchSystem::DefaultTablePath() {
  const std::string source_path = __FILE__;
  const auto separator = source_path.find_last_of('/');
  const std::string directory =
      separator == std::string::npos ? "." : source_path.substr(0, separator);
  return directory + "/data/x500_wrench_table.csv";
}

void BodyAeroWrenchSystem::Configure(
    const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &sdf,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) {
  _model_entity = entity;

  if (sdf) {
    _link_name = sdf->Get<std::string>("link_name", _link_name).first;
    _table_path = sdf->Get<std::string>("table_path", _table_path).first;
    _rho = sdf->Get<double>("air_density", _rho).first;
    _force_scale = sdf->Get<double>("force_scale", _force_scale).first;
    _moment_scale = sdf->Get<double>("moment_scale", _moment_scale).first;
    _force_axis_scale =
        sdf->Get<gz::math::Vector3d>("force_axis_scale", _force_axis_scale)
            .first;
    _moment_axis_scale =
        sdf->Get<gz::math::Vector3d>("moment_axis_scale", _moment_axis_scale)
            .first;
    _minimum_airspeed =
        sdf->Get<double>("minimum_airspeed", _minimum_airspeed).first;
    _neighbor_count =
        sdf->Get<std::size_t>("neighbor_count", _neighbor_count).first;
  }

  if (_table_path.empty()) {
    _table_path = DefaultTablePath();
  }

  if (!LoadTable(_table_path)) {
    gzerr << "BodyAeroWrenchSystem: failed to load table " << _table_path
          << std::endl;
    return;
  }

  if (!ResolveLink(ecm)) {
    gzerr << "BodyAeroWrenchSystem: link '" << _link_name << "' not found"
          << std::endl;
    return;
  }

  _link.EnableVelocityChecks(ecm, true);
  _wind_entity = ecm.EntityByComponents(gz::sim::components::Wind());
  _configured = true;
  gzmsg << "BodyAeroWrenchSystem: loaded " << _points.size()
        << " X500 wind-tunnel points from " << _table_path << std::endl;
}

bool BodyAeroWrenchSystem::ResolveLink(gz::sim::EntityComponentManager &ecm) {
  if (_model_entity == gz::sim::kNullEntity) {
    return false;
  }

  gz::sim::Model model(_model_entity);
  _link = gz::sim::Link(model.LinkByName(ecm, _link_name));
  return _link.Valid(ecm);
}

bool BodyAeroWrenchSystem::LoadTable(const std::string &path) {
  std::ifstream stream(path);

  if (!stream.is_open()) {
    return false;
  }

  _points.clear();
  std::string line;
  std::getline(stream, line);

  while (std::getline(stream, line)) {
    if (line.empty()) {
      continue;
    }

    std::array<double, 8> values{};
    std::stringstream row(line);
    std::string field;
    bool valid = true;

    for (double &value : values) {
      if (!std::getline(row, field, ',')) {
        valid = false;
        break;
      }

      try {
        value = std::stod(field);
      } catch (...) {
        valid = false;
        break;
      }
    }

    if (valid) {
      _points.push_back({values[0],
                         values[1],
                         {values[2], values[3], values[4]},
                         {values[5], values[6], values[7]}});
    }
  }

  if (_points.empty()) {
    return false;
  }

  auto alpha_bounds =
      std::minmax_element(_points.begin(), _points.end(),
                          [](const WrenchPoint &lhs, const WrenchPoint &rhs) {
                            return lhs.alpha_deg < rhs.alpha_deg;
                          });
  auto beta_bounds =
      std::minmax_element(_points.begin(), _points.end(),
                          [](const WrenchPoint &lhs, const WrenchPoint &rhs) {
                            return lhs.beta_deg < rhs.beta_deg;
                          });
  _alpha_min_deg = alpha_bounds.first->alpha_deg;
  _alpha_max_deg = alpha_bounds.second->alpha_deg;
  _beta_min_deg = beta_bounds.first->beta_deg;
  _beta_max_deg = beta_bounds.second->beta_deg;
  _neighbor_count = std::clamp<std::size_t>(_neighbor_count, 1, _points.size());
  return true;
}

BodyAeroWrenchSystem::WrenchPoint
BodyAeroWrenchSystem::Interpolate(double alpha_deg, double beta_deg) const {
  struct Neighbor {
    double distance_squared;
    const WrenchPoint *point;
  };

  std::vector<Neighbor> neighbors;
  neighbors.reserve(_points.size());

  for (const WrenchPoint &point : _points) {
    const double da = (alpha_deg - point.alpha_deg) / _alpha_distance_scale_deg;
    const double db = (beta_deg - point.beta_deg) / _beta_distance_scale_deg;
    const double distance_squared = da * da + db * db;

    if (distance_squared < 1e-12) {
      return point;
    }

    neighbors.push_back({distance_squared, &point});
  }

  std::partial_sort(neighbors.begin(), neighbors.begin() + _neighbor_count,
                    neighbors.end(),
                    [](const Neighbor &lhs, const Neighbor &rhs) {
                      return lhs.distance_squared < rhs.distance_squared;
                    });

  WrenchPoint result{};
  result.alpha_deg = alpha_deg;
  result.beta_deg = beta_deg;
  double weight_sum = 0.0;

  for (std::size_t index = 0; index < _neighbor_count; ++index) {
    const double weight = 1.0 / neighbors[index].distance_squared;
    result.force_per_q += neighbors[index].point->force_per_q * weight;
    result.moment_per_q += neighbors[index].point->moment_per_q * weight;
    weight_sum += weight;
  }

  result.force_per_q /= weight_sum;
  result.moment_per_q /= weight_sum;
  return result;
}

void BodyAeroWrenchSystem::PreUpdate(const gz::sim::UpdateInfo &info,
                                     gz::sim::EntityComponentManager &ecm) {
  if (!_configured || info.paused) {
    return;
  }

  const auto world_velocity = _link.WorldLinearVelocity(ecm);
  const auto world_pose = _link.WorldPose(ecm);

  if (!world_velocity || !world_pose) {
    return;
  }

  gz::math::Vector3d wind_world{};

  if (_wind_entity == gz::sim::kNullEntity) {
    _wind_entity = ecm.EntityByComponents(gz::sim::components::Wind());
  }

  if (_wind_entity != gz::sim::kNullEntity) {
    const auto wind_velocity =
        ecm.Component<gz::sim::components::WorldLinearVelocity>(_wind_entity);

    if (wind_velocity) {
      wind_world = wind_velocity->Data();
    }
  }

  const gz::math::Vector3d air_velocity_world =
      world_velocity.value() - wind_world;
  const double airspeed = air_velocity_world.Length();

  if (airspeed < _minimum_airspeed) {
    return;
  }

  gz::math::Vector3d lookup_velocity =
      world_pose->Rot().Inverse().RotateVector(air_velocity_world);
  const bool reverse_flow = lookup_velocity.X() < 0.0;

  if (reverse_flow) {
    lookup_velocity.X(-lookup_velocity.X());
    lookup_velocity.Y(-lookup_velocity.Y());
  }

  const double radians_to_degrees = 180.0 / GZ_PI;
  const double alpha_raw =
      std::atan2(-lookup_velocity.Z(),
                 std::hypot(lookup_velocity.X(), lookup_velocity.Y())) *
      radians_to_degrees;
  const double beta_raw =
      std::atan2(lookup_velocity.Y(), lookup_velocity.X()) * radians_to_degrees;
  const bool outside_envelope =
      alpha_raw < _alpha_min_deg || alpha_raw > _alpha_max_deg ||
      beta_raw < _beta_min_deg || beta_raw > _beta_max_deg;
  const double alpha = std::clamp(alpha_raw, _alpha_min_deg, _alpha_max_deg);
  const double beta = std::clamp(beta_raw, _beta_min_deg, _beta_max_deg);

  if (!_warned_outside_envelope && outside_envelope) {
    gzwarn << "BodyAeroWrenchSystem: airflow is outside the measured envelope; "
              "clamping alpha/beta to ["
           << _alpha_min_deg << ", " << _alpha_max_deg << "] / ["
           << _beta_min_deg << ", " << _beta_max_deg << "] deg" << std::endl;
    _warned_outside_envelope = true;
  }

  const WrenchPoint coefficients = Interpolate(alpha, beta);
  const double dynamic_pressure = 0.5 * _rho * airspeed * airspeed;
  gz::math::Vector3d force_body =
      coefficients.force_per_q * (dynamic_pressure * _force_scale);
  gz::math::Vector3d moment_body =
      coefficients.moment_per_q * (dynamic_pressure * _moment_scale);
  force_body.X(force_body.X() * _force_axis_scale.X());
  force_body.Y(force_body.Y() * _force_axis_scale.Y());
  force_body.Z(force_body.Z() * _force_axis_scale.Z());
  moment_body.X(moment_body.X() * _moment_axis_scale.X());
  moment_body.Y(moment_body.Y() * _moment_axis_scale.Y());
  moment_body.Z(moment_body.Z() * _moment_axis_scale.Z());

  if (reverse_flow) {
    force_body.X(-force_body.X());
    force_body.Y(-force_body.Y());
    moment_body.X(-moment_body.X());
    moment_body.Y(-moment_body.Y());
  }

  const auto &rotation = world_pose->Rot();
  const gz::math::Vector3d force_world = rotation.RotateVector(force_body);
  const gz::math::Vector3d moment_world = rotation.RotateVector(moment_body);
  _link.AddWorldForce(ecm, force_world);
  _link.AddWorldWrench(ecm, gz::math::Vector3d::Zero, moment_world);
}

} // namespace custom
