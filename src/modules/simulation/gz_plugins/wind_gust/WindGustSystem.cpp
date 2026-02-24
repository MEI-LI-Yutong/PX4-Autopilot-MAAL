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

 #include "WindGustSystem.hpp"

 #include <gz/plugin/Register.hh>
 #include <gz/common/Console.hh>

 #include <gz/sim/components/Wind.hh>
 #include <gz/sim/components/LinearVelocity.hh>
 #include <gz/sim/components/Pose.hh>
 #include <gz/sim/components/Model.hh>
 #include <gz/sim/components/Name.hh>
 #include <gz/sim/Util.hh>

 #include <gz/math/Vector3.hh>
 #include <gz/math/Helpers.hh>
 #include <gz/msgs/vector3d.pb.h>
 #include <gz/msgs/Utility.hh>
 #include <algorithm>
 #include <cmath>
 #include <fstream>
 #include <sstream>

 using namespace gz::sim;
 using namespace gz::math;
 using namespace custom;

namespace
{
std::string DefaultCsvPath()
{
	const std::string file_path = __FILE__;
	const auto pos = file_path.find_last_of('/');
	if (pos == std::string::npos) {
		return "data/flight_wind_xyz_attitude.csv";
	}
	return file_path.substr(0, pos) + "/data/flight_wind_xyz_attitude.csv";
}

std::string ResolveCsvPath(const std::string &path)
{
	if (path.empty()) {
		return DefaultCsvPath();
	}
	if (!path.empty() && path.front() == '/') {
		return path;
	}
	const std::string base = DefaultCsvPath();
	const auto pos = base.find_last_of('/');
	if (pos == std::string::npos) {
		return path;
	}
	return base.substr(0, pos) + "/" + path;
}
} // namespace

 // Register the plugin
 GZ_ADD_PLUGIN(
     WindGustSystem,
     gz::sim::System,
     WindGustSystem::ISystemConfigure,
     WindGustSystem::ISystemPreUpdate)

 GZ_ADD_PLUGIN_ALIAS(WindGustSystem, "custom::WindGustSystem")

 /////////////////////////////////////////////////
 void WindGustSystem::Configure(const Entity &_entity,
				const std::shared_ptr<const sdf::Element> &_sdf,
				EntityComponentManager &_ecm,
				EventManager &)
 {
     _worldEntity = _entity;

     // Parse parameters
    if (_sdf) {
	 if (_sdf->HasElement("model")) {
	     _model = _sdf->Get<std::string>("model", _model).first;
	 }
     if (_sdf->HasElement("csv_path")) {
         _csv_path = _sdf->Get<std::string>("csv_path", _csv_path).first;
     }
     if (_sdf->HasElement("csv_loop")) {
         _csv_loop = _sdf->Get<bool>("csv_loop", _csv_loop).first;
     }
     if (_sdf->HasElement("csv_time_offset_s")) {
         _csv_time_offset_s = _sdf->Get<double>("csv_time_offset_s", _csv_time_offset_s).first;
     }
	 if (_sdf->HasElement("mean")) {
	     _mean = _sdf->Get<Vector3d>("mean", Vector3d::Zero).first;
	 }
	 if (_sdf->HasElement("amplitude")) {
	     _amplitude = _sdf->Get<Vector3d>("amplitude", Vector3d::Zero).first;
	 }
	 if (_sdf->HasElement("frequency")) {
	     _frequency_hz = _sdf->Get<double>("frequency", 0.0).first;
	 }
	 if (_sdf->HasElement("phase")) {
	     _phase_rad = _sdf->Get<double>("phase", 0.0).first;
	 }
     // 1-cos gust parameters (optional)
     if (_sdf->HasElement("gust_length")) {
         _gust_length_m = _sdf->Get<double>("gust_length", _gust_length_m).first;
     }
     if (_sdf->HasElement("airspeed")) {
         _airspeed_ms = _sdf->Get<double>("airspeed", _airspeed_ms).first;
     }
    if (_sdf->HasElement("direction")) {
        _direction = _sdf->Get<Vector3d>("direction", _direction).first;
    }

    if (_model == "csv") {
        _csv_path = ResolveCsvPath(_csv_path);
        _csv_loaded = LoadCsvWind(_csv_path);
        if (_csv_loaded) {
            gzwarn << "[WindGustSystem] csv=" << _csv_path << std::endl;
        } else {
            gzwarn << "[WindGustSystem] csv load failed: " << _csv_path << std::endl;
        }
    }

    // Dryden parameters (optional)
    if (_sdf->HasElement("dryden_sigma")) {
        _dryden_sigma = _sdf->Get<Vector3d>("dryden_sigma", _dryden_sigma).first;
    }
    if (_sdf->HasElement("dryden_length")) {
        _dryden_length = _sdf->Get<Vector3d>("dryden_length", _dryden_length).first;
    }
    if (_sdf->HasElement("seed")) {
        _rng_seed = static_cast<uint32_t>(_sdf->Get<int>("seed", 0).first);
        _rng.seed(_rng_seed);
        _rng_seeded = true;
    }

     // one_minus_cos_simp parameters (optional): A0 and T
    if (_sdf->HasElement("A0")) {
        _simp_A0 = _sdf->Get<double>("A0", _simp_A0).first;
    } else if (_sdf->HasElement("a0")) {
        _simp_A0 = _sdf->Get<double>("a0", _simp_A0).first;
    }
    if (_sdf->HasElement("T")) {
        _simp_T = _sdf->Get<double>("T", _simp_T).first;
    } else if (_sdf->HasElement("t")) {
        _simp_T = _sdf->Get<double>("t", _simp_T).first;
    }
    if (_sdf->HasElement("trigger_x")) {
        _simp_trigger_x = _sdf->Get<double>("trigger_x", _simp_trigger_x).first;
    }
    if (_sdf->HasElement("trigger_model")) {
        _simp_trigger_model = _sdf->Get<std::string>("trigger_model", _simp_trigger_model).first;
    }

   // ======== Spatial wind parameters (NEW) ========
   if (_sdf->HasElement("spatial_model")) {
       _spatial_model = _sdf->Get<std::string>("spatial_model", _spatial_model).first;
   }
   if (_sdf->HasElement("tracked_model")) {
       _tracked_model = _sdf->Get<std::string>("tracked_model", _tracked_model).first;
   }
   // Only boundary_layer is supported going forward
   if (!_spatial_model.empty() &&
       _spatial_model != "none" &&
       _spatial_model != "boundary_layer") {
       gzwarn << "WindGustSystem: unsupported spatial_model '" << _spatial_model
              << "' — only 'boundary_layer' is supported. Disabling spatial wind." << std::endl;
       _spatial_model = "none";
   }

   // Boundary layer parameters
   if (_sdf->HasElement("bl_ref_height")) {
       _bl_ref_height = _sdf->Get<double>("bl_ref_height", _bl_ref_height).first;
   }
   if (_sdf->HasElement("bl_exponent")) {
       _bl_exponent = _sdf->Get<double>("bl_exponent", _bl_exponent).first;
   }
   if (_sdf->HasElement("bl_ref_wind")) {
       _bl_ref_wind = _sdf->Get<Vector3d>("bl_ref_wind", _bl_ref_wind).first;
   }
    }

     // Try to resolve wind entity now; lazily fallback during updates
     _windEntity = _ecm.EntityByComponents(components::Wind());

     // Setup debug topic publisher: /world/<name>/wind_gust
     std::string worldName{"world"};
     if (auto name = _ecm.Component<components::Name>(_entity)) {
	 worldName = name->Data();
     }
     _topic = std::string("/world/") + worldName + "/wind_gust";
     _pub = _node.Advertise<gz::msgs::Vector3d>(_topic);

     // Initialize with current mean + amplitude (if frequency==0, constant)
     if (_windEntity != kNullEntity) {
	 Vector3d init = _mean + _amplitude;
	 auto windVel = _ecm.Component<components::WorldLinearVelocity>(_windEntity);
	 if (!windVel) {
	     _ecm.CreateComponent(_windEntity, components::WorldLinearVelocity(init));
	 } else {
	     windVel->Data() = init;
	 }
     }

     _configured = true;
 }

 /////////////////////////////////////////////////
 void WindGustSystem::PreUpdate(const UpdateInfo &_info,
				EntityComponentManager &_ecm)
 {
     if (!_configured || _info.paused) {
	 return;
     }

     // Locate wind entity if not set yet
     if (_windEntity == kNullEntity) {
	 _windEntity = _ecm.EntityByComponents(components::Wind());
	 if (_windEntity == kNullEntity) {
	     if (!_warnedMissingWind) {
		 gzdbg << "WindGustSystem: No wind entity found; waiting for <wind> in world SDF..." << std::endl;
		 _warnedMissingWind = true;
	     }
	     return;
	 }
     }

     // Compute wind vector at sim time
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (_last_time_s < 0.0) {
        _last_time_s = t;
    }
    const double dt = std::max(0.0, t - _last_time_s);
    _last_time_s = t;
    Vector3d wind = _mean;

    if (_model == "csv") {
        if (_csv_loaded) {
            wind = SampleCsvWind(t);
        } else {
            wind = Vector3d::Zero;
        }

        if (_csv_loaded && !_csv_started) {
            _csv_started = true;
            _csv_start_time_s = t;
            gzwarn << "[WindGustSystem] csv start: t0=" << _csv_start_time_s
                   << ", t=" << t << std::endl;
        }

    } else if (_model == "sine") {
	 if (_frequency_hz > 0.0) {
	     const double omega = 2.0 * GZ_PI * _frequency_hz;
	     wind.X() += _amplitude.X() * std::sin(omega * t + _phase_rad);
	     wind.Y() += _amplitude.Y() * std::sin(omega * t + _phase_rad);
	     wind.Z() += _amplitude.Z() * std::sin(omega * t + _phase_rad);
	 } else {
	     wind += _amplitude;
	 }

   } else if (_model == "one_minus_cos_simp") {
     // Simple 1-cos single gust based on explicit A0 and T
     bool gust_active = true;
     if (!std::isnan(_simp_trigger_x)) {
         if (!_simp_triggered) {
             const std::string &model_name =
                 !_simp_trigger_model.empty() ? _simp_trigger_model : _tracked_model;
             Vector3d pos;
             if (GetModelPosition(_ecm, model_name, _simp_trigger_entity, _warnedMissingTriggerModel, pos)) {
                 if (pos.X() >= _simp_trigger_x) {
                     _simp_triggered = true;
                     _simp_trigger_time_s = t;
                 }
             }
         }
         if (!_simp_triggered) {
             // Not triggered yet: keep gust at zero
             gust_active = false;
         }
     }

     Vector3d dir = _direction;
     if (dir.Length() < 1e-6) {
         dir.Set(1, 0, 0);
     } else {
         dir.Normalize();
     }
     const double T = _simp_T;
     double wg = 0.0;
     if (gust_active) {
         const double t0 = _simp_triggered ? _simp_trigger_time_s : 26.0; // gust start time (s)
         if (T > 1e-6 && t >= t0 && t <= t0 + T) {
             const double tau = t - t0;
             wg = 0.5 * _simp_A0 * (1.0 - std::cos(2.0 * GZ_PI * tau / T));
         }
     }
     wind += dir * wg;

   } else if (_model == "dryden") {
    // Dryden continuous gust model with zero at -a/sqrt(3)
    // Determine airspeed
    double Va = _airspeed_ms;
    if (Va <= 0.0) {
        Va = _mean.Length();
        if (Va <= 0.0) {
            Va = 15.0;
        }
    }

    const double Lu = std::max(1e-6, _dryden_length.X());
    const double Lv = std::max(1e-6, _dryden_length.Y());
    const double Lw = std::max(1e-6, _dryden_length.Z());
    const double au = Va / Lu;
    const double av = Va / Lv;
    const double aw = Va / Lw;

    // RNG seed if not provided explicitly
    if (!_rng_seeded) {
        std::random_device rd;
        _rng.seed(rd());
        _rng_seeded = true;
    }

    if (dt > 0.0) {
        const double sqrt_dt = std::sqrt(dt);

        // u: Ornstein-Uhlenbeck exact discretization
        if (au > 1e-9) {
            const double E = std::exp(-au * dt);
            const double var_inc = (1.0 - E * E) / (2.0 * au);
            _xu = E * _xu + std::sqrt(std::max(0.0, var_inc)) * _norm(_rng);
        } else {
            _xu = 0.0;
        }

        // v: second-order Euler–Maruyama
        {
            const double xv1_prev = _xv1;
            const double n = _norm(_rng) * sqrt_dt;
            _xv1 += (-2.0 * av * _xv1 - av * av * _xv2) * dt + n;
            _xv2 += xv1_prev * dt;
        }
        // w: second-order Euler–Maruyama
        {
            const double xw1_prev = _xw1;
            const double n = _norm(_rng) * sqrt_dt;
            _xw1 += (-2.0 * aw * _xw1 - aw * aw * _xw2) * dt + n;
            _xw2 += xw1_prev * dt;
        }
    }

    const double ug = _dryden_sigma.X() * std::sqrt(2.0 * au) * _xu;
    const double vg = _dryden_sigma.Y() * std::sqrt(3.0 * av) * (_xv1 + (av / std::sqrt(3.0)) * _xv2);
    const double wg = _dryden_sigma.Z() * std::sqrt(3.0 * aw) * (_xw1 + (aw / std::sqrt(3.0)) * _xw2);

    wind += Vector3d(ug, vg, wg);

   } else { // default: one_minus_cos single gust
	 // Derive V_inf if not provided
	 double V = _airspeed_ms;
	 if (V <= 0.0) {
	     V = _mean.Length();
	 }
     // If still invalid, no gust component
     if (V > 0.0 && _gust_length_m > 0.0) {
         const double T = _gust_length_m / V;               // gust duration
         const double t0 = 26.0;                            // gust start time (s)

         double wg = 0.0;
         if (t >= t0 && t <= t0 + T) {
             // amplitude from given formula
             const double A = 17.07 * std::pow((_gust_length_m / 212.28), 1.6) / 2.0;
             const double tau = t - t0;
             wg = A * (1.0 - std::cos(2.0 * GZ_PI * tau / T));
         }

         Vector3d dir = _direction;
         if (dir.Length() < 1e-6) {
             dir.Set(1, 0, 0);
         } else {
             dir.Normalize();
         }
         wind += dir * wg;
     }
    }

    // ======== Add spatial wind component (NEW) ========
    if (_spatial_model != "none" && !_tracked_model.empty()) {
        Vector3d pos;
        if (GetTrackedPosition(_ecm, pos)) {
            Vector3d spatial_wind = ComputeSpatialWind(pos);
            wind += spatial_wind;
        }
    }

     // Update component
     auto windVel = _ecm.Component<components::WorldLinearVelocity>(_windEntity);
     if (!windVel) {
	 _ecm.CreateComponent(_windEntity, components::WorldLinearVelocity(wind));
     } else {
	 windVel->Data() = wind;
     }

     // Publish debug topic
     gz::msgs::Vector3d msg;
     gz::msgs::Set(&msg, wind);
     _pub.Publish(msg);
 }

/////////////////////////////////////////////////
// ======== Spatial Wind Helper Functions (NEW) ========
/////////////////////////////////////////////////

bool WindGustSystem::GetTrackedPosition(EntityComponentManager &_ecm, Vector3d &pos)
{
    return GetModelPosition(_ecm, _tracked_model, _tracked_entity, _warnedMissingModel, pos);
}

bool WindGustSystem::GetModelPosition(EntityComponentManager &_ecm,
                                      const std::string &model_name,
                                      gz::sim::Entity &entity_cache,
                                      bool &warned,
                                      Vector3d &pos)
{
    if (model_name.empty()) {
        return false;
    }

    // Try to find model if not already found
    if (entity_cache == kNullEntity) {
        entity_cache = _ecm.EntityByComponents(
            components::Name(model_name),
            components::Model());

        if (entity_cache == kNullEntity && !warned) {
            gzdbg << "WindGustSystem: Model '" << model_name
                  << "' not found for wind tracking" << std::endl;
            warned = true;
            return false;
        }
    }

    if (entity_cache == kNullEntity) {
        return false;
    }

    // Get world pose of the model
    auto worldPoseComp = _ecm.Component<components::WorldPose>(entity_cache);
    if (worldPoseComp) {
        pos = worldPoseComp->Data().Pos();
        return true;
    }

    // Fallback: try Pose component and use worldPose() helper
    auto poseComp = _ecm.Component<components::Pose>(entity_cache);
    if (poseComp) {
        pos = worldPose(entity_cache, _ecm).Pos();
        return true;
    }

    return false;
}

bool WindGustSystem::LoadCsvWind(const std::string &path)
{
    _csv_time_s.clear();
    _csv_wind.clear();

    std::ifstream file(path);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    if (!std::getline(file, line)) {
        return false;
    }

    int idx_time = -1;
    int idx_wind_n = -1;
    int idx_wind_e = -1;
    {
        std::stringstream header(line);
        std::string col;
        int idx = 0;
        while (std::getline(header, col, ',')) {
            if (col == "time_s") {
                idx_time = idx;
            } else if (col == "windN") {
                idx_wind_n = idx;
            } else if (col == "windE") {
                idx_wind_e = idx;
            }
            ++idx;
        }
    }

    if (idx_time < 0 || idx_wind_n < 0 || idx_wind_e < 0) {
        gzwarn << "WindGustSystem: csv header missing required columns (time_s, windN, windE)" << std::endl;
        return false;
    }

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream row(line);
        std::string cell;
        int idx = 0;
        double time_s = 0.0;
        double wind_n = 0.0;
        double wind_e = 0.0;
        while (std::getline(row, cell, ',')) {
            if (idx == idx_time) {
                time_s = std::stod(cell);
            } else if (idx == idx_wind_n) {
                wind_n = std::stod(cell);
            } else if (idx == idx_wind_e) {
                wind_e = std::stod(cell);
            }
            ++idx;
        }
        _csv_time_s.push_back(time_s);
        _csv_wind.emplace_back(wind_n, wind_e, 0.0);
    }

    if (_csv_time_s.size() < 2) {
        gzwarn << "WindGustSystem: csv wind data too short" << std::endl;
        _csv_time_s.clear();
        _csv_wind.clear();
        return false;
    }

    return true;
}

Vector3d WindGustSystem::SampleCsvWind(double t_s) const
{
    if (_csv_time_s.empty()) {
        return Vector3d::Zero;
    }

    double t = t_s + _csv_time_offset_s;
    const double t_start = _csv_time_s.front();
    const double t_end = _csv_time_s.back();
    const double duration = t_end - t_start;

    if (_csv_loop && duration > 1e-6) {
        t = std::fmod(t - t_start, duration);
        if (t < 0) {
            t += duration;
        }
        t += t_start;
    } else {
        if (t <= t_start) {
            return _csv_wind.front();
        }
        if (t >= t_end) {
            return _csv_wind.back();
        }
    }

    auto it = std::lower_bound(_csv_time_s.begin(), _csv_time_s.end(), t);
    if (it == _csv_time_s.begin()) {
        return _csv_wind.front();
    }
    if (it == _csv_time_s.end()) {
        return _csv_wind.back();
    }

    const size_t idx1 = static_cast<size_t>(std::distance(_csv_time_s.begin(), it) - 1);
    const size_t idx2 = idx1 + 1;
    const double t1 = _csv_time_s[idx1];
    const double t2 = _csv_time_s[idx2];
    const double denom = (t2 - t1) > 1e-9 ? (t2 - t1) : 1.0;
    const double alpha = (t - t1) / denom;
    const Vector3d &w1 = _csv_wind[idx1];
    const Vector3d &w2 = _csv_wind[idx2];
    return w1 + (w2 - w1) * alpha;
}

Vector3d WindGustSystem::ComputeSpatialWind(const Vector3d &pos)
{
    Vector3d spatial_wind(0, 0, 0);

    if (_spatial_model == "boundary_layer") {
        // Power law boundary layer profile: V(z) = V_ref * (z/z_ref)^α
        double z = pos.Z();
        if (z > 0.1 && _bl_ref_height > 0.1) {  // Avoid singularity at z=0
            double ratio = std::pow(z / _bl_ref_height, _bl_exponent);
            spatial_wind = _bl_ref_wind * ratio;
        } else if (z <= 0.1) {
            // Below 0.1m, use ground level (z=0.1m) wind
            double ratio = std::pow(0.1 / _bl_ref_height, _bl_exponent);
            spatial_wind = _bl_ref_wind * ratio;
        }
    }

    return spatial_wind;
}
