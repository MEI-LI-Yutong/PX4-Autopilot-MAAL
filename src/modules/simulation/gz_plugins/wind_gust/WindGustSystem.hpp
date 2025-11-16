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

 #include <gz/sim/System.hh>
 #include <gz/sim/Entity.hh>
 #include <gz/math/Vector3.hh>
 #include <gz/transport/Node.hh>
 #include <chrono>
 #include <string>
 #include <sdf/sdf.hh>
#include <gz/transport/Publisher.hh>
#include <gz/math/Helpers.hh>
#include <cmath>
#include <random>

 namespace custom
 {
 class WindGustSystem :
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
     // ======== Temporal wind parameters (original) ========
     gz::math::Vector3d _mean{0, 0, 0};
     gz::math::Vector3d _amplitude{0, 0, 0};
     double _frequency_hz{0.0};
    double _phase_rad{0.0};
    std::string _model{"one_minus_cos"};
    // 1-cos gust parameters
    double _gust_length_m{30.0};     // l_g
    double _airspeed_ms{-1.0};       // V_inf (<=0 means derive from |mean|)
    gz::math::Vector3d _direction{1, 0, 0};

    // one_minus_cos_simp parameters: v = (A0/2) * [1 - cos(2*pi*t/T)]
    double _simp_A0{0.0};            // peak gust magnitude (m/s)
    double _simp_T{10.0};            // period (s)

    // Dryden model parameters
    gz::math::Vector3d _dryden_sigma{1.0, 1.0, 1.0};   // (sigma_u, sigma_v, sigma_w) [m/s]
    gz::math::Vector3d _dryden_length{200.0, 200.0, 50.0}; // (Lu, Lv, Lw) [m]
    double _last_time_s{-1.0};
    // Dryden states (world axes u->X, v->Y, w->Z)
    double _xu{0.0};
    double _xv1{0.0}, _xv2{0.0};
    double _xw1{0.0}, _xw2{0.0};
    // RNG for Dryden
    std::mt19937 _rng{std::random_device{}()};
    std::normal_distribution<double> _norm{0.0, 1.0};
    bool _rng_seeded{false};
    uint32_t _rng_seed{0};

    // ======== Spatial wind parameters (NEW) ========
    std::string _spatial_model{"none"};  // supported: none, boundary_layer
    std::string _tracked_model{""};      // Name of model to track (empty = disabled)
    gz::sim::Entity _tracked_entity{gz::sim::kNullEntity};

    // Note: Previously supported linear_shear, sine_wave, and vortex have
    // been removed to keep only the boundary_layer model.

     // Boundary layer: V = V_ref · (z/z_ref)^α
     double _bl_ref_height{10.0};                     // [m]
     double _bl_exponent{0.143};                      // Power law exponent
     gz::math::Vector3d _bl_ref_wind{0, 0, 0};       // Wind at z_ref

     // State
     gz::sim::Entity _worldEntity{gz::sim::kNullEntity};
     gz::sim::Entity _windEntity{gz::sim::kNullEntity};
     bool _configured{false};
     bool _warnedMissingWind{false};
     bool _warnedMissingModel{false};
    gz::transport::Node _node;
    gz::transport::Node::Publisher _pub;
    std::string _topic;

    // ======== Spatial wind helper functions ========
    gz::math::Vector3d ComputeSpatialWind(const gz::math::Vector3d &pos);
    bool GetTrackedPosition(gz::sim::EntityComponentManager &_ecm, gz::math::Vector3d &pos);
    // Compute simple 1-cos gust at time t (seconds)
    inline double one_minus_cos_simp(double t) const {
        const double T = _simp_T;
        if (T <= 1e-6) {
            return 0.0;
        }
        // phase shift in seconds (phase_rad corresponds to 2*pi per period)
        const double t_phase = (_phase_rad / (2.0 * GZ_PI)) * T;
        double t_mod = std::fmod(t + t_phase, T);
        if (t_mod < 0) t_mod += T;
        return 0.5 * _simp_A0 * (1.0 - std::cos(2.0 * GZ_PI * t_mod / T));
    }
};
} // namespace custom
