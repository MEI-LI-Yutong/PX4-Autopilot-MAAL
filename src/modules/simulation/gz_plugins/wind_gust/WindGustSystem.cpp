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
#include <gz/sim/Util.hh>

#include <gz/math/Vector3.hh>
#include <gz/math/Helpers.hh>

using namespace gz::sim;
using namespace gz::math;
using namespace custom;

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
    }

    // Try to resolve wind entity now; lazily fallback during updates
    _windEntity = _ecm.EntityByComponents(components::Wind());

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

    // Compute wind vector at sim time (sinusoidal model for now)
    const double t = std::chrono::duration<double>(_info.simTime).count();
    Vector3d wind = _mean;

    if (_frequency_hz > 0.0) {
        const double omega = 2.0 * GZ_PI * _frequency_hz;
        wind.X() += _amplitude.X() * std::sin(omega * t + _phase_rad);
        wind.Y() += _amplitude.Y() * std::sin(omega * t + _phase_rad);
        wind.Z() += _amplitude.Z() * std::sin(omega * t + _phase_rad);
    } else {
        wind += _amplitude;
    }

    // Update component
    auto windVel = _ecm.Component<components::WorldLinearVelocity>(_windEntity);
    if (!windVel) {
        _ecm.CreateComponent(_windEntity, components::WorldLinearVelocity(wind));
    } else {
        windVel->Data() = wind;
    }
}

