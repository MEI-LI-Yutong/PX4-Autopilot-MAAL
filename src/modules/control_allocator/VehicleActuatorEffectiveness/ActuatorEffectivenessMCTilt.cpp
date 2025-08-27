/****************************************************************************
 *
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectivenessMCTilt.hpp"

using namespace matrix;

ActuatorEffectivenessMCTilt::ActuatorEffectivenessMCTilt(ModuleParams *parent)
	: ModuleParams(parent),
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::FixedUpwards, true),
	  _tilts(this)
{
}

bool
ActuatorEffectivenessMCTilt::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE) {
		return false;
	}

	// MC motors
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset such that tilts point upwards when control input == 0 (trim is 0 if min_angle == -max_angle).
	// Note that we don't set configuration.trim here, because in the case of trim == +-1, yaw is always saturated
	// and reduced to 0 with the sequential desaturation method. Instead we add it after.
	_tilt_offsets.setZero();

	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	return (rotors_added_successfully && tilts_added_successfully);
}

void ActuatorEffectivenessMCTilt::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp,
		int matrix_index, ActuatorVector &actuator_sp, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	// Apply tilt offsets first
	actuator_sp += _tilt_offsets;

	// Update collective tilt angle from vehicle thrust setpoint
	updateCollectiveTiltAngle();

	// Save yaw-only values before adding collective tilt for saturation detection
	const int tilt_count = _tilts.count();
	float yaw_only_values[ActuatorEffectivenessTilts::MAX_COUNT];

	for (int i = 0; i < tilt_count; ++i) {
		yaw_only_values[i] = actuator_sp(_first_tilt_idx + i);
	}

	// Calculate yaw saturation flags based on yaw-only values (unaffected by collective tilt)
	calculateYawSaturationFlags(yaw_only_values, tilt_count, actuator_min, actuator_max);

	// Apply collective tilt control to forward-tilting servos
	applyCollectiveTilt(actuator_sp, yaw_only_values, tilt_count, actuator_min, actuator_max);
}

void ActuatorEffectivenessMCTilt::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// Note: the values '-1', '1' and '0' are just to indicate a negative,
	// positive or no saturation to the rate controller. The actual magnitude is not used.
	if (_yaw_tilt_saturation_flags.tilt_yaw_pos) {
		status.unallocated_torque[2] = 1.f;

	} else if (_yaw_tilt_saturation_flags.tilt_yaw_neg) {
		status.unallocated_torque[2] = -1.f;

	} else {
		status.unallocated_torque[2] = 0.f;
	}
}

void ActuatorEffectivenessMCTilt::updateCollectiveTiltAngle()
{
	vehicle_thrust_setpoint_s vt;

	if (_vt_setpoint_sub.update(&vt)) {
		if (PX4_ISFINITE(vt.tilt_extra_angle)) {
			_collective_tilt_angle = vt.tilt_extra_angle;
			_collective_tilt_valid = true;

		} else {
			_collective_tilt_valid = false;
		}
	}
}

void ActuatorEffectivenessMCTilt::calculateYawSaturationFlags(const float yaw_only_values[], int tilt_count,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	bool yaw_saturated_positive = true;
	bool yaw_saturated_negative = true;

	for (int i = 0; i < tilt_count; ++i) {
		const int actuator_idx = i + _first_tilt_idx;
		const float yaw_torque = _tilts.getYawTorqueOfTilt(i);

		// Custom yaw saturation logic: only declare yaw saturated if all tilts 
		// are at the negative or positive yawing limit
		if (yaw_torque > FLT_EPSILON) {
			if (yaw_saturated_positive && yaw_only_values[i] < actuator_max(actuator_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && yaw_only_values[i] > actuator_min(actuator_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

		} else if (yaw_torque < -FLT_EPSILON) {
			if (yaw_saturated_negative && yaw_only_values[i] < actuator_max(actuator_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && yaw_only_values[i] > actuator_min(actuator_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

void ActuatorEffectivenessMCTilt::applyCollectiveTilt(ActuatorVector &actuator_sp, const float yaw_only_values[],
		int tilt_count, const matrix::Vector<float, NUM_ACTUATORS> &actuator_min,
		const matrix::Vector<float, NUM_ACTUATORS> &actuator_max)
{
	_collective_was_clipped = false;

	if (!_collective_tilt_valid) {
		return;
	}

	for (int i = 0; i < tilt_count; ++i) {
		const auto &tilt_config = _tilts.config(i);

		// Only apply collective tilt to forward tilting servos
		if (tilt_config.tilt_direction != ActuatorEffectivenessTilts::TiltDirection::TowardsFront) {
			continue;
		}

		const int actuator_idx = i + _first_tilt_idx;
		const float min_angle = tilt_config.min_angle;
		const float max_angle = tilt_config.max_angle;

		// Avoid division by zero
		if (fabsf(max_angle - min_angle) < FLT_EPSILON) {
			continue;
		}

		// Constrain angle to servo limits
		const float constrained_angle = math::constrain(_collective_tilt_angle, min_angle, max_angle);

		// Normalize to [-1, 1] servo space
		float collective_normalized = 2.0f * (constrained_angle - min_angle) / (max_angle - min_angle) - 1.0f;
		collective_normalized = math::constrain(collective_normalized, -1.0f, 1.0f);

		// Calculate available margins around yaw base value
		const float yaw_base = yaw_only_values[i];
		const float margin_positive = actuator_max(actuator_idx) - yaw_base;
		const float margin_negative = yaw_base - actuator_min(actuator_idx);

		// Clamp collective command to available margins
		const float collective_clamped = math::constrain(collective_normalized, -margin_negative, margin_positive);

		if (fabsf(collective_clamped - collective_normalized) > FLT_EPSILON) {
			_collective_was_clipped = true;
		}

		// Apply collective tilt command
		actuator_sp(actuator_idx) = yaw_base + collective_clamped;

		// Final safety clamp
		actuator_sp(actuator_idx) = math::constrain(actuator_sp(actuator_idx), 
			actuator_min(actuator_idx), actuator_max(actuator_idx));
	}
}
