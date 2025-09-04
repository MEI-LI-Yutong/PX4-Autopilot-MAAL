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
#include <lib/mathlib/mathlib.h>

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
	// 1) 只处理主矩阵
	if (matrix_index != 0) {
		return;
	}

	// 2) 计算 dt
	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.01f; // fallback
	if (_last_update_time > 0) {
		dt = math::constrain((now - _last_update_time) * 1e-6f, 0.001f, 0.1f);
	}
	_last_update_time = now;

	// 3) 加 offset（在 yaw 分配基础上对齐"竖直=0"）
	actuator_sp += _tilt_offsets;

	// 4) 更新来自消息的 collective（状态机：valid=true / valid=false / 超时）
	updateCollectiveTiltAngle();

	// 5) 速率限制（跟随或回零）
	updateCollectiveTiltWithRateLimit(dt);

	// 6) 保存 yaw-only 值（collective 未叠加前）
	const int tilt_count = _tilts.count();
	float yaw_only_values[ActuatorEffectivenessTilts::MAX_COUNT] {};
	for (int i = 0; i < tilt_count; ++i) {
		yaw_only_values[i] = actuator_sp(_first_tilt_idx + i);
	}

	// 7) 计算 yaw 饱和（基于 yaw-only）
	calculateYawSaturationFlags(yaw_only_values, tilt_count, actuator_min, actuator_max);

	// 8) 叠加 collective（yaw 优先裁剪）
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
	vehicle_thrust_setpoint_s vt{};
	const hrt_abstime now = hrt_absolute_time();

	if (_vt_setpoint_sub.update(&vt)) {
		if (vt.tilt_extra_angle_valid && PX4_ISFINITE(vt.tilt_extra_angle)) {
			// 有效角度命令
			if (_incremental_mode) {
				// 增量模式：目标角度 = 基准角度 + 增量角度
				_collective_tilt_target = _collective_tilt_base + vt.tilt_extra_angle;
			} else {
				// 绝对模式：直接设置目标角度
				_collective_tilt_target = vt.tilt_extra_angle;
			}

			_collective_tilt_valid  = true;
			_last_valid_command_time = now;

			// 如在回零阶段，打断回零
			if (_ramp_state == RampState::RAMP_TO_ZERO) {
				_ramp_state = RampState::NONE;
			}

		} else if (!vt.tilt_extra_angle_valid) {
			// 显式禁用：立即启动回零（若当前不为 0）
			_collective_tilt_valid = false;
			_collective_tilt_target = _collective_tilt_base; // 回到基准位置
			_last_valid_command_time = now;

			if (fabsf(_collective_tilt_angle - _collective_tilt_base) > 1e-3f) {
				_ramp_state = RampState::RAMP_TO_ZERO;
			} else {
				_collective_tilt_angle = _collective_tilt_base;
				_ramp_state = RampState::NONE;
			}
		}
	}

	// 超时：上一次有效命令后超过阈值，自动禁用并回零
	if (_collective_tilt_valid && (now - _last_valid_command_time > COLLECTIVE_TIMEOUT_US)) {
		_collective_tilt_valid = false;
		_collective_tilt_target = _collective_tilt_base; // 回到基准位置

		if (fabsf(_collective_tilt_angle - _collective_tilt_base) > 1e-3f) {
			_ramp_state = RampState::RAMP_TO_ZERO;
		} else {
			_collective_tilt_angle = _collective_tilt_base;
			_ramp_state = RampState::NONE;
		}
	}
}

void ActuatorEffectivenessMCTilt::updateCollectiveTiltWithRateLimit(float dt)
{
	if (!_collective_tilt_valid && _ramp_state == RampState::NONE) {
		return;
	}

		// Choose rate limit based on current state
	float rate_limit;
	if (_ramp_state == RampState::RAMP_TO_ZERO ||
	    (fabsf(_collective_tilt_target - _collective_tilt_base) < FLT_EPSILON &&
	     fabsf(_collective_tilt_angle - _collective_tilt_base) > FLT_EPSILON)) {
		rate_limit = COLLECTIVE_TILT_RATE_RETURN_RAD_S; // Faster return to base
	} else {
		rate_limit = COLLECTIVE_TILT_RATE_MAX_RAD_S;    // Normal rate limit
	}

	// Apply rate limiting
	const float max_change = rate_limit * dt;
	const float angle_error = _collective_tilt_target - _collective_tilt_angle;
	const float angle_change = math::constrain(angle_error, -max_change, max_change);

	_collective_tilt_angle += angle_change;

	// Check if ramping to base is complete
	if (_ramp_state == RampState::RAMP_TO_ZERO && fabsf(_collective_tilt_angle - _collective_tilt_base) < 0.01f) {
		_collective_tilt_angle = _collective_tilt_base;
		_collective_tilt_valid = false;
		_ramp_state = RampState::NONE;
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

	if (!_collective_tilt_valid && _ramp_state == RampState::NONE) {
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

		// Use the new normalization function
		float collective_normalized = normalizeAngleToActuator(_collective_tilt_angle, min_angle, max_angle);

		// Calculate available margins around yaw base value (yaw priority allocation)
		const float yaw_base = yaw_only_values[i];
		const float margin_positive = actuator_max(actuator_idx) - yaw_base;
		const float margin_negative = yaw_base - actuator_min(actuator_idx);

		// Clamp collective command to available margins (preserves yaw control authority)
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

float ActuatorEffectivenessMCTilt::normalizeAngleToActuator(float angle_rad, float min_angle_rad, float max_angle_rad) const
{
	// Avoid division by zero
	if (fabsf(max_angle_rad - min_angle_rad) < FLT_EPSILON) {
		return 0.0f;
	}

	// Constrain angle to servo limits
	const float constrained_angle = math::constrain(angle_rad, min_angle_rad, max_angle_rad);

	// Normalize to [-1, 1] actuator space
	float normalized = 2.0f * (constrained_angle - min_angle_rad) / (max_angle_rad - min_angle_rad) - 1.0f;

	// Final constraint to ensure [-1, 1] range
	return math::constrain(normalized, -1.0f, 1.0f);
}
