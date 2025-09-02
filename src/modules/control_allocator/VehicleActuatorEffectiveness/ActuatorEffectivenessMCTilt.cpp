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
	  _mc_rotors(this, ActuatorEffectivenessRotors::AxisConfiguration::Configurable, true), // 可配置轴 + tilt 支持
	  _tilts(this)
{
}

bool ActuatorEffectivenessMCTilt::handleUtrimMessage()
{
	utrim_s u{};
	if (_utrim_sub.update(&u) && u.valid) {
		// 计算平均 tilt servo 归一化值：normalized_values[3..]
		int tilt_cnt = _tilts.count();
		if (tilt_cnt <= 0) {
			return false;
		}

		float sum = 0.f;
		int used = 0;
		// utrim.normalized_values 有6个元素，前3个是电机，后3个是舵机
		for (int i = 0; i < tilt_cnt && (3 + i) < 6; ++i) {
			float n = u.normalized_values[3 + i]; // 期望 ∈ [-1,1]
			if (PX4_ISFINITE(n)) {
				sum += n;
				++used;
			}
		}

		if (used == 0) {
			return false;
		}

		float collective_norm = sum / (float)used;
		collective_norm = math::constrain(collective_norm, -1.f, 1.f);

		const hrt_abstime now = hrt_absolute_time();
		bool angle_change = !PX4_ISFINITE(_last_collective_norm) ||
		                    fabsf(collective_norm - _last_collective_norm) > 0.01f;
		bool time_ok = (now - _last_dynamic_update) > MIN_DYNAMIC_INTERVAL_US;

		if ((_config_phase == false) && (angle_change || time_ok)) {
			_dynamic_update_pending = true;
		}

		_last_collective_norm = collective_norm;

		// 缓存 utrim 数据
		_last_utrim = u;
		_have_utrim = true;

		return true;
	}

	return false;
}

void ActuatorEffectivenessMCTilt::updateRotorAxisFromCollective(float collective_norm)
{
	if (!PX4_ISFINITE(collective_norm)) {
		collective_norm = -1.f; // 默认竖直
	}
	_mc_rotors.updateAxisFromTilts(_tilts, collective_norm);
}

void ActuatorEffectivenessMCTilt::buildActuatorTrimAndControlTrim(Configuration &c)
{
	const int sel = c.selected_matrix;
	const int n_act = c.num_actuators_matrix[sel];

	// 初始化
	c.linearization_point[sel].setZero();
	c.trim[sel].setZero();
	c.control_trim[sel].setZero();

	// 构建 actuator_trim
	for (int j = 0; j < n_act; j++) {
		float raw_val = 0.f;

		// 如果有 utrim 数据，使用对应的值
		if (_have_utrim) {
			if (j < _mc_rotors.geometry().num_rotors && j < 3) {
				// 电机值
				raw_val = _last_utrim.normalized_values[j];
			} else if (j >= _first_tilt_idx && j < _first_tilt_idx + _tilts.count() && (j - _first_tilt_idx) < 3) {
				// 倾转舵机值
				raw_val = _last_utrim.normalized_values[3 + (j - _first_tilt_idx)];
			}
		}

		// 加上 tilt_offsets（统一在这里加，避免重复）
		c.trim[sel](j) = raw_val + _tilt_offsets(j);
		c.linearization_point[sel](j) = c.trim[sel](j);
	}

	// 计算 control_trim = E * actuator_trim
	for (int axis = 0; axis < NUM_AXES; axis++) {
		float sum = 0.f;
		for (int j = 0; j < n_act; j++) {
			sum += c.effectiveness_matrices[sel](axis, j) * c.trim[sel](j);
		}
		c.control_trim[sel](axis) = sum;
	}
}

bool
ActuatorEffectivenessMCTilt::getEffectivenessMatrix(Configuration &configuration,
		EffectivenessUpdateReason external_update)
{
	// CONFIGURATION_UPDATE：第一次用竖直轴向建立归一化尺度
	if (external_update == EffectivenessUpdateReason::CONFIGURATION_UPDATE) {
		updateRotorAxisFromCollective(-1.f); // 竖直轴向

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

		buildActuatorTrimAndControlTrim(configuration);
		_config_phase = false;
		_last_dynamic_update = hrt_absolute_time();
		_dynamic_update_pending = false;

		return (rotors_added_successfully && tilts_added_successfully);
	}

	// 非配置更新：处理动态更新
	if (external_update == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE && !_dynamic_update_pending) {
		return false;
	}

	// 执行动态更新
	float applied_norm = (_dynamic_update_pending) ? _last_collective_norm : _last_collective_norm;

	updateRotorAxisFromCollective(applied_norm);

	// MC motors
	_mc_rotors.enableYawByDifferentialThrust(!_tilts.hasYawControl());
	const bool rotors_added_successfully = _mc_rotors.addActuators(configuration);

	// Tilts
	_first_tilt_idx = configuration.num_actuators_matrix[0];
	_tilts.updateTorqueSign(_mc_rotors.geometry());
	const bool tilts_added_successfully = _tilts.addActuators(configuration);

	// Set offset
	_tilt_offsets.setZero();
	for (int i = 0; i < _tilts.count(); ++i) {
		float delta_angle = _tilts.config(i).max_angle - _tilts.config(i).min_angle;

		if (delta_angle > FLT_EPSILON) {
			float trim = -1.f - 2.f * _tilts.config(i).min_angle / delta_angle;
			_tilt_offsets(_first_tilt_idx + i) = trim;
		}
	}

	buildActuatorTrimAndControlTrim(configuration);

	_last_dynamic_update = hrt_absolute_time();
	_dynamic_update_pending = false;

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

	// 3) offset 已经集成到 actuator_trim 中，不需要重复添加

	// 6) 保存 yaw-only 值（collective 未叠加前）
	const int tilt_count = _tilts.count();
	float yaw_only_values[ActuatorEffectivenessTilts::MAX_COUNT] {};
	for (int i = 0; i < tilt_count; ++i) {
		yaw_only_values[i] = actuator_sp(_first_tilt_idx + i);
	}

	// 7) 计算 yaw 饱和（基于 yaw-only）
	calculateYawSaturationFlags(yaw_only_values, tilt_count, actuator_min, actuator_max);

	// NOTE: 不再在这里叠加 collective tilt，因为它已经通过动态轴向更新包含在效果矩阵中
	// 原有的 applyCollectiveTilt 功能已经被 updateRotorAxisFromCollective 替代
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
			_collective_tilt_target = vt.tilt_extra_angle;
			_collective_tilt_valid  = true;
			_last_valid_command_time = now;

			// 如在回零阶段，打断回零
			if (_ramp_state == RampState::RAMP_TO_ZERO) {
				_ramp_state = RampState::NONE;
			}

		} else if (!vt.tilt_extra_angle_valid) {
			// 显式禁用：立即启动回零（若当前不为 0）
			_collective_tilt_valid = false;
			_collective_tilt_target = 0.f;
			_last_valid_command_time = now;

			if (fabsf(_collective_tilt_angle) > 1e-3f) {
				_ramp_state = RampState::RAMP_TO_ZERO;
			} else {
				_collective_tilt_angle = 0.f;
				_ramp_state = RampState::NONE;
			}
		}
	}

	// 超时：上一次有效命令后超过阈值，自动禁用并回零
	if (_collective_tilt_valid && (now - _last_valid_command_time > COLLECTIVE_TIMEOUT_US)) {
		_collective_tilt_valid = false;
		_collective_tilt_target = 0.f;

		if (fabsf(_collective_tilt_angle) > 1e-3f) {
			_ramp_state = RampState::RAMP_TO_ZERO;
		} else {
			_collective_tilt_angle = 0.f;
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
	    (fabsf(_collective_tilt_target) < FLT_EPSILON && fabsf(_collective_tilt_angle) > FLT_EPSILON)) {
		rate_limit = COLLECTIVE_TILT_RATE_RETURN_RAD_S; // Faster return to zero
	} else {
		rate_limit = COLLECTIVE_TILT_RATE_MAX_RAD_S;    // Normal rate limit
	}

	// Apply rate limiting
	const float max_change = rate_limit * dt;
	const float angle_error = _collective_tilt_target - _collective_tilt_angle;
	const float angle_change = math::constrain(angle_error, -max_change, max_change);

	_collective_tilt_angle += angle_change;

	// Check if ramping to zero is complete
	if (_ramp_state == RampState::RAMP_TO_ZERO && fabsf(_collective_tilt_angle) < 0.01f) {
		_collective_tilt_angle = 0.0f;
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

		// 直接将增量角度归一化（相对于舵机范围的一半）
		// _collective_tilt_angle 是相对于无倾转状态（中位）的增量角度
		float collective_normalized = _collective_tilt_angle / ((max_angle - min_angle) / 2.0f);
		collective_normalized = math::constrain(collective_normalized, -1.0f, 1.0f);

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

void ActuatorEffectivenessMCTilt::checkForDynamicUpdates()
{
	handleUtrimMessage();
}
