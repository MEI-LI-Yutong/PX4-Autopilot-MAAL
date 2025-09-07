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
	actuator_sp += _tilt_offsets;

	// ====== 新增：Fx 增量层（在分配完成后叠加 Δθ） ======
	_fx_residual = 0.f;
	
	// 清空log数据
	_tilt_log_data = {};

	if (matrix_index == 0 && _tilts.count() > 0) {

		const auto &geom = _mc_rotors.geometry();

		// 收集所有由"朝前倾"舵机控制的电机
		struct Item { int motor; int tilt; float fi; float y; float old_tilt_angle; };
		Item items[ActuatorEffectivenessRotors::NUM_ROTORS_MAX];
		int N = 0;

		// 记录舵机索引映射 (用于log记录)
		int tilt_fl_idx = -1, tilt_fr_idx = -1, tilt_tail_idx = -1;

		for (int m = 0; m < geom.num_rotors; ++m) {
			const auto &r = geom.rotors[m];
			const int t = r.tilt_index;
			if (t < 0 || t >= _tilts.count()) continue;
			if (_tilts.config(t).tilt_direction != ActuatorEffectivenessTilts::TiltDirection::TowardsFront) continue;

			const float u0 = actuator_sp(m);        // 当前分配后电机归一化输出
			const float ct = r.thrust_coef;
			const float fi = ct * u0;               // 小角近似推力幅值
			const float y  = r.position(1);

			const int servo_col = _first_tilt_idx + t;
			if (servo_col < 0 || servo_col >= NUM_ACTUATORS) continue;
			const float old_angle = servoNormalizedToAngle(t, actuator_sp(servo_col));

			items[N++] = {m, t, fi, y, old_angle};

			// 根据Y坐标判断舵机组别 (用于log记录)
			if (fabsf(y) < 0.05f) { // 尾部
				tilt_tail_idx = t;
				_tilt_log_data.theta_tail_old = old_angle;
				_tilt_log_data.servo_tail_old = actuator_sp(servo_col);
			} else if (y > 0.f) { // 前左
				tilt_fl_idx = t;
				_tilt_log_data.theta_fl_old = old_angle;
				_tilt_log_data.servo_fl_old = actuator_sp(servo_col);
			} else { // 前右
				tilt_fr_idx = t;
				_tilt_log_data.theta_fr_old = old_angle;
				_tilt_log_data.servo_fr_old = actuator_sp(servo_col);
			}

			if (N >= ActuatorEffectivenessRotors::NUM_ROTORS_MAX) break;
		}

		// 计算 Σ f_i 与 Fx_cmd, Fz_cmd
		float F_total = 0.f;
		for (int i = 0; i < N; ++i) F_total += items[i].fi;

		const float Fx_cmd = control_sp(3);
		const float Fz_cmd = control_sp(5); // 注意：使用 control_sp(5) 作为归一化基准

		// 记录基本日志数据
		_tilt_log_data.fx_cmd = Fx_cmd;
		_tilt_log_data.fz_cmd = Fz_cmd;
		_tilt_log_data.f_total = F_total;
		_tilt_log_data.num_motors = N;

		if (N > 0 && PX4_ISFINITE(Fx_cmd) && PX4_ISFINITE(Fz_cmd) && fabsf(Fz_cmd) > EPS_F) {
			// 公共增量 Δθ（小角线性）
			// 注意：z轴朝下为正，前向倾转需要负角度来产生正向Fx
			// 使用 Fz_cmd 作为归一化基准：Δθ = -Fx_cmd / Fz_cmd
			float delta_theta = -Fx_cmd / Fz_cmd;
			delta_theta = math::constrain(delta_theta, -MAX_LINEAR_TILT_RAD, MAX_LINEAR_TILT_RAD);
			
			_tilt_log_data.delta_theta = delta_theta;

			// 对每个唯一 tilt 写一次目标角，然后给其下挂电机做垂直补偿
			bool tilt_written[ActuatorEffectivenessTilts::MAX_COUNT] {};
			float tilt_new_angle[ActuatorEffectivenessTilts::MAX_COUNT] {};

			for (int i = 0; i < N; ++i) {
				const int t = items[i].tilt;
				const int servo_col = _first_tilt_idx + t;

				if (!tilt_written[t]) {
					// 当前归一化舵机值
					const float current_normalized = actuator_sp(servo_col);
					
					// 将角度增量转换为归一化增量
					const float delta_normalized = angleToServoNormalized(t, delta_theta) - angleToServoNormalized(t, 0.f);
					
					// 目标归一化值 = 当前归一化值 + 归一化增量
					const float target_normalized = math::constrain(current_normalized + delta_normalized, -1.f, 1.f);
					
					// 应用限幅
					actuator_sp(servo_col) = target_normalized;

					// 读回实际角（包含量化/限幅）
					tilt_new_angle[t] = servoNormalizedToAngle(t, actuator_sp(servo_col));
					tilt_written[t] = true;

					// 记录具体舵机的角度和归一化值
					if (t == tilt_fl_idx) {
						_tilt_log_data.theta_fl_new = tilt_new_angle[t];
						_tilt_log_data.servo_fl_new = actuator_sp(servo_col);
					} else if (t == tilt_fr_idx) {
						_tilt_log_data.theta_fr_new = tilt_new_angle[t];
						_tilt_log_data.servo_fr_new = actuator_sp(servo_col);
					} else if (t == tilt_tail_idx) {
						_tilt_log_data.theta_tail_new = tilt_new_angle[t];
						_tilt_log_data.servo_tail_new = actuator_sp(servo_col);
					}
				}
			}

			// 逐电机垂直补偿 + Fx 实现值累计
			float Fx_real = 0.f;

			for (int i = 0; i < N; ++i) {
				const int m = items[i].motor;
				const int t = items[i].tilt;

				const float old_a = items[i].old_tilt_angle;
				const float new_a = tilt_new_angle[t];

				// 保持 Fz：scale = cos(old)/cos(new)
				float scale = 1.f;
				const float c_old = cosf(old_a);
				const float c_new = cosf(new_a);
				if (fabsf(c_new) > 1e-4f) {
					scale = c_old / c_new;
				}

				float cmd = actuator_sp(m) * scale;
				cmd = math::constrain(cmd, actuator_min(m), actuator_max(m));
				actuator_sp(m) = cmd;

				// Fx 贡献（线性）：f_i * Δθ_i_real
				Fx_real += items[i].fi * (new_a - old_a);
			}

			_fx_residual = Fx_cmd - Fx_real;
			_tilt_log_data.fx_real = Fx_real;
			
			// 提示：若几何/推力不对称，加同一 Δθ 可能产生轻微 yaw 扰动，这里不抑制，
			// 上层下一周期会吸收。需要的话可估算 ΔMz 并注入 status.unallocated_torque[2]。
		}
	}
	// ====== Fx 增量层结束 ======

	bool yaw_saturated_positive = true;
	bool yaw_saturated_negative = true;

	for (int i = 0; i < _tilts.count(); ++i) {

		// custom yaw saturation logic: only declare yaw saturated if all tilts are at the negative or positive yawing limit
		if (_tilts.getYawTorqueOfTilt(i) > FLT_EPSILON) {

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

		} else if (_tilts.getYawTorqueOfTilt(i) < -FLT_EPSILON) {
			if (yaw_saturated_negative && actuator_sp(i + _first_tilt_idx) < actuator_max(i + _first_tilt_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && actuator_sp(i + _first_tilt_idx) > actuator_min(i + _first_tilt_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

void ActuatorEffectivenessMCTilt::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	// 新增：把 Fx 残差注入，便于上层积分
	if (matrix_index == 0) {
		status.unallocated_thrust[0] += _fx_residual;

		// 填充倾转舵机调试数据到 control_allocator_status
		status.tilt_fx_cmd = _tilt_log_data.fx_cmd;
		status.tilt_fz_cmd = _tilt_log_data.fz_cmd;
		status.tilt_f_total = _tilt_log_data.f_total;
		status.tilt_delta_theta = _tilt_log_data.delta_theta;
		status.tilt_theta_fl_old = _tilt_log_data.theta_fl_old;
		status.tilt_theta_fr_old = _tilt_log_data.theta_fr_old;
		status.tilt_theta_tail_old = _tilt_log_data.theta_tail_old;
		status.tilt_theta_fl_new = _tilt_log_data.theta_fl_new;
		status.tilt_theta_fr_new = _tilt_log_data.theta_fr_new;
		status.tilt_theta_tail_new = _tilt_log_data.theta_tail_new;
		status.tilt_servo_fl_old = _tilt_log_data.servo_fl_old;
		status.tilt_servo_fr_old = _tilt_log_data.servo_fr_old;
		status.tilt_servo_tail_old = _tilt_log_data.servo_tail_old;
		status.tilt_servo_fl_new = _tilt_log_data.servo_fl_new;
		status.tilt_servo_fr_new = _tilt_log_data.servo_fr_new;
		status.tilt_servo_tail_new = _tilt_log_data.servo_tail_new;
		status.tilt_fx_real = _tilt_log_data.fx_real;
		status.tilt_num_motors = static_cast<uint8_t>(_tilt_log_data.num_motors);
	}

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

// ===== Helpers: angle <-> normalized [-1, 1] =====
float ActuatorEffectivenessMCTilt::angleToServoNormalized(int tilt_index, float theta) const
{
	if (tilt_index < 0 || tilt_index >= _tilts.count()) return 0.f;
	const auto &cfg = _tilts.config(tilt_index);
	const float min_a = cfg.min_angle; // radians
	const float max_a = cfg.max_angle;
	const float delta = max_a - min_a;
	if (delta < 1e-5f) return 0.f;
	const float t = math::constrain(theta, min_a, max_a);
	const float s = 2.f * (t - min_a) / delta - 1.f;
	return math::constrain(s, -1.f, 1.f);
}

float ActuatorEffectivenessMCTilt::servoNormalizedToAngle(int tilt_index, float normalized) const
{
	if (tilt_index < 0 || tilt_index >= _tilts.count()) return 0.f;
	const auto &cfg = _tilts.config(tilt_index);
	const float min_a = cfg.min_angle;
	const float max_a = cfg.max_angle;
	const float delta = max_a - min_a;
	const float n = math::constrain(normalized, -1.f, 1.f);
	return min_a + (n + 1.f) * 0.5f * delta;
}
