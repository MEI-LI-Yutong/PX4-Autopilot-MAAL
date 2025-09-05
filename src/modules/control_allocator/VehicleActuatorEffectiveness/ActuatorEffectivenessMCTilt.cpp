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
	_fx_residual = 0.f;

	if (matrix_index == 0 && ENABLE_LINEAR_FX_YAW_ENERGY && _tilts.count() > 0) {
		// lazyInitTailTiltSubscription();

		const auto &geom = _mc_rotors.geometry();

		float f_fl = 0.f, f_fr = 0.f, f_tail = 0.f;
		float y_fl_mean = 0.f, y_fr_mean = 0.f;
		int fl_cnt=0, fr_cnt=0, tail_cnt=0;

		int tilt_idx_fl = -1, tilt_idx_fr = -1, tilt_idx_tail = -1;

		static constexpr int MAX_R = ActuatorEffectivenessRotors::NUM_ROTORS_MAX;
		Group motor_group[MAX_R];
		for (int i=0;i<MAX_R;i++) motor_group[i]=Group::None;

		// 收集当前各倾转电机的近似推力 f_i = ct_i * u_i
		for (int m=0; m<geom.num_rotors; ++m) {
			const auto &r = geom.rotors[m];
			if (r.tilt_index < 0 || r.tilt_index >= _tilts.count()) {
				continue;
			}
			const float u = actuator_sp(m);      // 归一化输出 (已含 offset 后的 raw motor setpoint)
			const float ct = r.thrust_coef;
			const float fi = ct * u;
			const float y  = r.position(1);

			if (fabsf(y) < TAIL_Y_EPS) {
				f_tail += fi;
				tail_cnt++;
				if (tilt_idx_tail<0) tilt_idx_tail = r.tilt_index;
				motor_group[m]=Group::Tail;

			} else if (y > 0.f) {
				f_fl += fi;
				y_fl_mean += y;
				fl_cnt++;
				if (tilt_idx_fl<0) tilt_idx_fl = r.tilt_index;
				motor_group[m]=Group::FrontLeft;

			} else {
				f_fr += fi;
				y_fr_mean += y;
				fr_cnt++;
				if (tilt_idx_fr<0) tilt_idx_fr = r.tilt_index;
				motor_group[m]=Group::FrontRight;
			}
		}

		if (fl_cnt>0) y_fl_mean/=fl_cnt;
		if (fr_cnt>0) y_fr_mean/=fr_cnt;

		const float Fx_cmd = control_sp(3);
		const float Mz_cmd = control_sp(2);

		// 至少需要前左右用于 yaw + Fx
		bool geometry_ok = (fl_cnt>0 && fr_cnt>0 && (fabsf(f_fl)>EPS_F || fabsf(f_fr)>EPS_F));

		float theta_fl = 0.f, theta_fr = 0.f, theta_tail = 0.f;

		if (geometry_ok && (fabsf(Fx_cmd)>1e-7f || fabsf(Mz_cmd)>1e-7f)) {

			// 外部尾舵：如果有效则固定尾舵角 => 2x2 解，仅前左右未知
			bool tail_fixed_external=false;
			// if (ENABLE_EXTERNAL_TAIL_TILT && tail_cnt>0 && tilt_idx_tail>=0) {
			// 	float ext;
			// 	if (readExternalTailTilt(ext, tilt_idx_tail)) {
			// 		theta_tail = ext;
			// 		tail_fixed_external = true;
			// 	}
			// }

			if (!tail_fixed_external) {
				// 能耗最优 α_i = f_i => A_i = f_i
				// 通用公式：
				// S_A = fL+fR+fT; S_Ay = yL fL + yR fR; S_Ay2 = yL^2 fL + yR^2 fR
				// λ1 = (-Fx S_Ay2 - Mz S_Ay)/D
				// λ2 = ( Fx S_Ay + Mz S_A )/D
				// θ_T = -λ1
				// θ_L = -(λ1 + λ2 yL)
				// θ_R = -(λ1 + λ2 yR)
				float fL = f_fl;
				float fR = f_fr;
				float fT = f_tail;
				float yL = (fl_cnt>0)? y_fl_mean : 0.f;
				float yR = (fr_cnt>0)? y_fr_mean : 0.f;

				// 若无尾部 (tail_cnt==0) => fT=0，公式仍可用；注意 S_A 包含 fT
				const float S_A  = fL + fR + fT;
				const float S_Ay = yL * fL + yR * fR;
				const float S_Ay2= yL * yL * fL + yR * yR * fR;
				const float D = S_A * S_Ay2 - S_Ay * S_Ay;

				if (fabsf(D) > 1e-9f && S_A > EPS_F) {
					float lambda1 = (-Fx_cmd * S_Ay2 - Mz_cmd * S_Ay)/D;
					float lambda2 = ( Fx_cmd * S_Ay  + Mz_cmd * S_A )/D;

					if (fT > EPS_F) {
						theta_tail = -lambda1;
					} else {
						theta_tail = 0.f;
					}
					theta_fl  = -(lambda1 + lambda2 * yL);
					theta_fr  = -(lambda1 + lambda2 * yR);

				} else {
					// 退化（例如 y 对称但推力≈0），简化：不考虑尾部，使用 yaw 差动 + 公共
					float pair_sum = fL + fR;
					if (pair_sum > EPS_F) {
						// 公共前向
						float delta_common = Fx_cmd / pair_sum;
						// 差动 yaw 近似
						float y_mean_abs = 0.5f*(fabsf(yL)+fabsf(yR));
						float delta_yaw = 0.f;
						if (y_mean_abs>EPS_F) {
							float f_equiv = 0.5f*pair_sum;
							delta_yaw = - Mz_cmd / (2.f * y_mean_abs * f_equiv);
						}
						theta_fl = delta_common + delta_yaw;
						theta_fr = delta_common - delta_yaw;
					}
					theta_tail = 0.f;
				}

			} else {
				// 外部尾舵固定：解 2x2
				// 约束:
				// fL θL + fR θR = Fx_cmd - fT θT_fixed
				// yL fL θL + yR fR θR = -Mz_cmd
				float fL=f_fl, fR=f_fr;
				float yL=y_fl_mean, yR=y_fr_mean;
				float B1 = Fx_cmd - f_tail * theta_tail;
				float B2 = -Mz_cmd;
				float det = fL * fR * (yL - yR);
				if (fabsf(det) > 1e-9f && fabsf(fL)>EPS_F && fabsf(fR)>EPS_F) {
					theta_fl = ( B1 * yR * fR - B2 * fR ) / det;
					theta_fr = (-B1 * yL * fL + B2 * fL ) / det;
				} else {
					theta_fl = theta_fr = 0.f;
				}
			}

			// 限幅
			auto limit = [&](float &a){ a = math::constrain(a, -MAX_LINEAR_TILT_RAD, MAX_LINEAR_TILT_RAD); };
			limit(theta_fl); limit(theta_fr); limit(theta_tail);

			// 实际 Fx (线性)
			float Fx_real = f_fl * theta_fl + f_fr * theta_fr + f_tail * theta_tail;
			_fx_residual = Fx_cmd - Fx_real;

			// 映射舵机
			auto setServo = [&](int tilt_index, float theta){
				if (tilt_index<0 || tilt_index>=_tilts.count()) return;
				float s = angleToServoNormalized(tilt_index, theta);
				actuator_sp(_first_tilt_idx + tilt_index) = s;
			};
			setServo(tilt_idx_fl, theta_fl);
			setServo(tilt_idx_fr, theta_fr);
			setServo(tilt_idx_tail, theta_tail);

			// 每组垂直补偿
			for (int m=0; m<geom.num_rotors; ++m) {
				if (motor_group[m] == Group::None) continue;
				float theta_g = 0.f;
				switch (motor_group[m]) {
				case Group::FrontLeft:  theta_g = theta_fl; break;
				case Group::FrontRight: theta_g = theta_fr; break;
				case Group::Tail:       theta_g = theta_tail; break;
				default: break;
				}
				float scale = 1.f + 0.5f * theta_g * theta_g;
				float cmd = actuator_sp(m) * scale;
				cmd = math::constrain(cmd, actuator_min(m), actuator_max(m));
				actuator_sp(m) = cmd;
			}
		}
	}

	bool yaw_saturated_positive = true;
	bool yaw_saturated_negative = true;

	for (int i = 0; i < _tilts.count(); ++i) {
		const int actuator_idx = i + _first_tilt_idx;
		const float yaw_torque = _tilts.getYawTorqueOfTilt(i);

		// Custom yaw saturation logic: only declare yaw saturated if all tilts
		// are at the negative or positive yawing limit
		if (yaw_torque > FLT_EPSILON) {
			if (yaw_saturated_positive && actuator_sp(actuator_idx) < actuator_max(actuator_idx) - FLT_EPSILON) {
				yaw_saturated_positive = false;
			}

			if (yaw_saturated_negative && actuator_sp(actuator_idx) > actuator_min(actuator_idx) + FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

		} else if (yaw_torque < -FLT_EPSILON) {
			if (yaw_saturated_negative && actuator_sp(actuator_idx) < actuator_max(actuator_idx) - FLT_EPSILON) {
				yaw_saturated_negative = false;
			}

			if (yaw_saturated_positive && actuator_sp(actuator_idx) > actuator_min(actuator_idx) + FLT_EPSILON) {
				yaw_saturated_positive = false;
			}
		}
	}

	_yaw_tilt_saturation_flags.tilt_yaw_neg = yaw_saturated_negative;
	_yaw_tilt_saturation_flags.tilt_yaw_pos = yaw_saturated_positive;
}

void ActuatorEffectivenessMCTilt::getUnallocatedControl(int matrix_index, control_allocator_status_s &status)
{
	if (ENABLE_LINEAR_FX_YAW_ENERGY && matrix_index==0) {
		status.unallocated_thrust[0] += _fx_residual;
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

void ActuatorEffectivenessMCTilt::updateCollectiveTiltAngle()
{
	vehicle_thrust_setpoint_s vt{};
	const hrt_abstime now = hrt_absolute_time();

	if (_vt_setpoint_sub.update(&vt)) {
		// if (vt.tilt_extra_angle_valid && PX4_ISFINITE(vt.tilt_extra_angle)) {
		// 	// 有效角度命令
		// 	if (_incremental_mode) {
		// 		// 增量模式：目标角度 = 基准角度 + 增量角度
		// 		_collective_tilt_target = _collective_tilt_base + vt.tilt_extra_angle;
		// 	} else {
		// 		// 绝对模式：直接设置目标角度
		// 		_collective_tilt_target = vt.tilt_extra_angle;
		// 	}
		// 
		// 	_collective_tilt_valid  = true;
		// 	_last_valid_command_time = now;
		// 
		// 	// 如在回零阶段，打断回零
		// 	if (_ramp_state == RampState::RAMP_TO_ZERO) {
		// 		_ramp_state = RampState::NONE;
		// 	}
		// 
		// } else if (!vt.tilt_extra_angle_valid) {
		// 	// 显式禁用：立即启动回零（若当前不为 0）
		// 	_collective_tilt_valid = false;
		// 	_collective_tilt_target = _collective_tilt_base; // 回到基准位置
		// 	_last_valid_command_time = now;
		// 
		// 	if (fabsf(_collective_tilt_angle - _collective_tilt_base) > 1e-3f) {
		// 		_ramp_state = RampState::RAMP_TO_ZERO;
		// 	} else {
		// 		_collective_tilt_angle = _collective_tilt_base;
		// 		_ramp_state = RampState::NONE;
		// 	}
		// }
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

float ActuatorEffectivenessMCTilt::angleToServoNormalized(int tilt_index, float theta) const
{
	if (tilt_index<0 || tilt_index>=_tilts.count()) return 0.f;
	const auto &cfg = _tilts.config(tilt_index);
	float min_a = cfg.min_angle;
	float max_a = cfg.max_angle;
	float delta = max_a - min_a;
	if (delta < 1e-5f) return 0.f;
	float t = math::constrain(theta, min_a, max_a);
	float s = 2.f * (t - min_a)/delta - 1.f;
	return math::constrain(s, -1.f, 1.f);
}

void ActuatorEffectivenessMCTilt::lazyInitTailTiltSubscription()
{
	if (!ENABLE_EXTERNAL_TAIL_TILT || _tail_tilt_sub_initialized) return;
	// 需要你已添加 tail_tilt_setpoint.msg
	_tail_tilt_setpoint_sub = uORB::Subscription{ORB_ID(tail_tilt_setpoint)};
	_tail_tilt_sub_initialized = true;
}

bool ActuatorEffectivenessMCTilt::readExternalTailTilt(float &theta_tail_out, int tail_tilt_index)
{
	if (!ENABLE_EXTERNAL_TAIL_TILT || !_tail_tilt_sub_initialized) return false;

	tail_tilt_setpoint_s msg{};
	if (_tail_tilt_setpoint_sub.copy(&msg)) {
		_last_tail_tilt_ts = msg.timestamp;
		_last_tail_tilt_norm = msg.normalized_setpoint;
	}
	if (!PX4_ISFINITE(_last_tail_tilt_norm)) return false;
	float nz = math::constrain(_last_tail_tilt_norm, 0.f, 1.f);

	if (tail_tilt_index<0 || tail_tilt_index>=_tilts.count()) return false;
	const auto &cfg = _tilts.config(tail_tilt_index);
	float theta = cfg.min_angle + nz*(cfg.max_angle - cfg.min_angle);
	theta_tail_out = math::constrain(theta, -MAX_LINEAR_TILT_RAD, MAX_LINEAR_TILT_RAD);
	return true;
}
