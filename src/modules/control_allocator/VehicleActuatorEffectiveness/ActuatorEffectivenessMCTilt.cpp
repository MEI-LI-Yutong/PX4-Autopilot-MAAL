/****************************************************************************
 *   Copyright (c) 2021-2023 PX4 Development Team. All rights reserved.
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

    // Set offset such that tilts point upwards when control input == 0
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

void ActuatorEffectivenessMCTilt::updateSetpoint(const matrix::Vector<float, NUM_AXES> &control_sp, int matrix_index,
        ActuatorVector &actuator_sp, const ActuatorVector &actuator_min, const ActuatorVector &actuator_max)
{
    actuator_sp += _tilt_offsets;

    // =================== Fx 等角层 + 抗风零空间推进（后处理） ===================
    _fx_residual = 0.f;

    if (matrix_index == 0 && _tilts.count() > 0 && ENABLE_FX_LAYER) {

        const auto &geom = _mc_rotors.geometry();

        // 只选择"朝前倾"的 tilt（避免侧向力）
        struct Item { int motor; int tilt; float fi; float y; float old_tilt_angle; };
        Item items[ActuatorEffectivenessRotors::NUM_ROTORS_MAX];
        int N = 0;

        // 记录 3 组 tilt 索引（可能为 -1）
        int tilt_fl_idx = -1, tilt_fr_idx = -1, tilt_tail_idx = -1;

        for (int m = 0; m < geom.num_rotors; ++m) {
            const auto &r = geom.rotors[m];
            const int t = r.tilt_index;
            if (t < 0 || t >= _tilts.count()) continue;
            if (_tilts.config(t).tilt_direction != ActuatorEffectivenessTilts::TiltDirection::TowardsFront) continue;

            const float u0 = actuator_sp(m);        // 当前已分配电机归一化输出
            const float ct = r.thrust_coef;
            const float fi = ct * u0;               // 小角近似推力幅值
            const float y  = r.position(1);

            const int servo_col = _first_tilt_idx + t;
            if (servo_col < 0 || servo_col >= NUM_ACTUATORS) continue;
            const float old_angle = servoNormalizedToAngle(t, actuator_sp(servo_col));

            items[N++] = {m, t, fi, y, old_angle};

            // 简单用 y 分组（只取第一次命中的 tilt 索引，用于等量加 ψ）
            if (fabsf(y) < TAIL_Y_EPS) {
                if (tilt_tail_idx < 0) tilt_tail_idx = t;
            } else if (y > 0.f) {
                if (tilt_fl_idx < 0) tilt_fl_idx = t;
            } else {
                if (tilt_fr_idx < 0) tilt_fr_idx = t;
            }

            if (N >= ActuatorEffectivenessRotors::NUM_ROTORS_MAX) break;
        }

        // 计算 Σ f_i、Fx/Fz 指令
        float F_total = 0.f;
        for (int i = 0; i < N; ++i) F_total += items[i].fi;

        const float Fx_cmd = control_sp(3);
        const float Fz_cmd = control_sp(5);

        if (N > 0 && PX4_ISFINITE(Fx_cmd) && PX4_ISFINITE(Fz_cmd) && fabsf(Fz_cmd) > EPS_F) {
            // ---------- Step A: 等角 tilt0 生成 Fx ----------
            float delta_theta = -Fx_cmd / Fz_cmd; // 你的约定：z 向下为正，要产生 +Fx 需负角
            delta_theta = math::constrain(delta_theta, -MAX_LINEAR_TILT_RAD, MAX_LINEAR_TILT_RAD);

            // 写回舵机角；记录等角后的角度
            bool tilt_written[ActuatorEffectivenessTilts::MAX_COUNT] {};
            float tilt_angle_after_A[ActuatorEffectivenessTilts::MAX_COUNT] {};

            for (int i = 0; i < N; ++i) {
                const int t = items[i].tilt;
                const int servo_col = _first_tilt_idx + t;

                if (!tilt_written[t]) {
                    const float old_angle = servoNormalizedToAngle(t, actuator_sp(servo_col));
                    const float raw_target = old_angle + delta_theta;
                    const float clamped_target = math::constrain(raw_target,
                        _tilts.config(t).min_angle, _tilts.config(t).max_angle);
                    const float final_target = math::constrain(clamped_target,
                        -MAX_LINEAR_TILT_RAD, MAX_LINEAR_TILT_RAD);

                    actuator_sp(servo_col) = angleToServoNormalized(t, final_target);
                    tilt_angle_after_A[t] = servoNormalizedToAngle(t, actuator_sp(servo_col));
                    tilt_written[t] = true;
                }
            }

            // 垂直补偿 + Fx 实现值累计（Step A）
            float Fx_real = 0.f;

            for (int i = 0; i < N; ++i) {
                const int m = items[i].motor;
                const int t = items[i].tilt;

                const float old_a = items[i].old_tilt_angle;
                const float new_a = tilt_angle_after_A[t];

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

            // ---------- Step B: 抗风（尾部 φ 固定负向，前排等量 ψ，净 Fx 保持不变） ----------
            if (ENABLE_ANTIWIND && tilt_tail_idx >= 0 && tilt_fl_idx >= 0 && tilt_fr_idx >= 0) {

                // 用 Step A 后的角度作为起点
                const float a_tail_cur = tilt_angle_after_A[tilt_tail_idx];
                const float a_fl_cur   = tilt_angle_after_A[tilt_fl_idx];
                const float a_fr_cur   = tilt_angle_after_A[tilt_fr_idx];

                // 线性域/物理域限幅
                auto clamp_min = [&](int t) { return math::max(_tilts.config(t).min_angle, -MAX_LINEAR_TILT_RAD); };
                auto clamp_max = [&](int t) { return math::min(_tilts.config(t).max_angle,  MAX_LINEAR_TILT_RAD); };

                const float min_tail = clamp_min(tilt_tail_idx), max_tail = clamp_max(tilt_tail_idx);
                const float min_fl   = clamp_min(tilt_fl_idx),   max_fl   = clamp_max(tilt_fl_idx);
                const float min_fr   = clamp_min(tilt_fr_idx),   max_fr   = clamp_max(tilt_fr_idx);

                // 重新用当前（Step A 后）motor 命令估推力，得到组推力
                float f_tail = 0.f, f_front_total = 0.f;

                for (int i = 0; i < N; ++i) {
                    const auto &r = geom.rotors[items[i].motor];
                    const float fi_now = r.thrust_coef * actuator_sp(items[i].motor); // 用补偿后的命令更贴合当前
                    if (fabsf(items[i].y) < TAIL_Y_EPS) f_tail += fi_now;
                    else f_front_total += fi_now;
                }

                if (f_tail > EPS_F && f_front_total > EPS_F) {
                    // φ 固定负号：让尾部永远产生负向 Fx
                    // 先取缺省 φ0=-10°，再按角度域裁剪到可行范围
                    const float phi0 = ANTIWIND_PHI_RAD; // <0
                    // 尾部角域允许的负向步长
                    float phi_min_tail = min_tail - a_tail_cur; // 这是能走到下限的负向极限（<=0）
                    // 目标 φ 应为负，裁剪到 [phi_min_tail, 0]
                    float phi = math::constrain(phi0, phi_min_tail, 0.f);

                    // 零 Fx 约束：ψ = -(f_tail/f_front_total) * φ
                    const float ratio = (f_tail / f_front_total);
                    float psi = -ratio * phi;

                    // 应用抗风滑移系数 k：按 k 缩放 phi 和 psi
                    phi *= _antiwind_blend_k;
                    psi *= _antiwind_blend_k;

                    // 前排左右还需满足各自角域：a_fl_cur+psi ∈ [min_fl,max_fl]，a_fr_cur+psi 同理
                    float psi_min = math::max(min_fl - a_fl_cur, min_fr - a_fr_cur);
                    float psi_max = math::min(max_fl - a_fl_cur, max_fr - a_fr_cur);
                    psi = math::constrain(psi, psi_min, psi_max);

                    // 如果 ψ 被裁剪，反推 φ 以尽量满足零Fx（保持符号为负）
                    phi = - (f_front_total / f_tail) * psi;
                    phi = math::constrain(phi, phi_min_tail, 0.f);

                    // 最终再算一次 ψ，确保一致
                    psi = -ratio * phi;
                    psi = math::constrain(psi, psi_min, psi_max);

                    // 写回舵机角（在 Step A 结果上叠加）
                    {
                        // 尾部
                        const int servo_tail = _first_tilt_idx + tilt_tail_idx;
                        const float tail_target = math::constrain(a_tail_cur + phi, min_tail, max_tail);
                        actuator_sp(servo_tail) = angleToServoNormalized(tilt_tail_idx, tail_target);
                        // 前左
                        const int servo_fl = _first_tilt_idx + tilt_fl_idx;
                        const float fl_target = math::constrain(a_fl_cur + psi, min_fl, max_fl);
                        actuator_sp(servo_fl) = angleToServoNormalized(tilt_fl_idx, fl_target);
                        // 前右
                        const int servo_fr = _first_tilt_idx + tilt_fr_idx;
                        const float fr_target = math::constrain(a_fr_cur + psi, min_fr, max_fr);
                        actuator_sp(servo_fr) = angleToServoNormalized(tilt_fr_idx, fr_target);
                    }

                    // 读回实际角（考虑量化/限幅），二次垂直补偿 + Fx 额外贡献（理论 ≈0）
                    float Fx_extra = 0.f;

                    // 先缓存"抗风前后的"角
                    float a_tail_new = servoNormalizedToAngle(tilt_tail_idx, actuator_sp(_first_tilt_idx + tilt_tail_idx));
                    float a_fl_new   = servoNormalizedToAngle(tilt_fl_idx,   actuator_sp(_first_tilt_idx + tilt_fl_idx));
                    float a_fr_new   = servoNormalizedToAngle(tilt_fr_idx,   actuator_sp(_first_tilt_idx + tilt_fr_idx));

                    for (int i = 0; i < N; ++i) {
                        const int m = items[i].motor;
                        const int t = items[i].tilt;

                        // old2 = Step A 角度；new2 = 抗风后角度
                        float old2 = tilt_angle_after_A[t];
                        float new2 = old2;
                        if (t == tilt_tail_idx) new2 = a_tail_new;
                        else if (t == tilt_fl_idx) new2 = a_fl_new;
                        else if (t == tilt_fr_idx) new2 = a_fr_new;

                        float scale2 = 1.f;
                        const float c_old2 = cosf(old2);
                        const float c_new2 = cosf(new2);
                        if (fabsf(c_new2) > 1e-4f) {
                            scale2 = c_old2 / c_new2;
                        }

                        float cmd2 = actuator_sp(m) * scale2;
                        cmd2 = math::constrain(cmd2, actuator_min(m), actuator_max(m));
                        actuator_sp(m) = cmd2;

                        // Fx 额外贡献
                        Fx_extra += (geom.rotors[m].thrust_coef * cmd2) * (new2 - old2);
                        // 注：这里用更新后的 cmd2 乘 (new2-old2) 近似计算，触限时更贴近实际
                    }

                    // 理论上 Fx_extra≈0；受限幅/量化误差计入 Fx_real
                    Fx_real += Fx_extra;
                }
            }

            _fx_residual = Fx_cmd - Fx_real;
        }
    }
    // =================== Fx 层结束 ===================

    // 保留原有 yaw 饱和判定与上报
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
    // 注入 Fx 残差，便于上层积分（仅 matrix 0）
    if (matrix_index == 0) {
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