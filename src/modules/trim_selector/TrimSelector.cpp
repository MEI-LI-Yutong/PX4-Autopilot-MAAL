/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

#include "TrimSelector.hpp"

#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/defines.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind.h>
#include <lib/perf/perf_counter.h>
#include <logger/messages.h>

#include <mathlib/mathlib.h>
#include <geo/geo.h> // CONSTANTS_ONE_G

using namespace matrix;

TrimSelector::TrimSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	parameters_update(true);
}

TrimSelector::~TrimSelector() = default;

bool TrimSelector::init()
{
    ScheduleOnInterval(50_ms); // 20 Hz
    PX4_INFO("Trim Selector initialized at 20 Hz (with takeoff/landing ramp)");
	return true;
}

void TrimSelector::parameters_update(bool force)
{
	if (_parameter_update_sub.updated() || force) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}
}

static inline float rad2deg(float r) { return r * 180.f / M_PI_F; }

bool TrimSelector::compute_nominal_trim(float &f1, float &f2, float &f3,
                                        float &theta1_deg, float &theta2_deg, float &theta3_deg,
                                        float &horizontal_velocity_magnitude, bool &data_valid)
{
    trajectory_setpoint_s traj{};
    // 优先读取本周期更新的轨迹设定；若无更新则使用上一次的轨迹设定
    bool has_traj = _trajectory_setpoint_sub.update(&traj) || _trajectory_setpoint_sub.copy(&traj);

	horizontal_velocity_magnitude = 0.f;
	float pitch_setpoint_rad = 0.f;
	data_valid = false;

	if (has_traj && PX4_ISFINITE(traj.velocity[0]) && PX4_ISFINITE(traj.velocity[1])) {
		Vector2f vxy(traj.velocity);
		horizontal_velocity_magnitude = sqrtf(vxy.norm_squared());

		// 这里仍用你的经验：pitch = k * |v_xy|
		pitch_setpoint_rad = _param_ts_pitch_gain.get() * horizontal_velocity_magnitude;
		data_valid = true;
	}

	if (data_valid) {
		// 名义（不含 ramp）的推力与角度（单位：N 与 deg）
		f1 = 5.0f + 2.0f * horizontal_velocity_magnitude;
		f2 = 5.0f + 2.0f * horizontal_velocity_magnitude;
		f3 = 8.0f + 1.5f * horizontal_velocity_magnitude;

		const float pitch_deg = rad2deg(pitch_setpoint_rad);
		theta1_deg = 0.5f * pitch_deg;
		theta2_deg = 0.5f * pitch_deg;
		theta3_deg = 0.8f * pitch_deg;

	} else {
		// 无有效轨迹：使用图片中的默认值，角度为 0（注意：仅名义，实际输出仍要乘以 ramp s）
		f1 = 10.1569f;  // 图片中的第一个值
		f2 = 10.1569f;  // 图片中的第二个值
		f3 = 8.1255f;   // 图片中的第三个值
		theta1_deg = 0.f;
		theta2_deg = 0.f;
		theta3_deg = 0.f;
	}

	return data_valid;
}

void TrimSelector::update_takeoff_land_ramp(float dt)
{
	// 默认：不启用 ramp 就直通（s=1）
	if (_param_ts_ramp_en.get() == 0) {
		_s = 1.f;
		_s_target = 1.f;
		return;
	}

	vehicle_status_s vs{};
	vehicle_control_mode_s vcm{};
	vehicle_land_detected_s vld{};
	manual_control_setpoint_s msp{};

	_vehicle_status_sub.copy(&vs);
	_vehicle_control_mode_sub.copy(&vcm);
	_vehicle_land_detected_sub.copy(&vld);
	_manual_sp_sub.copy(&msp);

	const bool landed = vld.landed || vld.ground_contact;

	// 自动模式触发
	const bool auto_takeoff =
		(vs.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF) ||
		(vs.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION && landed);

	const bool auto_landing =
		(vs.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) ||
		(vs.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL);

	// 手动模式触发（非 auto）
	const bool is_auto = vcm.flag_control_auto_enabled;
	const float thr_tko = math::constrain(_param_ts_thr_tko.get(), 0.f, 1.f);
	const bool throttle_valid = PX4_ISFINITE(msp.throttle);

	const bool manual_takeoff = (!is_auto) && landed && throttle_valid && (msp.throttle > thr_tko);
	const bool manual_landing = (!is_auto) && (landed || (throttle_valid && (msp.throttle < 0.1f)));

	// 目标 s
	if (landed && !(auto_takeoff || manual_takeoff)) {
		_s_target = 0.f; // 地面待机：不开启名义配平
	} else if (auto_takeoff || manual_takeoff) {
		_s_target = 1.f; // 起飞：拉满名义配平
	} else if (auto_landing || manual_landing) {
		_s_target = math::constrain(_param_ts_s_land.get(), 0.f, 1.f); // 降落阶段减到 s_land
	} else {
		_s_target = 1.f; // 正常飞行保持 1
	}

    // 一阶滤波 ramp（时间常数）
    const float tau_up = math::max(_param_ts_ramp_t_up.get(), 0.05f);
    const float tau_dn = math::max(_param_ts_ramp_t_dn.get(), 0.05f);

    const bool rising = (_s_target > _s);
    const float tau = rising ? tau_up : tau_dn;
    const float alpha = dt / (tau + dt);

    _s += alpha * (_s_target - _s);
    _s = math::constrain(_s, 0.f, 1.f);
}

void TrimSelector::Run()
{
	parameters_update();

	// dt
	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.01f; // fallback 10ms
	if (_last_run != 0) {
		dt = math::constrain((now - _last_run) / 1e6f, 0.002f, 0.05f);
	}
	_last_run = now;

	// 更新 ramp
	update_takeoff_land_ramp(dt);

	// 计算名义配平（不含 ramp）
	float f1_nom=0.f, f2_nom=0.f, f3_nom=0.f;
	float th1_nom=0.f, th2_nom=0.f, th3_nom=0.f;
	float vxy_mag=0.f;
	bool data_valid=false;
	compute_nominal_trim(f1_nom, f2_nom, f3_nom, th1_nom, th2_nom, th3_nom, vxy_mag, data_valid);

    // 应用 ramp：实际输出 = s * 名义（对 s 使用 smoothstep 以获得更平滑的端点）
    const float s_raw = _s;
    const float s = (3.f * s_raw * s_raw) - (2.f * s_raw * s_raw * s_raw); // smoothstep(s_raw)

	utrim_s utrim{};
	utrim.timestamp = now;
	utrim.horizontal_velocity = vxy_mag;
	utrim.valid = data_valid; // 表示名义值的来源（轨迹有效性），而不是 ramp 状态

	utrim.polynomial_values[0] = s * f1_nom; // f1 [N]
	utrim.polynomial_values[1] = s * f2_nom; // f2 [N]
	utrim.polynomial_values[2] = s * f3_nom; // f3 [N]
	utrim.polynomial_values[3] = s * th1_nom; // θ1 [deg]
	utrim.polynomial_values[4] = s * th2_nom; // θ2 [deg]
	utrim.polynomial_values[5] = s * th3_nom; // θ3 [deg]

	_utrim_pub.publish(utrim);

	// 同步发布 theta_trim（俯仰角，带 ramp）
	theta_trim_s theta_trim{};
	theta_trim.timestamp = now;

	// 固定发布 0 度（单位：deg）
	theta_trim.pitch_angle = 0.0f;

	_theta_trim_pub.publish(theta_trim);

	/*
	// 调试日志（限频） — 如需调试可取消注释
	static uint64_t last_log = 0;
	if (now - last_log > 1000_ms) {
		PX4_DEBUG("TrimSelector: s=%.2f -> f=[%.2f %.2f %.2f]N, th=[%.1f %.1f %.1f]deg, vxy=%.2f, valid=%s",
		         (double)s,
		         (double)utrim.polynomial_values[0], (double)utrim.polynomial_values[1], (double)utrim.polynomial_values[2],
		         (double)utrim.polynomial_values[3], (double)utrim.polynomial_values[4], (double)utrim.polynomial_values[5],
		         (double)vxy_mag, data_valid ? "true":"false");
		last_log = now;
	}
	*/
}

int TrimSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
Trim Selector 模块发布 utrim（名义配平）和 theta_trim，并加入“起飞/降落斜坡（ramp）”。
- 地面：s≈0，utrim≈0，不会 arm 就起飞
- 起飞（自动/手动触发）：s 从 0 平滑到 1
- 降落：s 从 1 平滑到 TS_S_LAND（或 0）

utrim 内容为 [f1 f2 f3 θ1 θ2 θ3]，单位分别为 [N] 与 [deg]。
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trim_selector", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TrimSelector::task_spawn(int argc, char *argv[])
{
	PX4_INFO("Starting Trim Selector...");
	TrimSelector *instance = new TrimSelector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("Trim Selector started");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	PX4_ERR("Trim Selector start failed");
	return PX4_ERROR;
}

int TrimSelector::custom_command(int argc, char *argv[])
{
	return print_usage("Unsupported command");
}

extern "C" __EXPORT int trim_selector_main(int argc, char *argv[])
{
	return TrimSelector::main(argc, argv);
}
