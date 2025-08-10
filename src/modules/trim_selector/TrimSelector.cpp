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

#include <mathlib/mathlib.h>
#include <geo/geo.h>
// 若之前未包含矩阵库头，补充包含
#include <lib/matrix/matrix/math.hpp>

using namespace matrix;

TrimSelector::TrimSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化参数更新
	parameters_update(true);
}

TrimSelector::~TrimSelector()
{
	// 析构函数，清理资源（如果有需要）
}

/* === 新增函数实现：获取风速大小 === */
float TrimSelector::get_wind_magnitude()
{
    using namespace time_literals;

    // 1) 首选 airspeed_wind 话题
    airspeed_wind_s wind{};

    if (_airspeed_wind_sub.copy(&wind)) {
        const bool recent = (hrt_absolute_time() - wind.timestamp) < 500_ms;

        if (recent && PX4_ISFINITE(wind.windspeed_north) && PX4_ISFINITE(wind.windspeed_east)) {
            return sqrtf(wind.windspeed_north * wind.windspeed_north +
                         wind.windspeed_east  * wind.windspeed_east);
        }
    }

    // 2) 回退方案：|v_ground − v_air_rel|
    vehicle_local_position_s lpos{};
    vehicle_attitude_s       att{};
    airspeed_validated_s     asp{};

    const bool have_lpos = _vehicle_local_position_sub.copy(&lpos);
    const bool have_att  = _vehicle_attitude_sub.copy(&att);
    const bool have_asp  = _airspeed_validated_sub.copy(&asp);

    if (have_lpos && have_att && have_asp && PX4_ISFINITE(asp.true_airspeed_m_s)) {
        // 地速（惯性系）
        const Vector2f v_ground{lpos.vx, lpos.vy};

        // 将机体系真空速 (TAS, 0, 0) 旋转到惯性系
        const Dcmf R_nb{Quatf(att.q)};
        const Vector3f v_air_body{asp.true_airspeed_m_s, 0.f, 0.f};
        const Vector3f v_air_earth = R_nb * v_air_body;

        const Vector2f v_air_xy{v_air_earth(0), v_air_earth(1)};

        // 风 = 地速 − 空速
        const Vector2f wind_xy = v_ground - v_air_xy;
        return wind_xy.norm();
    }

    // 3) 数据不足，返回 0
    return 0.f;
}

bool TrimSelector::init()
{
	// 初始化定时器，定期执行Run()函数
	ScheduleOnInterval(10_ms); // 100Hz 更新率，与 position control 相同
	PX4_INFO("Trim Selector module initialized and started");
	return true;
}

void TrimSelector::parameters_update(bool force)
{
	// 检查参数更新
	if (_parameter_update_sub.updated() || force) {
		// 清除更新标志
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// 从存储中更新参数
		updateParams();
	}
}

void TrimSelector::Run()
{
	// 首先检查参数是否需要更新
	parameters_update();

	// 订阅轨迹设定点
	trajectory_setpoint_s trajectory_setpoint{};
	bool has_trajectory_data = _trajectory_setpoint_sub.copy(&trajectory_setpoint);

    // 添加调试信息（每5秒打印一次状态）
	static uint64_t last_status_time = 0;
	uint64_t now = hrt_absolute_time();
	if (now - last_status_time > 5000000) { // 5秒
		if (!has_trajectory_data) {
			PX4_WARN("Trim Selector: No trajectory_setpoint data - using defaults");
		} else if (!PX4_ISFINITE(trajectory_setpoint.velocity[0]) || !PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
			PX4_WARN("Trim Selector: Invalid velocity data - using defaults");
		} else {
			PX4_INFO("Trim Selector: Valid trajectory data available");
		}
		last_status_time = now;
	}

    // === 平滑起降：更新斜坡状态 ===
    vehicle_status_s vstatus{};
    _vehicle_status_sub.copy(&vstatus);
    vehicle_land_detected_s land{};
    _vehicle_land_detected_sub.copy(&land);
    manual_control_setpoint_s manual{};
    _manual_control_setpoint_sub.copy(&manual);

    const bool armed = (vstatus.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
    const bool landed = land.landed;
    const bool ground_contact = land.ground_contact;
    const bool auto_landing_mode = (vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
                                 || vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL
                                 || vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND);
    const bool manual_mode = (vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_ALTCTL
                           || vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_POSCTL
                           || vstatus.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL);

    // 获取油门输入（0-1范围，1表示最大油门）
    const float throttle = manual.throttle;
    const bool high_throttle = (throttle > _takeoff_throttle_threshold);
    const bool low_throttle = (throttle < _landing_throttle_threshold);

    switch (_ramp_phase) {
    case RampPhase::Idle:
        _utrim_alpha = 0.f;
        // 起飞条件：解锁 + 未落地 + (自动模式 或 手动模式下高油门)
        if (armed && !landed) {
            bool should_takeoff = false;
            if (!manual_mode) {
                // 自动模式：直接触发起飞斜坡
                should_takeoff = true;
            } else if (manual_mode && high_throttle) {
                // 手动模式：需要高油门才触发起飞斜坡
                should_takeoff = true;
            }

            if (should_takeoff) {
                _ramp_phase = RampPhase::TakeoffRamp;
                _ramp_start_time = now;
                _utrim_alpha = 0.f;
            }
        }
        break;

    case RampPhase::TakeoffRamp: {
        const float dt_s = (now - _ramp_start_time) * 1e-6f;
        const float progress = dt_s / _takeoff_ramp_time_s;
        _utrim_alpha = math::constrain(progress, 0.f, 1.f);
        if (!armed) {
            _ramp_phase = RampPhase::Idle;
            _utrim_alpha = 0.f;
        } else if (_utrim_alpha >= 1.f) {
            _ramp_phase = RampPhase::Flight;
            _utrim_alpha = 1.f;
        }
        break; }

    case RampPhase::Flight:
        _utrim_alpha = 1.f;
        if (!armed) {
            _ramp_phase = RampPhase::Idle;
            _utrim_alpha = 0.f;
        } else {
            // 降落条件：自动降落模式 或 接地 或 (手动模式下低油门+接地)
            bool should_land = false;
            if (auto_landing_mode) {
                should_land = true;
            } else if (ground_contact || landed) {
                should_land = true;
            } else if (manual_mode && low_throttle && ground_contact) {
                should_land = true;
            }

            if (should_land) {
                _ramp_phase = RampPhase::LandingRamp;
                _ramp_start_time = now;
            }
        }
        break;

    case RampPhase::LandingRamp: {
        const float dt_s = (now - _ramp_start_time) * 1e-6f;
        const float progress = dt_s / _landing_ramp_time_s;
        _utrim_alpha = math::constrain(1.f - progress, 0.f, 1.f);
        if (!armed || landed) {
            if (_utrim_alpha <= 0.f) {
                _ramp_phase = RampPhase::Idle;
            }
        }
        break; }
    }

    // 每2秒打印一次斜坡状态
    static uint64_t last_ramp_log = 0;
    if (now - last_ramp_log > 2'000'000) {
        const char* phase_names[] = {"Idle", "TakeoffRamp", "Flight", "LandingRamp"};
        const char* nav_mode = manual_mode ? "MANUAL" : "AUTO";
        PX4_INFO("Trim Selector: phase=%s, alpha=%.2f, throttle=%.2f, mode=%s, armed=%d, landed=%d",
                 phase_names[(int)_ramp_phase], (double)_utrim_alpha, (double)throttle,
                 nav_mode, armed, landed);
        last_ramp_log = now;
    }

    // 初始化默认值
	float horizontal_velocity_magnitude = 0.0f;
	float pitch_setpoint = 0.0f;
	bool data_valid = false;

	// 检查是否有有效的轨迹数据
	if (has_trajectory_data && PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
		// 计算水平速度大小
		horizontal_velocity_magnitude = sqrtf(matrix::Vector2f(trajectory_setpoint.velocity).norm_squared());
		// 使用参数作为增益
		pitch_setpoint = _param_ts_pitch_gain.get() * horizontal_velocity_magnitude;
		data_valid = true;
	}
	// 如果没有有效数据，使用默认值（已经初始化为0）

    // === 计算风速大小（在 pitch_setpoint 计算之后） ===
    const float wind_magnitude = get_wind_magnitude();
    static uint64_t last_wind_log = 0;
    if (now - last_wind_log > 1_s) {
        PX4_INFO("Trim Selector: wind=%.2f m/s", (double)wind_magnitude);
        last_wind_log = now;
    }

	// 创建并发布 utrim 消息（始终发布）
	utrim_s utrim{};
	utrim.timestamp = hrt_absolute_time();
	utrim.horizontal_velocity = horizontal_velocity_magnitude;
	utrim.valid = data_valid;

    // 注释掉基于速度和俯仰角的计算，改为都使用默认值
    /*
    if (data_valid) {
		// 有效数据：基于实际速度和俯仰角计算
        utrim.polynomial_values[0] = 5.0f + horizontal_velocity_magnitude * 2.0f;  // f1
        utrim.polynomial_values[1] = 5.0f + horizontal_velocity_magnitude * 2.0f;  // f2
        utrim.polynomial_values[2] = 8.0f + horizontal_velocity_magnitude * 1.5f;  // f3

		float pitch_degrees = pitch_setpoint * 180.0f / M_PI_F;
		utrim.polynomial_values[3] = pitch_degrees * 0.5f;  // theta1
		utrim.polynomial_values[4] = pitch_degrees * 0.5f;  // theta2
		utrim.polynomial_values[5] = pitch_degrees * 0.8f;  // theta3
        // 应用斜坡缩放，丝滑起飞/降落
        utrim.polynomial_values[0] *= _utrim_alpha;
        utrim.polynomial_values[1] *= _utrim_alpha;
        utrim.polynomial_values[2] *= _utrim_alpha;

    } else {
    */
		// 默认值：使用指定的推力值
        utrim.polynomial_values[0] = 10.1569f;  // f1
        utrim.polynomial_values[1] = 10.1569f;  // f2
        utrim.polynomial_values[2] = 8.1255f;   // f3
        // 应用斜坡缩放
        utrim.polynomial_values[0] *= _utrim_alpha;
        utrim.polynomial_values[1] *= _utrim_alpha;
        utrim.polynomial_values[2] *= _utrim_alpha;
		utrim.polynomial_values[3] = 0.0f;  // theta1 默认值
		utrim.polynomial_values[4] = 0.0f;  // theta2 默认值
		utrim.polynomial_values[5] = 0.0f;  // theta3 默认值

		// 在第一次使用默认值时打印计算信息
		static bool first_default_log = true;
		if (first_default_log) {
			PX4_INFO("Trim Selector: Using specified defaults - f1=%.4f, f2=%.4f, f3=%.4f",
				(double)utrim.polynomial_values[0], (double)utrim.polynomial_values[1], (double)utrim.polynomial_values[2]);
			first_default_log = false;
		}
	// }

	// 发布 utrim（始终发布）
	_utrim_pub.publish(utrim);

	// 创建并发布 theta_trim 消息（始终发布）
	theta_trim_s theta_trim{};
	theta_trim.timestamp = hrt_absolute_time();

	if (data_valid) {
		// 有效数据：基于计算的俯仰角
		theta_trim.pitch_angle = pitch_setpoint * 180.0f / M_PI_F;
	} else {
		// 默认值：零俯仰角
		theta_trim.pitch_angle = 0.0f;
	}

	// 发布 theta_trim（始终发布）
	_theta_trim_pub.publish(theta_trim);

	// 添加调试信息（每秒只打印一次，避免日志刷屏）
	static uint64_t last_log_time = 0;
	if (now - last_log_time > 1000000) { // 1秒
		PX4_INFO("Trim Selector: Using DEFAULT values for utrim");

		// 记录 utrim 发布数据
		PX4_INFO("Published utrim: f1=%.3f, f2=%.3f, f3=%.3f, θ1=%.3f°, θ2=%.3f°, θ3=%.3f°, valid=%s",
			(double)utrim.polynomial_values[0], (double)utrim.polynomial_values[1], (double)utrim.polynomial_values[2],
			(double)utrim.polynomial_values[3], (double)utrim.polynomial_values[4], (double)utrim.polynomial_values[5],
			utrim.valid ? "true" : "false");

		// 记录 theta_trim 发布数据
		PX4_INFO("Published theta_trim: pitch=%.3f°",
			(double)theta_trim.pitch_angle);

		last_log_time = now;
	}
}

int TrimSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
Trim Selector模块接收轨迹设定点，计算姿态设定点和控制分配参数，并将其发布到uORB。
特别是，它将计算俯仰角设定点，等于速度的2倍（theta = 2 * v_x），
同时发布utrim消息用于自定义控制分配。

### 实现
该模块订阅轨迹设定点，计算所需的姿态，并发布姿态设定点和控制分配参数。
发布的消息包括：
- theta_trim: 姿态设定点（四元数、推力、俯仰角）
- utrim: 控制分配参数（6个多项式值：f1,f2,f3,θ1,θ2,θ3）
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trim_selector", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TrimSelector::task_spawn(int argc, char *argv[])
{
	PX4_INFO("Starting Trim Selector module...");
	TrimSelector *instance = new TrimSelector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("Trim Selector module started successfully");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	PX4_ERR("Trim Selector module start failed");
	return PX4_ERROR;
}

int TrimSelector::custom_command(int argc, char *argv[])
{
	return print_usage("不支持的命令");
}

extern "C" __EXPORT int trim_selector_main(int argc, char *argv[])
{
	return TrimSelector::main(argc, argv);
}
