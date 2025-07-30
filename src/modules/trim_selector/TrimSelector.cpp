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

using namespace matrix;

TrimSelector::TrimSelector() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// 初始化参数更新
	parameters_update(true);

	// 初始化性能计数器
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME);
}

TrimSelector::~TrimSelector()
{
	perf_free(_loop_perf);
}

bool TrimSelector::init()
{
	// 初始化定时器，定期执行Run()函数
	ScheduleOnInterval(50_ms); // 20Hz 更新率，与 position control 相同
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

float TrimSelector::calculate_polynomial(float x, int polynomial_index)
{
	float result = 0.0f;

	switch (polynomial_index) {
		case 0: // Polynomial 1
		case 1: // Polynomial 2
		case 2: // Polynomial 3
			result = 0.01856f * x * x - 0.02238f * x + 10.03f;
			break;
		case 3: // Polynomial 4
		case 4: // Polynomial 5
			result = -0.01351f * x * x - 0.0001729f * x + 5.133f;
			break;
		case 5: // Polynomial 6
			result = -0.0183f * x * x - 0.00006f * x + 6.31f;
			break;
		default:
			break;
	}

	return result;
}

void TrimSelector::Run()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_loop_perf);

	// 首先检查参数是否需要更新
	parameters_update();

	// 订阅真空速
	airspeed_validated_s airspeed{};
	_airspeed_validated_sub.copy(&airspeed);

	// 订阅低速（地速）
	vehicle_local_position_s local_pos{};
	_vehicle_local_position_sub.copy(&local_pos);

	// 订阅轨迹设定点
	trajectory_setpoint_s trajectory_setpoint{};
	_trajectory_setpoint_sub.copy(&trajectory_setpoint);

	// PX4_INFO("trajectory_setpoint.velocity[x y z]: %.2f %.2f %.2f", (double)trajectory_setpoint.velocity[0], (double)trajectory_setpoint.velocity[1], (double)trajectory_setpoint.velocity[2]);

	// 计算风速
	if (PX4_ISFINITE(airspeed.true_airspeed_m_s) && local_pos.v_xy_valid) {
		// 计算地速
		const float ground_speed_x = local_pos.vx;
		const float ground_speed_y = local_pos.vy;

		// 计算风速（地速 - 空速）
		// 注意：这是一个简化的计算，假设飞机朝向与速度方向一致
		wind_s wind{};
		wind.timestamp = hrt_absolute_time();

		// 计算地速在水平面的大小和方向
		const float ground_speed = sqrtf(ground_speed_x * ground_speed_x + ground_speed_y * ground_speed_y);
		const float ground_course = atan2f(ground_speed_y, ground_speed_x);

		// 计算风速分量
		wind.windspeed_north = ground_speed_x - airspeed.true_airspeed_m_s * cosf(ground_course);
		wind.windspeed_east = ground_speed_y - airspeed.true_airspeed_m_s * sinf(ground_course);

		// 计算总风速大小（用于日志显示）
		const float wind_speed = sqrtf(wind.windspeed_north * wind.windspeed_north +
					  wind.windspeed_east * wind.windspeed_east);

		// 计算风向
		const float wind_direction = atan2f(wind.windspeed_east, wind.windspeed_north);

		wind.timestamp_sample = local_pos.timestamp;

		_wind_pub.publish(wind);

		// 添加到日志消息
		_log_message.timestamp = hrt_absolute_time();
		_log_message.severity = 6; // info
		snprintf(_log_message.text, sizeof(_log_message.text),
			 "WIND: gs=%.2f, tas=%.2f, ws=%.2f, wd=%.1f",
			 (double)ground_speed,
			 (double)airspeed.true_airspeed_m_s,
			 (double)wind_speed,
			 (double)math::degrees(wind_direction));
		_log_message_pub.publish(_log_message);
	}

	// 计算水平速度和发布utrim
	if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
		// 计算水平速度大小
		float horizontal_velocity_magnitude = sqrtf(matrix::Vector2f(trajectory_setpoint.velocity).norm_squared());

		// 创建并填充utrim消息
		utrim_s utrim{};
		utrim.timestamp = hrt_absolute_time();
		utrim.horizontal_velocity = horizontal_velocity_magnitude;
		utrim.valid = true;

		// 计算6个多项式值
		for (int i = 0; i < 6; i++) {
			utrim.polynomial_values[i] = calculate_polynomial(horizontal_velocity_magnitude, i);
		}

		// 发布utrim消息
		_utrim_pub.publish(utrim);

		// 添加到日志
		_log_message.timestamp = hrt_absolute_time();
		_log_message.severity = 6; // info
		snprintf(_log_message.text, sizeof(_log_message.text),
			 "UTRIM: v=%.2f, p1=%.2f, p2=%.2f, p3=%.2f, p4=%.2f, p5=%.2f, p6=%.2f",
			 (double)utrim.horizontal_velocity,
			 (double)utrim.polynomial_values[0],
			 (double)utrim.polynomial_values[1],
			 (double)utrim.polynomial_values[2],
			 (double)utrim.polynomial_values[3],
			 (double)utrim.polynomial_values[4],
			 (double)utrim.polynomial_values[5]);
		_log_message_pub.publish(_log_message);
	}

	// 计算俯仰角期望和发布theta_trim（原有逻辑）
	if (PX4_ISFINITE(trajectory_setpoint.velocity[0]) && PX4_ISFINITE(trajectory_setpoint.velocity[1])) {
		float horizontal_velocity_magnitude = sqrtf(matrix::Vector2f(trajectory_setpoint.velocity).norm_squared());

		float pitch_setpoint = 0.0f;
		if (horizontal_velocity_magnitude > 2.0f) {
			// float pitch_setpoint = _param_ts_pitch_gain.get() * horizontal_velocity_magnitude;
			pitch_setpoint = 10.0f; // 如果速度大于2.0m/s，设置为10度
		} else {
			pitch_setpoint = 0.0f; // 如果速度小于等于2.0m/s，设置为0度
		}

		theta_trim_s theta_trim{};
		theta_trim.timestamp = hrt_absolute_time();
		theta_trim.pitch_angle = pitch_setpoint; // 直接发布俯仰角度


		_theta_trim_pub.publish(theta_trim);
		}


	perf_end(_loop_perf);
}

int TrimSelector::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### 描述
Trim Selector模块接收轨迹设定点，计算姿态设定点，并将其发布到uORB。
特别是，它将根据水平速度计算俯仰角设定点：当速度大于2.0m/s时为10度，否则为0度。

### 实现
该模块订阅轨迹设定点，计算所需的姿态，并发布姿态设定点。
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("trim_selector", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int TrimSelector::task_spawn(int argc, char *argv[])
{
	TrimSelector *instance = new TrimSelector();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

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
