/****************************************************************************
 *
 *   Copyright (C) 2018 - 2019 PX4 Development Team. All rights reserved.
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

/**
 * @file ControlMath.cpp
 */

#include "ControlMath.hpp"
#include <px4_platform_common/defines.h>
#include <float.h>
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

using namespace matrix;

namespace ControlMath
{
// void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
// {
// 	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
// 	att_sp.thrust_body[2] = -thr_sp.length();
// }

void thrustToAttitude(const Vector3f &thr_sp, const float yaw_sp, const float pitch_sp, vehicle_attitude_setpoint_s &att_sp, const float dt, float &tilt_prev)
{
	bodyzToAttitude(-thr_sp, yaw_sp, pitch_sp, att_sp);

	// 从姿态四元数中重构旋转矩阵
	Quatf q_sp(att_sp.q_d[0], att_sp.q_d[1], att_sp.q_d[2], att_sp.q_d[3]);
	Dcmf R_sp(q_sp);

	// 使用从bodyzToAttitude函数中计算的roll_sp调整thr_sp(2)
	Vector3f thr_sp_adjusted = thr_sp;

	// 将世界坐标系的推力转换到机体坐标系
	const Vector3f thrust_body = R_sp.transpose() * thr_sp_adjusted;
	float tilt_extra_angle = atan2f(thrust_body(0), fabsf(thrust_body(2)));
	tilt_extra_angle = math::constrain(tilt_extra_angle, -M_PI_F/6.0f, M_PI_F/6.0f);

	// Apply rate limit: max 180°/s (π rad/s)
	const float tilt_rate_max = M_PI_F; // π rad/s = 180°/s
	const float tilt_change_max = tilt_rate_max * dt;
	const float tilt_change = tilt_extra_angle - tilt_prev;
	const float tilt_change_limited = math::constrain(tilt_change, -tilt_change_max, tilt_change_max);
	tilt_extra_angle = tilt_prev + tilt_change_limited;
	tilt_prev = tilt_extra_angle;

	// float T_total = fabsf(thrust_body(2)) / cosf(tilt_extra_angle);
	// float T_total = thrust_body.length();
	// 设置机体坐标系推力
	att_sp.thrust_body[0] = thrust_body(0);
	// att_sp.thrust_body[1] = thrust_body(1);
	att_sp.thrust_body[2] = thrust_body(2);
	att_sp.tilt_extra_angle = tilt_extra_angle;
}

void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, const float max_angle)
{
	// determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	// limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// corner case exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

// void bodyzToAttitude(Vector3f body_z, const float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
// {
// 	// zero vector, no direction, set safe level value
// 	if (body_z.norm_squared() < FLT_EPSILON) {
// 		body_z(2) = 1.f;
// 	}

// 	body_z.normalize();

// 	// vector of desired yaw direction in XY plane, rotated by PI/2
// 	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

// 	// desired body_x axis, orthogonal to body_z
// 	Vector3f body_x = y_C % body_z;

// 	// keep nose to front while inverted upside down
// 	if (body_z(2) < 0.f) {
// 		body_x = -body_x;
// 	}

// 	if (fabsf(body_z(2)) < 0.000001f) {
// 		// desired thrust is in XY plane, set X downside to construct correct matrix,
// 		// but yaw component will not be used actually
// 		body_x.zero();
// 		body_x(2) = 1.f;
// 	}

// 	body_x.normalize();

// 	// desired body_y axis
// 	const Vector3f body_y = body_z % body_x;

// 	Dcmf R_sp;

// 	// fill rotation matrix
// 	for (int i = 0; i < 3; i++) {
// 		R_sp(i, 0) = body_x(i);
// 		R_sp(i, 1) = body_y(i);
// 		R_sp(i, 2) = body_z(i);
// 	}

// 	// copy quaternion setpoint to attitude setpoint topic
// 	const Quatf q_sp{R_sp};
// 	q_sp.copyTo(att_sp.q_d);
// }

float bodyzToAttitude(Vector3f body_z_des, const float yaw_sp, const float pitch_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// zero vector, no direction, set safe level value
	if (body_z_des.norm_squared() < FLT_EPSILON) {
		body_z_des(2) = 1.f;
	}

	body_z_des.normalize();

	/* 根据 yaw_sp，pitch_sp，将世界坐标系的 thr_sp 旋转到机体坐标系 */
	/* roll 先假设为 0，将力转换方向，得到机体坐标系的力期望 */

	/* 2) 在已定 yaw+pitch 坐标系里解 roll */
	Dcmf R_yaw_pitch(Eulerf(0.f, pitch_sp, yaw_sp));      // R = Rz*yaw · Ry*pitch
	Vector3f d_local = R_yaw_pitch.transpose() * body_z_des;
	float roll_sp = atan2f(-d_local(1), d_local(2));
	// roll_sp = matrix::wrap_pi(roll_sp);
	roll_sp = math::constrain(roll_sp, -20.0f * M_PI_F / 180.0f, 20.0f * M_PI_F / 180.0f);

	// 构建最终的旋转矩阵
	Dcmf R_sp(Eulerf(roll_sp, pitch_sp, yaw_sp));

	// copy quaternion setpoint to attitude setpoint topic
	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	return roll_sp;
}

Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, const float &max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		// vector does not exceed maximum magnitude
		return v0 + v1;

	} else if (v0.length() >= max) {
		// the magnitude along v0, which has priority, already exceeds maximum.
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		// the two vectors are equal
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		// the first vector is 0.
		return v1.normalized() * max;

	} else {
		// vf = final vector with ||vf|| <= max
		// s = scaling factor
		// u1 = unit of v1
		// vf = v0 + v1 = v0 + s * u1
		// constraint: ||vf|| <= max
		//
		// solve for s: ||vf|| = ||v0 + s * u1|| <= max
		//
		// Derivation:
		// For simplicity, replace v0 -> v, u1 -> u
		// 				   		   v0(0/1/2) -> v0/1/2
		// 				   		   u1(0/1/2) -> u0/1/2
		//
		// ||v + s * u||^2 = (v0+s*u0)^2+(v1+s*u1)^2+(v2+s*u2)^2 = max^2
		// v0^2+2*s*u0*v0+s^2*u0^2 + v1^2+2*s*u1*v1+s^2*u1^2 + v2^2+2*s*u2*v2+s^2*u2^2 = max^2
		// s^2*(u0^2+u1^2+u2^2) + s*2*(u0*v0+u1*v1+u2*v2) + (v0^2+v1^2+v2^2-max^2) = 0
		//
		// quadratic equation:
		// -> s^2*a + s*b + c = 0 with solution: s1/2 = (-b +- sqrt(b^2 - 4*a*c))/(2*a)
		//
		// b = 2 * u.dot(v)
		// a = 1 (because u is normalized)
		// c = (v0^2+v1^2+v2^2-max^2) = -max^2 + ||v||^2
		//
		// sqrt(b^2 - 4*a*c) =
		// 		sqrt(4*u.dot(v)^2 - 4*(||v||^2 - max^2)) = 2*sqrt(u.dot(v)^2 +- (||v||^2 -max^2))
		//
		// s1/2 = ( -2*u.dot(v) +- 2*sqrt(u.dot(v)^2 - (||v||^2 -max^2)) / 2
		//      =  -u.dot(v) +- sqrt(u.dot(v)^2 - (||v||^2 -max^2))
		// m = u.dot(v)
		// s = -m + sqrt(m^2 - c)
		//
		//
		//
		// notes:
		// 	- s (=scaling factor) needs to be positive
		// 	- (max - ||v||) always larger than zero, otherwise it never entered this if-statement
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

bool cross_sphere_line(const Vector3f &sphere_c, const float sphere_r,
		       const Vector3f &line_a, const Vector3f &line_b, Vector3f &res)
{
	// project center of sphere on line  normalized AB
	Vector3f ab_norm = line_b - line_a;

	if (ab_norm.length() < 0.01f) {
		return true;
	}

	ab_norm.normalize();
	Vector3f d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		// we have triangle CDX with known CD and CX = R, find DX
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.f) {
			// target waypoint is already behind us
			res = line_b;

		} else {
			// target is in front of us
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {

		// have no roots, return D
		res = d; // go directly to line

		// previous waypoint is still in front of us
		if ((sphere_c - line_a) * ab_norm < 0.f) {
			res = line_a;
		}

		// target waypoint is already behind us
		if ((sphere_c - line_b) * ab_norm > 0.f) {
			res = line_b;
		}

		return false;
	}
}

void addIfNotNan(float &setpoint, const float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		// No NAN, add to the setpoint
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		// Setpoint NAN, take addition
		setpoint = addition;
	}

	// Addition is NAN or both are NAN, nothing to do
}

void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

void setZeroIfNanVector3f(Vector3f &vector)
{
	// Adding zero vector overwrites elements that are NaN with zero
	addIfNotNanVector3f(vector, Vector3f());
}

bool thrustToAttitudeDecoupled(const Vector3f &F_world,
                               float yaw_sp,
                               float pitch_fixed,
                               float fy_for_roll,
                               float roll_limit_rad,
                               vehicle_attitude_setpoint_s &att_sp,
                               Vector3f &F_body_out)
{
	// 1. Create thrust vector for attitude calculation using only (0, Fy, Fz)
	Vector3f thr_for_att(0.f, fy_for_roll, F_world(2));

	if (thr_for_att.norm_squared() < 1e-6f) {
		thr_for_att = Vector3f(0.f, 0.f, 1.f); // avoid zero vector
	}

	// 2. Desired body Z-axis direction (world frame): body_z_des_world = -normalize(thr_for_att)
	Vector3f body_z_des_world = -thr_for_att.normalized();

	// 3. Build yaw + pitch fixed attitude matrix
	Dcmf R_yaw_pitch(Eulerf(0.f, pitch_fixed, yaw_sp));

	// 4. Transform to "yaw+pitch pre-aligned" coordinate system to solve for roll
	Vector3f d_local = R_yaw_pitch.transpose() * body_z_des_world;

	// 5. Roll calculation: rotate around body X-axis to align Z-axis with body_z_des_world
	float roll_sp = atan2f(-d_local(1), d_local(2));
	if (PX4_ISFINITE(roll_limit_rad) && roll_limit_rad > 0.f) {
		roll_sp = math::constrain(roll_sp, -roll_limit_rad, roll_limit_rad);
	}

	// 6. Final attitude matrix
	Dcmf R_sp(Eulerf(roll_sp, pitch_fixed, yaw_sp));
	Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);

	// 7. Fill attitude setpoint structure
	att_sp.yaw_sp_move_rate = 0.f; // will be set by upper layer
	att_sp.thrust_body[0] = NAN;   // not used (use vehicle_thrust_setpoint instead)
	att_sp.thrust_body[1] = NAN;
	att_sp.thrust_body[2] = NAN;
	att_sp.tilt_extra_angle = 0.f;
	att_sp.reset_integral = false;
	att_sp.fw_control_yaw_wheel = false;

	// 8. Transform world thrust to body frame: F_body = R_sp^T * F_world
	F_body_out = R_sp.transpose() * F_world;

	return true;
}

void thrustToAttitudeDecoupled(const Vector3f &thr_sp,
                               const float yaw_sp,
                               const float pitch_sp,
                               vehicle_attitude_setpoint_s &att_sp,
                               const float dt,
                               float &tilt_prev,
                               uORB::Publication<vehicle_thrust_setpoint_s> *thrust_pub)
{
	// Use decoupled approach: Fx handled by geometry, attitude only uses (Fy=0, Fz)
	float fy_for_roll = 0.f; // First phase: no lateral control
	float roll_limit = math::radians(20.f); // Roll limit
	Vector3f F_body;

	// Call the decoupled algorithm
	thrustToAttitudeDecoupled(thr_sp, yaw_sp, pitch_sp, fy_for_roll, roll_limit, att_sp, F_body);

	// Publish 3D body thrust if publisher provided
	if (thrust_pub != nullptr) {
		vehicle_thrust_setpoint_s thrust_setpoint{};
		thrust_setpoint.timestamp = hrt_absolute_time();
		thrust_setpoint.xyz[0] = F_body(0);
		thrust_setpoint.xyz[1] = F_body(1);
		thrust_setpoint.xyz[2] = F_body(2);
		thrust_pub->publish(thrust_setpoint);
	}
}

} // ControlMath
