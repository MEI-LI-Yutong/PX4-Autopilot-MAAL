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
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

using namespace matrix;

namespace ControlMath
{
static constexpr float MAX_ROLL = 25.0f * M_PI_F / 180.0f;

void thrustToAttitude(const Vector3f &thr_sp, const Vector3f &thr_sp_increment, const float yaw_sp, const float pitch_sp, vehicle_attitude_setpoint_s &att_sp)
{
	// 1) 先得到“无 roll”的旋转矩阵（body→world），并将 thr_sp 转到机体系
	const Eulerf eul_no_roll(0.f, pitch_sp, yaw_sp);
	const Dcmf   R_yaw_pitch(eul_no_roll);
	const Vector3f thr_body0 = R_yaw_pitch.transpose() * thr_sp;

	// 2) 用机体 Y/Z 推力解 roll（右倾为＋），并限幅
	float roll_cmd = atan2f(-thr_body0(1), -thr_body0(2));
	roll_cmd = math::constrain(roll_cmd, -MAX_ROLL, MAX_ROLL);
	roll_cmd = 0.0f;

	// 3) 生成最终姿态四元数 (roll, pitch, yaw)
	const Eulerf eul_final(roll_cmd, pitch_sp, yaw_sp);
	const Quatf  q_sp(eul_final);
	q_sp.copyTo(att_sp.q_d);

	// 4) 使用最终姿态将 thr_sp 再次转换到机体系，得到真正机体推力
	const Dcmf R_final(eul_final);
	const Vector3f thrust_body = R_final.transpose() * thr_sp;
	att_sp.thrust_body[0] = 0.1f;
	att_sp.thrust_body[1] = 0.1f;
	att_sp.thrust_body[2] = thrust_body(2);

	// 调试输出
	PX4_INFO("ControlMath: thr_sp[%.3f,%.3f,%.3f] -> thrust_body[%.3f,%.3f,%.3f]",
	         (double)thr_sp(0), (double)thr_sp(1), (double)thr_sp(2),
	         (double)thrust_body(0), (double)thrust_body(1), (double)thrust_body(2));
	PX4_INFO("ControlMath: R_final matrix[0]=[%.3f,%.3f,%.3f]",
	         (double)R_final(0,0), (double)R_final(0,1), (double)R_final(0,2));
	PX4_INFO("ControlMath: R_final matrix[1]=[%.3f,%.3f,%.3f]",
	         (double)R_final(1,0), (double)R_final(1,1), (double)R_final(1,2));
	PX4_INFO("ControlMath: R_final matrix[2]=[%.3f,%.3f,%.3f]",
	         (double)R_final(2,0), (double)R_final(2,1), (double)R_final(2,2));

	// 注意：这里会覆盖位置控制器设置的增量推力
	// 如果你想保持位置控制器的增量推力，请注释掉上面的赋值

        // 原来的实现（直接拷贝到 thrust_body）保留在下方注释
        // att_sp.thrust_body[0] = thr_sp(0);
        // att_sp.thrust_body[1] = thr_sp(1);
        // att_sp.thrust_body[2] = thr_sp(2);

        // 原来把 thr 压缩为标量长度的实现：
        // bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
        // att_sp.thrust_body[2] = -thr_sp.length();
	// 10Hz 限频，仅打印 att_sp.thrust_body

	static hrt_abstime last_tb_log = 0;
	hrt_abstime now_tb = hrt_absolute_time();
	if (now_tb - last_tb_log >= 100_ms) {
		PX4_INFO("CM: thrust_body=%.3f %.3f %.3f",
			(double)att_sp.thrust_body[0], (double)att_sp.thrust_body[1], (double)att_sp.thrust_body[2]);
		last_tb_log = now_tb;
	}
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

// void bodyzToAttitude(Vector3f body_z, const float yaw_sp, const float pitch_sp, vehicle_attitude_setpoint_s &att_sp)
// {
// 	// 定义期望的机体 Z 轴方向 d = -normalize(body_z)
// 	Vector3f d = -body_z;
// 	if (d.norm_squared() < FLT_EPSILON) {
// 		d = Vector3f(0.f, 0.f, 1.f); // 默认向上
// 	} else {
// 		d.normalize();
// 	}
// 	// /* 1) 用期望 yaw、pitch 先建立参考坐标系（无 roll） */
// 	// Dcmf R_yaw_pitch(Eulerf(0.f, pitch_sp, yaw_sp));   // R = Rz(yaw) · Ry(pitch)

// 	// /* 2) 把期望机体 Z 轴旋回参考系下 */
// 	// Vector3f d_local = R_yaw_pitch.transpose() * d;
// 	// d_local.normalize();                               // 数值安全

// 	// /* 3) 根据几何关系解 roll。d_local ≈ [0, -sinR, cosR]ᵀ */
// 	// float roll_sp = atan2f(-d_local(1), d_local(2));

// 	// /* 4) 限幅 */
// 	// roll_sp = math::constrain(roll_sp, -M_PI_F / 6.f, M_PI_F / 6.f);

// 		// 固定 roll = 0°
// 	float roll_sp = 0.0f;


// 	// 构造最终姿态：R_sp = Rz(yaw_sp) * Ry(pitch_sp) * Rx(roll_sp)
// 	const Eulerf euler_final(roll_sp, pitch_sp, 0.7f);
// 	Dcmf R_sp(euler_final);
// 	const Quatf q_sp{euler_final};
// 	q_sp.copyTo(att_sp.q_d);
// }

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

} // ControlMath
