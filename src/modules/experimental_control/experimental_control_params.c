/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file experimental_control_params.c
 * Parameters for the experimental control module.
 */

#include <px4_platform_common/param.h>

/**
 * Experimental controller backend selector
 *
 * 0: Zero/stub backend (outputs zero to actuators)
 * Future values can select other in-firmware controllers.
 *
 * @min 0
 * @max 10
 * @group Experimental Control
 */
PARAM_DEFINE_INT32(EXP_CTRL_BACKEND, 0);

/**
 * Max motor RPM used for NN rescaling
 *
 * @min 0
 * @max 30000
 * @group Experimental Control
 */
PARAM_DEFINE_INT32(MC_NN_MAX_RPM, 12000);

/**
 * Min motor RPM used for NN rescaling
 *
 * @min 0
 * @max 10000
 * @group Experimental Control
 */
PARAM_DEFINE_INT32(MC_NN_MIN_RPM, 1000);

/**
 * Thrust coefficient used for NN rescaling
 *
 * Converts normalized thrust output into RPM. This is typically the quadratic
 * coefficient of thrust vs RPM.
 *
 * @min 1
 * @max 500000
 * @decimal 1
 * @group Experimental Control
 */
PARAM_DEFINE_FLOAT(MC_NN_THRST_COEF, 100000.f);

/**
 * Use manual control inputs for trajectory generation
 *
 * If enabled, manual roll/pitch/throttle generate velocity setpoints; otherwise
 * offboard trajectory_setpoint is used.
 *
 * @boolean
 * @group Experimental Control
 */
PARAM_DEFINE_INT32(MC_NN_MANL_CTRL, 1);

/**
 * Servo maximum angle (rad)
 *
 * @min -6.0
 * @max 6.0
 * @decimal 3
 * @group Experimental Control
 */
PARAM_DEFINE_FLOAT(MC_NN_SV_MAX_ANG, 0.523599f);

/**
 * Servo minimum angle (rad)
 *
 * @min -6.0
 * @max 6.0
 * @decimal 3
 * @group Experimental Control
 */
PARAM_DEFINE_FLOAT(MC_NN_SV_MIN_ANG, -0.523599f);
