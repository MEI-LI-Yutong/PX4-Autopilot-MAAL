/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file ControlAllocator.cpp
 *
 * Control allocator.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */

#include "ControlAllocator.hpp"

#include <drivers/drv_hrt.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <matrix/matrix/math.hpp>
#include <lib/geo/geo.h>

using namespace matrix;
using namespace time_literals;

ControlAllocator::ControlAllocator() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	_control_allocator_status_pub[0].advertise();
	_control_allocator_status_pub[1].advertise();

	_actuator_motors_pub.advertise();
	_actuator_servos_pub.advertise();
	_actuator_servos_trim_pub.advertise();

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_R%u_SLEW", i);
		_param_handles.slew_rate_motors[i] = param_find(buffer);
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		char buffer[17];
		snprintf(buffer, sizeof(buffer), "CA_SV%u_SLEW", i);
		_param_handles.slew_rate_servos[i] = param_find(buffer);
	}

	parameters_updated();
}

ControlAllocator::~ControlAllocator()
{
	for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
		delete _control_allocation[i];
	}

	delete _actuator_effectiveness;

	perf_free(_loop_perf);
}

bool
ControlAllocator::init()
{
	if (!_vehicle_torque_setpoint_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	ScheduleDelayed(50_ms);
#endif

	return true;
}

void
ControlAllocator::parameters_updated()
{
	_has_slew_rate = false;

	for (int i = 0; i < MAX_NUM_MOTORS; ++i) {
		param_get(_param_handles.slew_rate_motors[i], &_params.slew_rate_motors[i]);
		_has_slew_rate |= _params.slew_rate_motors[i] > FLT_EPSILON;
	}

	for (int i = 0; i < MAX_NUM_SERVOS; ++i) {
		param_get(_param_handles.slew_rate_servos[i], &_params.slew_rate_servos[i]);
		_has_slew_rate |= _params.slew_rate_servos[i] > FLT_EPSILON;
	}

	// Allocation method & effectiveness source
	// Do this first: in case a new method is loaded, it will be configured below
	bool updated = update_effectiveness_source();
	update_allocation_method(updated); // must be called after update_effectiveness_source()

	if (_num_control_allocation == 0) {
		return;
	}

	for (int i = 0; i < _num_control_allocation; ++i) {
		_control_allocation[i]->updateParameters();
	}

	update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::CONFIGURATION_UPDATE);
}

void
ControlAllocator::update_allocation_method(bool force)
{
	AllocationMethod configured_method = (AllocationMethod)_param_ca_method.get();

	if (!_actuator_effectiveness) {
		PX4_ERR("_actuator_effectiveness null");
		return;
	}

	if (_allocation_method_id != configured_method || force) {

		matrix::Vector<float, NUM_ACTUATORS> actuator_sp[ActuatorEffectiveness::MAX_NUM_MATRICES];

		// Cleanup first
		for (int i = 0; i < ActuatorEffectiveness::MAX_NUM_MATRICES; ++i) {
			// Save current state
			if (_control_allocation[i] != nullptr) {
				actuator_sp[i] = _control_allocation[i]->getActuatorSetpoint();
			}

			delete _control_allocation[i];
			_control_allocation[i] = nullptr;
		}

		_num_control_allocation = _actuator_effectiveness->numMatrices();

		AllocationMethod desired_methods[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getDesiredAllocationMethod(desired_methods);

		bool normalize_rpy[ActuatorEffectiveness::MAX_NUM_MATRICES];
		_actuator_effectiveness->getNormalizeRPY(normalize_rpy);

		for (int i = 0; i < _num_control_allocation; ++i) {
			AllocationMethod method = configured_method;

			if (configured_method == AllocationMethod::AUTO) {
				method = desired_methods[i];
			}

			switch (method) {
			case AllocationMethod::PSEUDO_INVERSE:
				_control_allocation[i] = new ControlAllocationPseudoInverse();
				break;

			case AllocationMethod::SEQUENTIAL_DESATURATION:
				_control_allocation[i] = new ControlAllocationSequentialDesaturation();
				break;

			default:
				PX4_ERR("Unknown allocation method");
				break;
			}

			if (_control_allocation[i] == nullptr) {
				PX4_ERR("alloc failed");
				_num_control_allocation = 0;

			} else {
				_control_allocation[i]->setNormalizeRPY(normalize_rpy[i]);
				_control_allocation[i]->setActuatorSetpoint(actuator_sp[i]);
			}
		}

		_allocation_method_id = configured_method;
	}
}

bool
ControlAllocator::update_effectiveness_source()
{
	const EffectivenessSource source = (EffectivenessSource)_param_ca_airframe.get();

	if (_effectiveness_source_id != source) {

		// try to instanciate new effectiveness source
		ActuatorEffectiveness *tmp = nullptr;

		switch (source) {
		case EffectivenessSource::NONE:
		case EffectivenessSource::MULTIROTOR:
			tmp = new ActuatorEffectivenessMultirotor(this);
			break;

		case EffectivenessSource::STANDARD_VTOL:
			tmp = new ActuatorEffectivenessStandardVTOL(this);
			break;

		case EffectivenessSource::TILTROTOR_VTOL:
			tmp = new ActuatorEffectivenessTiltrotorVTOL(this);
			break;

		case EffectivenessSource::TAILSITTER_VTOL:
			tmp = new ActuatorEffectivenessTailsitterVTOL(this);
			break;

		case EffectivenessSource::ROVER_ACKERMANN:
			tmp = new ActuatorEffectivenessRoverAckermann();
			break;

		case EffectivenessSource::ROVER_DIFFERENTIAL:
			// rover_differential_control does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::FIXED_WING:
			tmp = new ActuatorEffectivenessFixedWing(this);
			break;

		case EffectivenessSource::MOTORS_6DOF: // just a different UI from MULTIROTOR
			tmp = new ActuatorEffectivenessUUV(this);
			break;

		case EffectivenessSource::MULTIROTOR_WITH_TILT:
			tmp = new ActuatorEffectivenessMCTilt(this);
			break;

		case EffectivenessSource::CUSTOM:
			tmp = new ActuatorEffectivenessCustom(this);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_ESC:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::MOTORS);
			break;

		case EffectivenessSource::HELICOPTER_TAIL_SERVO:
			tmp = new ActuatorEffectivenessHelicopter(this, ActuatorType::SERVOS);
			break;

		case EffectivenessSource::HELICOPTER_COAXIAL:
			tmp = new ActuatorEffectivenessHelicopterCoaxial(this);
			break;

		case EffectivenessSource::SPACECRAFT_2D:
			// spacecraft_allocation does allocation and publishes directly to actuator_motors topic
			break;

		case EffectivenessSource::SPACECRAFT_3D:
			// spacecraft_allocation does allocation and publishes directly to actuator_motors topic
			break;

		default:
			PX4_ERR("Unknown airframe");
			break;
		}

		// Replace previous source with new one
		if (tmp == nullptr) {
			// It did not work, forget about it
			PX4_ERR("Actuator effectiveness init failed");
			_param_ca_airframe.set((int)_effectiveness_source_id);

		} else {
			// Swap effectiveness sources
			delete _actuator_effectiveness;
			_actuator_effectiveness = tmp;

			// Save source id
			_effectiveness_source_id = source;
		}

		return true;
	}

	return false;
}

void
ControlAllocator::Run()
{
	if (should_exit()) {
		_vehicle_torque_setpoint_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

#ifndef ENABLE_LOCKSTEP_SCHEDULER // Backup schedule would interfere with lockstep
	// Push backup schedule
	ScheduleDelayed(50_ms);
#endif

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		if (_handled_motor_failure_bitmask == 0) {
			// We don't update the geometry after an actuator failure, as it could lead to unexpected results
			// (e.g. a user could add/remove motors, such that the bitmask isn't correct anymore)
			updateParams();
			parameters_updated();
		}
	}

	if (_num_control_allocation == 0 || _actuator_effectiveness == nullptr) {
		return;
	}

	{
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.update(&vehicle_status)) {

			_armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;

			ActuatorEffectiveness::FlightPhase flight_phase{ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT};

			// Check if the current flight phase is HOVER or FIXED_WING
			if (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
				flight_phase = ActuatorEffectiveness::FlightPhase::HOVER_FLIGHT;

			} else {
				flight_phase = ActuatorEffectiveness::FlightPhase::FORWARD_FLIGHT;
			}

			// Special cases for VTOL in transition
			if (vehicle_status.is_vtol && vehicle_status.in_transition_mode) {
				if (vehicle_status.in_transition_to_fw) {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_HF_TO_FF;

				} else {
					flight_phase = ActuatorEffectiveness::FlightPhase::TRANSITION_FF_TO_HF;
				}
			}

			// Forward to effectiveness source
			_actuator_effectiveness->setFlightPhase(flight_phase);
		}
	}

	{
		vehicle_control_mode_s vehicle_control_mode;

		if (_vehicle_control_mode_sub.update(&vehicle_control_mode)) {
			_publish_controls = vehicle_control_mode.flag_control_allocation_enabled;
		}
	}

	// Guard against too small (< 0.2ms) and too large (> 20ms) dt's.
	const hrt_abstime now = hrt_absolute_time();
	const float dt = math::constrain(((now - _last_run) / 1e6f), 0.0002f, 0.02f);

	bool do_update = false;
	vehicle_torque_setpoint_s vehicle_torque_setpoint;
	vehicle_thrust_setpoint_s vehicle_thrust_setpoint;

	// Run allocator on torque changes
	if (_vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint)) {
		_torque_sp = matrix::Vector3f(vehicle_torque_setpoint.xyz);

		do_update = true;
		_timestamp_sample = vehicle_torque_setpoint.timestamp_sample;

	}

	if (_vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint)) {
		_thrust_sp = matrix::Vector3f(vehicle_thrust_setpoint.xyz);
	}

	if (do_update) {
		_last_run = now;

		check_for_motor_failures();

		update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::NO_EXTERNAL_UPDATE);

		// 更新 utrim 状态
		update_utrim_status();

		// 首先尝试自定义分配
		calculate_custom_allocation();

		// Set control setpoint vector(s)
		matrix::Vector<float, NUM_AXES> c[ActuatorEffectiveness::MAX_NUM_MATRICES];
		c[0](0) = _torque_sp(0);
		c[0](1) = _torque_sp(1);
		c[0](2) = _torque_sp(2);
		c[0](3) = _thrust_sp(0);
		c[0](4) = _thrust_sp(1);
		c[0](5) = _thrust_sp(2);

		if (_num_control_allocation > 1) {
			if (_vehicle_torque_setpoint1_sub.copy(&vehicle_torque_setpoint)) {
				c[1](0) = vehicle_torque_setpoint.xyz[0];
				c[1](1) = vehicle_torque_setpoint.xyz[1];
				c[1](2) = vehicle_torque_setpoint.xyz[2];
			}

			if (_vehicle_thrust_setpoint1_sub.copy(&vehicle_thrust_setpoint)) {
				c[1](3) = vehicle_thrust_setpoint.xyz[0];
				c[1](4) = vehicle_thrust_setpoint.xyz[1];
				c[1](5) = vehicle_thrust_setpoint.xyz[2];
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {

			_control_allocation[i]->setControlSetpoint(c[i]);

			// 尝试使用自定义分配
			if (_custom_allocation_valid) {
				// 使用自定义分配结果
				matrix::Vector<float, NUM_ACTUATORS> custom_actuator_sp;
				if (apply_custom_allocation(custom_actuator_sp)) {
					_control_allocation[i]->setActuatorSetpoint(custom_actuator_sp);
					// PX4_INFO("使用自定义控制分配");
				} else {
					// 回退到标准分配
					_control_allocation[i]->allocate();
				}
			} else {
				// 回退到标准分配
				_control_allocation[i]->allocate();
			}

			// 获取当前的执行器设定点用于后续处理
			matrix::Vector<float, NUM_ACTUATORS> actuator_sp = _control_allocation[i]->getActuatorSetpoint();

			_actuator_effectiveness->allocateAuxilaryControls(dt, i, actuator_sp); //flaps and spoilers
			_actuator_effectiveness->updateSetpoint(c[i], i, actuator_sp,
								_control_allocation[i]->getActuatorMin(), _control_allocation[i]->getActuatorMax());

			// 更新执行器设定点
			_control_allocation[i]->setActuatorSetpoint(actuator_sp);

			if (_has_slew_rate) {
				_control_allocation[i]->applySlewRateLimit(dt);
			}

			_control_allocation[i]->clipActuatorSetpoint();
		}
	}

	// Publish actuator setpoint and allocator status
	publish_actuator_controls();

	// Publish status at limited rate, as it's somewhat expensive and we use it for slower dynamics
	// (i.e. anti-integrator windup)
	if (now - _last_status_pub >= 5_ms) {
		publish_control_allocator_status(0);

		if (_num_control_allocation > 1) {
			publish_control_allocator_status(1);
		}

		_last_status_pub = now;
	}

	perf_end(_loop_perf);
}

void
ControlAllocator::update_effectiveness_matrix_if_needed(EffectivenessUpdateReason reason)
{
	ActuatorEffectiveness::Configuration config{};

	if (reason == EffectivenessUpdateReason::NO_EXTERNAL_UPDATE
	    && hrt_elapsed_time(&_last_effectiveness_update) < 100_ms) { // rate-limit updates
		return;
	}

	if (_actuator_effectiveness->getEffectivenessMatrix(config, reason)) {
		_last_effectiveness_update = hrt_absolute_time();

		memcpy(_control_allocation_selection_indexes, config.matrix_selection_indexes,
		       sizeof(_control_allocation_selection_indexes));

		// Get the minimum and maximum depending on type and configuration
		ActuatorEffectiveness::ActuatorVector minimum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector maximum[ActuatorEffectiveness::MAX_NUM_MATRICES];
		ActuatorEffectiveness::ActuatorVector slew_rate[ActuatorEffectiveness::MAX_NUM_MATRICES];
		int actuator_idx = 0;
		int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

		actuator_servos_trim_s trims{};
		static_assert(actuator_servos_trim_s::NUM_CONTROLS == actuator_servos_s::NUM_CONTROLS, "size mismatch");

		for (int actuator_type = 0; actuator_type < (int)ActuatorType::COUNT; ++actuator_type) {
			_num_actuators[actuator_type] = config.num_actuators[actuator_type];

			for (int actuator_type_idx = 0; actuator_type_idx < config.num_actuators[actuator_type]; ++actuator_type_idx) {
				if (actuator_idx >= NUM_ACTUATORS) {
					_num_actuators[actuator_type] = 0;
					PX4_ERR("Too many actuators");
					break;
				}

				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if ((ActuatorType)actuator_type == ActuatorType::MOTORS) {
					if (actuator_type_idx >= MAX_NUM_MOTORS) {
						PX4_ERR("Too many motors");
						_num_actuators[actuator_type] = 0;
						break;
					}

					if (_param_r_rev.get() & (1u << actuator_type_idx)) {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;

					} else {
						minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 0.f;
					}

					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_motors[actuator_type_idx];

				} else if ((ActuatorType)actuator_type == ActuatorType::SERVOS) {
					if (actuator_type_idx >= MAX_NUM_SERVOS) {
						PX4_ERR("Too many servos");
						_num_actuators[actuator_type] = 0;
						break;
					}

					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
					slew_rate[selected_matrix](actuator_idx_matrix[selected_matrix]) = _params.slew_rate_servos[actuator_type_idx];
					trims.trim[actuator_type_idx] = config.trim[selected_matrix](actuator_idx_matrix[selected_matrix]);

				} else {
					minimum[selected_matrix](actuator_idx_matrix[selected_matrix]) = -1.f;
				}

				maximum[selected_matrix](actuator_idx_matrix[selected_matrix]) = 1.f;

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		// Handle failed actuators
		if (_handled_motor_failure_bitmask) {
			actuator_idx = 0;
			memset(&actuator_idx_matrix, 0, sizeof(actuator_idx_matrix));

			for (int motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
				int selected_matrix = _control_allocation_selection_indexes[actuator_idx];

				if (_handled_motor_failure_bitmask & (1 << motors_idx)) {
					ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[selected_matrix];

					for (int i = 0; i < NUM_AXES; i++) {
						matrix(i, actuator_idx_matrix[selected_matrix]) = 0.0f;
					}
				}

				++actuator_idx_matrix[selected_matrix];
				++actuator_idx;
			}
		}

		for (int i = 0; i < _num_control_allocation; ++i) {
			_control_allocation[i]->setActuatorMin(minimum[i]);
			_control_allocation[i]->setActuatorMax(maximum[i]);
			_control_allocation[i]->setSlewRateLimit(slew_rate[i]);

			// Set all the elements of a row to 0 if that row has weak authority.
			// That ensures that the algorithm doesn't try to control axes with only marginal control authority,
			// which in turn would degrade the control of the main axes that actually should and can be controlled.

			ActuatorEffectiveness::EffectivenessMatrix &matrix = config.effectiveness_matrices[i];

			for (int n = 0; n < NUM_AXES; n++) {
				bool all_entries_small = true;

				for (int m = 0; m < config.num_actuators_matrix[i]; m++) {
					if (fabsf(matrix(n, m)) > 0.05f) {
						all_entries_small = false;
					}
				}

				if (all_entries_small) {
					matrix.row(n) = 0.f;
				}
			}

			// Assign control effectiveness matrix
			int total_num_actuators = config.num_actuators_matrix[i];
			_control_allocation[i]->setEffectivenessMatrix(config.effectiveness_matrices[i], config.trim[i],
					config.linearization_point[i], total_num_actuators, reason == EffectivenessUpdateReason::CONFIGURATION_UPDATE);
		}

		trims.timestamp = hrt_absolute_time();
		_actuator_servos_trim_pub.publish(trims);
	}
}

void
ControlAllocator::publish_control_allocator_status(int matrix_index)
{
	control_allocator_status_s control_allocator_status{};
	control_allocator_status.timestamp = hrt_absolute_time();

	// TODO: disabled motors (?)

	// Allocated control
	const matrix::Vector<float, NUM_AXES> &allocated_control = _control_allocation[matrix_index]->getAllocatedControl();

	// Unallocated control
	const matrix::Vector<float, NUM_AXES> unallocated_control = _control_allocation[matrix_index]->getControlSetpoint() -
			allocated_control;
	control_allocator_status.unallocated_torque[0] = unallocated_control(0);
	control_allocator_status.unallocated_torque[1] = unallocated_control(1);
	control_allocator_status.unallocated_torque[2] = unallocated_control(2);
	control_allocator_status.unallocated_thrust[0] = unallocated_control(3);
	control_allocator_status.unallocated_thrust[1] = unallocated_control(4);
	control_allocator_status.unallocated_thrust[2] = unallocated_control(5);

	// override control_allocator_status in customized saturation logic for certain effectiveness types
	_actuator_effectiveness->getUnallocatedControl(matrix_index, control_allocator_status);

	// Allocation success flags
	control_allocator_status.torque_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_torque[0],
			control_allocator_status.unallocated_torque[1],
			control_allocator_status.unallocated_torque[2]).norm_squared() < 1e-6f);
	control_allocator_status.thrust_setpoint_achieved = (Vector3f(control_allocator_status.unallocated_thrust[0],
			control_allocator_status.unallocated_thrust[1],
			control_allocator_status.unallocated_thrust[2]).norm_squared() < 1e-6f);

	// Actuator saturation
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_sp = _control_allocation[matrix_index]->getActuatorSetpoint();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_min = _control_allocation[matrix_index]->getActuatorMin();
	const matrix::Vector<float, NUM_ACTUATORS> &actuator_max = _control_allocation[matrix_index]->getActuatorMax();

	for (int i = 0; i < NUM_ACTUATORS; i++) {
		if (actuator_sp(i) > (actuator_max(i) - FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_UPPER;

		} else if (actuator_sp(i) < (actuator_min(i) + FLT_EPSILON)) {
			control_allocator_status.actuator_saturation[i] = control_allocator_status_s::ACTUATOR_SATURATION_LOWER;
		}
	}

	// Handled motor failures
	control_allocator_status.handled_motor_failure_mask = _handled_motor_failure_bitmask;

	// Custom allocation status and data
	control_allocator_status.custom_allocation_used = _custom_allocation_valid;

	// Set custom allocation inputs and results
	if (control_allocator_status.custom_allocation_used) {
		// Thrust inputs [fx, fy, fz]
		control_allocator_status.custom_thrust_input[0] = _thrust_sp(0);  // fx
		control_allocator_status.custom_thrust_input[1] = _thrust_sp(1);  // fy
		control_allocator_status.custom_thrust_input[2] = _thrust_sp(2);  // fz

		// Torque inputs [tau_x, tau_y, tau_z]
		control_allocator_status.custom_torque_input[0] = _torque_sp(0);  // tau_x
		control_allocator_status.custom_torque_input[1] = _torque_sp(1);  // tau_y
		control_allocator_status.custom_torque_input[2] = _torque_sp(2);  // tau_z

		// Custom allocation result vector du [6]
		for (int i = 0; i < 6; i++) {
			control_allocator_status.custom_allocation_result[i] = _custom_allocation_result(i);
		}

		// Custom trim vector [6]
		for (int i = 0; i < 6; i++) {
			control_allocator_status.custom_trim_vector[i] = _custom_trim_vec(i);
		}
	} else {
		// Clear custom allocation data when not used
		for (int i = 0; i < 3; i++) {
			control_allocator_status.custom_thrust_input[i] = 0.0f;
			control_allocator_status.custom_torque_input[i] = 0.0f;
		}
		for (int i = 0; i < 6; i++) {
			control_allocator_status.custom_allocation_result[i] = 0.0f;
			control_allocator_status.custom_trim_vector[i] = 0.0f;
		}
	}

	_control_allocator_status_pub[matrix_index].publish(control_allocator_status);
}

void
ControlAllocator::publish_actuator_controls()
{
	if (!_publish_controls) {
		return;
	}

	actuator_motors_s actuator_motors;
	actuator_motors.timestamp = hrt_absolute_time();
	actuator_motors.timestamp_sample = _timestamp_sample;

	actuator_servos_s actuator_servos;
	actuator_servos.timestamp = actuator_motors.timestamp;
	actuator_servos.timestamp_sample = _timestamp_sample;

	actuator_motors.reversible_flags = _param_r_rev.get();

	int actuator_idx = 0;
	int actuator_idx_matrix[ActuatorEffectiveness::MAX_NUM_MATRICES] {};

	uint32_t stopped_motors = _actuator_effectiveness->getStoppedMotors() | _handled_motor_failure_bitmask;

	// motors
	int motors_idx;

	for (motors_idx = 0; motors_idx < _num_actuators[0] && motors_idx < actuator_motors_s::NUM_CONTROLS; motors_idx++) {
		int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
		float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
		actuator_motors.control[motors_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;

		if (stopped_motors & (1u << motors_idx)) {
			actuator_motors.control[motors_idx] = NAN;
		}

		++actuator_idx_matrix[selected_matrix];
		++actuator_idx;
	}

	for (int i = motors_idx; i < actuator_motors_s::NUM_CONTROLS; i++) {
		actuator_motors.control[i] = NAN;
	}

	_actuator_motors_pub.publish(actuator_motors);

	// servos
	if (_num_actuators[1] > 0) {
		int servos_idx;

		for (servos_idx = 0; servos_idx < _num_actuators[1] && servos_idx < actuator_servos_s::NUM_CONTROLS; servos_idx++) {
			int selected_matrix = _control_allocation_selection_indexes[actuator_idx];
			float actuator_sp = _control_allocation[selected_matrix]->getActuatorSetpoint()(actuator_idx_matrix[selected_matrix]);
			actuator_servos.control[servos_idx] = PX4_ISFINITE(actuator_sp) ? actuator_sp : NAN;
			++actuator_idx_matrix[selected_matrix];
			++actuator_idx;
		}

		for (int i = servos_idx; i < actuator_servos_s::NUM_CONTROLS; i++) {
			actuator_servos.control[i] = NAN;
		}

		_actuator_servos_pub.publish(actuator_servos);
	}
}

void
ControlAllocator::check_for_motor_failures()
{
	failure_detector_status_s failure_detector_status;

	if ((FailureMode)_param_ca_failure_mode.get() > FailureMode::IGNORE
	    && _failure_detector_status_sub.update(&failure_detector_status)) {
		if (failure_detector_status.fd_motor) {

			if (_handled_motor_failure_bitmask != failure_detector_status.motor_failure_mask) {
				// motor failure bitmask changed
				switch ((FailureMode)_param_ca_failure_mode.get()) {
				case FailureMode::REMOVE_FIRST_FAILING_MOTOR: {
						// Count number of failed motors
						const int num_motors_failed = math::countSetBits(failure_detector_status.motor_failure_mask);

						// Only handle if it is the first failure
						if (_handled_motor_failure_bitmask == 0 && num_motors_failed == 1) {
							_handled_motor_failure_bitmask = failure_detector_status.motor_failure_mask;
							PX4_WARN("Removing motor from allocation (0x%x)", _handled_motor_failure_bitmask);

							for (int i = 0; i < _num_control_allocation; ++i) {
								_control_allocation[i]->setHadActuatorFailure(true);
							}

							update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
						}
					}
					break;

				default:
					break;
				}

			}

		} else if (_handled_motor_failure_bitmask != 0) {
			// Clear bitmask completely
			PX4_INFO("Restoring all motors");
			_handled_motor_failure_bitmask = 0;

			for (int i = 0; i < _num_control_allocation; ++i) {
				_control_allocation[i]->setHadActuatorFailure(false);
			}

			update_effectiveness_matrix_if_needed(EffectivenessUpdateReason::MOTOR_ACTIVATION_UPDATE);
		}
	}
}

int ControlAllocator::task_spawn(int argc, char *argv[])
{
	ControlAllocator *instance = new ControlAllocator();

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

int ControlAllocator::print_status()
{
	PX4_INFO("Running");

	// Print current allocation method
	switch (_allocation_method_id) {
	case AllocationMethod::NONE:
		PX4_INFO("Method: None");
		break;

	case AllocationMethod::PSEUDO_INVERSE:
		PX4_INFO("Method: Pseudo-inverse");
		break;

	case AllocationMethod::SEQUENTIAL_DESATURATION:
		PX4_INFO("Method: Sequential desaturation");
		break;

	case AllocationMethod::AUTO:
		PX4_INFO("Method: Auto");
		break;
	}

	// Print current airframe
	if (_actuator_effectiveness != nullptr) {
		PX4_INFO("Effectiveness Source: %s", _actuator_effectiveness->name());
	}

	// Print current effectiveness matrix
	for (int i = 0; i < _num_control_allocation; ++i) {
		const ActuatorEffectiveness::EffectivenessMatrix &effectiveness = _control_allocation[i]->getEffectivenessMatrix();

		if (_num_control_allocation > 1) {
			PX4_INFO("Instance: %i", i);
		}

		PX4_INFO("  Effectiveness.T =");
		effectiveness.T().print();
		PX4_INFO("  minimum =");
		_control_allocation[i]->getActuatorMin().T().print();
		PX4_INFO("  maximum =");
		_control_allocation[i]->getActuatorMax().T().print();
		PX4_INFO("  Configured actuators: %i", _control_allocation[i]->numConfiguredActuators());
	}

	if (_handled_motor_failure_bitmask) {
		PX4_INFO("Failed motors: %i (0x%x)", math::countSetBits(_handled_motor_failure_bitmask),
			 _handled_motor_failure_bitmask);
	}

	// Print perf
	perf_print_counter(_loop_perf);

	return 0;
}

int ControlAllocator::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ControlAllocator::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This implements control allocation. It takes torque and thrust setpoints
as inputs and outputs actuator setpoint messages.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("control_allocator", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void
ControlAllocator::update_utrim_status()
{
    // 尝试更新 utrim 消息
    if (_utrim_sub.update(&_current_utrim)) {
        // 有新消息，更新状态
        _utrim_available = _current_utrim.valid;
        _last_utrim_update = hrt_absolute_time();

        if (_utrim_available) {
            PX4_DEBUG("CA: utrim 更新成功，时间戳: %llu", (unsigned long long)_current_utrim.timestamp);
        }
    } else if (_utrim_sub.copy(&_current_utrim)) {
        // 没有新消息，但可以获取上一次的消息
        _utrim_available = _current_utrim.valid;

        if (_utrim_available) {
            PX4_DEBUG("CA: 使用上一次 utrim 消息，时间戳: %llu", (unsigned long long)_current_utrim.timestamp);
        }
    } else {
        // 无法获取任何 utrim 消息
        _utrim_available = false;
        PX4_DEBUG("CA: 无法获取 utrim 消息");
    }
}

void
ControlAllocator::calculate_custom_allocation()
{
    _custom_allocation_valid = false;

    // 检查是否有有效的推力设定点
    if (!PX4_ISFINITE(_thrust_sp(0)) || !PX4_ISFINITE(_thrust_sp(2))) {
        PX4_INFO("CA: 推力设定点无效");
        return;
    }

    _custom_trim_vec.setZero();

	// --- 2) read/define constants & states ---
    // 如果utrim消息可用且有效，使用utrim值；否则使用合理的默认值
    float f1, f2, f3, theta1, theta2, theta3;

    if (_utrim_available && _current_utrim.valid) {
        // 使用utrim消息中的值
        f1 = _current_utrim.polynomial_values[0];
        f2 = _current_utrim.polynomial_values[1];
        f3 = _current_utrim.polynomial_values[2];
        theta1 = _current_utrim.polynomial_values[3] * (M_PI_F/180.0f);
        theta2 = _current_utrim.polynomial_values[4] * (M_PI_F/180.0f);
        theta3 = _current_utrim.polynomial_values[5] * (M_PI_F/180.0f);

        PX4_DEBUG("CA: 使用utrim值 f1=%.2f f2=%.2f f3=%.2f θ1=%.1f° θ2=%.1f° θ3=%.1f°",
                 (double)f1, (double)f2, (double)f3,
                 (double)_current_utrim.polynomial_values[3], (double)_current_utrim.polynomial_values[4], (double)_current_utrim.polynomial_values[5]);
    } else {
        // 使用更合理的起飞默认值，而不是回退到标准分配
        f1 = 12.0f;  // 增加默认推力值以支持起飞
        f2 = 12.0f;
        f3 = 12.0f;
        theta1 = 0.0f;
        theta2 = 0.0f;
        theta3 = 0.0f;

        PX4_DEBUG("CA: utrim不可用，使用默认值 f1=%.2f f2=%.2f f3=%.2f", (double)f1, (double)f2, (double)f3);
    }

    // geometric constants
    const float L1 = 0.23f;
    const float L2 = 0.40f;
    const float L3 = 0.20f;

    // Desired task vector b (5x1) (use your measured setpoints)
    matrix::Vector<float, 5> b;
    b(0) = _thrust_sp(0);                 // fx
    b(1) = _thrust_sp(2);                 // fz
    b(2) = (-_torque_sp(0)) / L3;         // dtau_x / L3
    b(3) = (_torque_sp(1)) / L1;          // dtau_y / L1
    b(4) = (-_torque_sp(2)) / L3;         // dtau_z / L3

    // --- 3) construct geometric effectiveness matrix A (5x6) ---
    matrix::Matrix<float, 5, 6> A;
    A.setZero();

    // compute sines/cosines
    const float s1 = sinf(theta1), c1 = cosf(theta1);
    const float s2 = sinf(theta2), c2 = cosf(theta2);
    const float s3 = sinf(theta3), c3 = cosf(theta3);

    // fill A: (example following your geometric form)
    // row 0: fx coefficients (df1, df2, df3, dtheta1, dtheta2, dtheta3)
    A(0,0) =  s1; A(0,1) =  s2; A(0,2) =  s3; A(0,3) =  f1 * c1; A(0,4) =  f2 * c2; A(0,5) =  f3 * c3;

    // row 1: fz
    A(1,0) = -c1; A(1,1) = -c2; A(1,2) = -c3; A(1,3) =  f1 * s1; A(1,4) =  f2 * s2; A(1,5) =  f3 * s3;

    // row 2: d*tau_x / L3
    A(2,0) = -c1; A(2,1) =  c2; A(2,2) = 0.0f; A(2,3) =  f1 * s1; A(2,4) = -f2 * s2; A(2,5) = 0.0f;

    // row 3: d*tau_y / L1
    A(3,0) =  c1; A(3,1) =  c2; A(3,2) = -(L2 / L1) * c3; A(3,3) = -f1 * s1; A(3,4) = -f2 * s2; A(3,5) =  f3 * (L2 / L1) * s3;

    // row 4: d*tau_z / L3
    A(4,0) = -s1; A(4,1) =  s2; A(4,2) = 0.0f; A(4,3) = -f1 * c1; A(4,4) =  f2 * c2; A(4,5) = 0.0f;

    // --- 4) column weights W_diag (6 elements) ---
    // Lower weight = more freedom to move (motors usually cheaper), higher weight = penalize change (servos)
    matrix::Vector<float, 6> W_diag;
    W_diag(0) = 1.0f;   // motor1
    W_diag(1) = 1.0f;   // motor2
    W_diag(2) = 1.0f;   // motor3
    W_diag(3) = 2.0f;   // servo1 (penalize)
    W_diag(4) = 2.0f;   // servo2
    W_diag(5) = 2.0f;   // servo3

    // Optionally adapt servo weights when fi small (not needed if fi in 3-9N)
    // for (int j=3; j<6; ++j) W_diag(j) = base_servo_weight / max(f_i_normalized, eps);

    // --- 5) scale columns: A_scaled = A * W^{-1}  (i.e. divide each column j by W_diag(j)) ---
    matrix::Matrix<float, 5, 6> A_scaled = A;
    for (int j = 0; j < 6; ++j) {
        const float inv_w = 1.0f / math::max(W_diag(j), 1e-6f);
        for (int i = 0; i < 5; ++i) {
            A_scaled(i,j) *= inv_w;
        }
    }

    // --- 6) build normal matrix A_scaled * A_scaled^T (5x5) ---
    matrix::Matrix<float, 5, 5> AAt = A_scaled * A_scaled.transpose();

    // --- 7) optional pre-check: diag / approximate condition test ---
    // A simple heuristic: check diagonal magnitude spread to detect ill-conditioning
    float max_diag = 0.f, min_diag = 1e12f;
    for (int i = 0; i < 5; ++i) {
        float d = fabsf(AAt(i,i));
        max_diag = math::max(max_diag, d);
        min_diag = math::min(min_diag, d);
    }
    // If min_diag is extremely small relative to max_diag, we may need stronger regularization or fallback
    const float cond_diag_ratio = (min_diag > 0.f) ? (max_diag / min_diag) : 1e12f;

    // --- 8) regularization and geninv attempt(s) ---
    matrix::Matrix<float, 5, 5> AAt_inv;
    const float lambda_base = 1e-6f;    // base damp
    float lambda = lambda_base;

    // If heuristic indicates poor conditioning, increase lambda
    if (cond_diag_ratio > 1e6f) {
        lambda = 1e-3f; // escalate regularization
        PX4_DEBUG("CA: condition ratio %.3e, increasing lambda to %.3e", (double)cond_diag_ratio, (double)lambda);
    }

    // add lambda to diagonal (Tikhonov)
    for (int i = 0; i < 5; ++i) { AAt(i,i) += lambda; }

    bool invert_ok = matrix::geninv(AAt, AAt_inv);

    // if geninv failed, try increasing lambda a couple times
    if (!invert_ok) {
        for (int retry = 0; retry < 3 && !invert_ok; ++retry) {
            lambda *= 10.0f;
            for (int i = 0; i < 5; ++i) { AAt(i,i) += lambda; } // add extra reg
            invert_ok = matrix::geninv(AAt, AAt_inv);
            PX4_DEBUG("CA: geninv retry %d lambda=%.3e ok=%d", retry, (double)lambda, invert_ok);
        }
    }

    if (!invert_ok) {
        PX4_WARN("CA: geninv failed even after retries; allocation invalid");
        _custom_allocation_valid = false;
        return;
    }

    // --- 9) compute du_scaled = A_scaled^T * AAt_inv * b  (6x1) ---
    matrix::Matrix<float, 6, 5> At = A_scaled.transpose();
    matrix::Vector<float, 6> du_scaled = At * (AAt_inv * b);

    // --- 10) undo scaling: du = W^{-1} * du_scaled  (since we did A_scaled = A * W^{-1}) ---
    matrix::Vector<float, 6> du;
    for (int j = 0; j < 6; ++j) {
        const float inv_w = 1.0f / math::max(W_diag(j), 1e-6f);
        du(j) = inv_w * du_scaled(j);
    }

    // At this point, du holds [ df1, df2, df3, dtheta1_scaled, dtheta2_scaled, dtheta3_scaled ]
    // If your A used geometric columns for theta (i.e. the 4..6 columns were cos/sin),
    // the 4..6 entries correspond to f_i * dtheta_i (or dtheta_i depending on construction).
    // Ensure you interpret du consistently with how you built A.

    // --- 11) compute residual and quick sanity check ---
    matrix::Vector<float,5> residual = A * du - b;
    float res_norm = 0.0f;
    float b_norm = 0.0f;
    for (int i=0;i<5;i++) { res_norm += residual(i)*residual(i); b_norm += b(i)*b(i); }
    res_norm = sqrtf(res_norm);
    b_norm = sqrtf(b_norm);
    float rel_res = (b_norm > 0.0f) ? (res_norm / b_norm) : res_norm;

    if (rel_res > 0.05f) { // example threshold 5% - tune for your vehicle
        PX4_WARN("CA: relative residual %.3f > threshold, consider switching to fallback", (double)rel_res);
        // Option: fall back to stronger method (e.g., increase lambda, or use a different solver)
        // For now we accept but flag.
    }

    // --- 12) actuator limits & clipping (simple) ---
    // Map du entries to actual actuator command deltas / normalized ranges
    // Example mapping (you should replace with your actuator scaling):
    matrix::Vector<float, 6> actuator_cmd = du; // placeholder mapping

    // Clip each actuator to [min,max] (example -1..1 for servos, 0..1 for motors)
    for (int j = 0; j < 3; ++j) {
        // motors: suppose normalized 0..1
        actuator_cmd(j) = math::constrain(actuator_cmd(j), -0.5f, 0.5f); // example limits; adjust
    }
    for (int j = 3; j < 6; ++j) {
        actuator_cmd(j) = math::constrain(actuator_cmd(j), -1.0f, 1.0f);
    }

    // --- 13) optional: sequential desaturation (advanced) ---
    // If clipping happened and you want to preserve primary objectives, you can implement
    // sequential desaturation: iteratively fix saturated actuators and re-solve for remaining DOF.
    // Place hook here to call a desaturation routine if required.

    // --- 14) finalize results ---
    _custom_allocation_result = actuator_cmd; // store the result in your class member
    _custom_allocation_valid = true;

    PX4_DEBUG("CA: allocation OK rel_res=%.3f", (double)rel_res);

    // Update trim vector for apply_custom_allocation
    _custom_trim_vec.setZero();
    if (_utrim_available && _current_utrim.valid) {
        // 前三个量（f1,f2,f3 推力值）转换为[0,1]范围（电机相关）
        const float motor_coeff = 34.024f;
        const float motor_offset = 767.4f;

        for (int i = 0; i < 3 && i < NUM_ACTUATORS; ++i) {
            float thrust_value = _current_utrim.polynomial_values[i];  // f1,f2,f3 推力值
            // 从推力计算电机信号 [0-100]
            float motor_signal = (thrust_value/CONSTANTS_ONE_G*1000.0f + motor_offset) / motor_coeff;
            // 限制在 [0-100] 范围内
            motor_signal = math::constrain(motor_signal, 0.0f, 100.0f);
            // 归一化到 [0-1] 范围
            _custom_trim_vec(i) = motor_signal / 100.0f;
        }

        // 后三个量（theta1,theta2,theta3 角度）转换为[-1,1]范围（舵机相关）
        for (int i = 3; i < 6 && i < NUM_ACTUATORS; ++i) {
            float angle_deg = _current_utrim.polynomial_values[i];  // theta1,theta2,theta3 角度值（度）
            // 角度转换为[-1,1]范围: angle / 45°
            _custom_trim_vec(i) = math::constrain(angle_deg / 45.0f, -1.0f, 1.0f);
        }
    } else {
        // 当utrim不可用时，使用默认的trim值
        // 对于电机：使用默认推力值计算trim
        const float motor_coeff = 34.024f;
        const float motor_offset = 767.4f;

        for (int i = 0; i < 3 && i < NUM_ACTUATORS; ++i) {
            float default_thrust = (i == 0 ? f1 : (i == 1 ? f2 : f3));
            float motor_signal = (default_thrust/CONSTANTS_ONE_G*1000.0f + motor_offset) / motor_coeff;
            motor_signal = math::constrain(motor_signal, 0.0f, 100.0f);
            _custom_trim_vec(i) = motor_signal / 100.0f;
        }

        // 对于舵机：使用默认角度（0度）
        for (int i = 3; i < 6 && i < NUM_ACTUATORS; ++i) {
            _custom_trim_vec(i) = 0.0f;  // 0度对应0.0f
        }
    }
}

bool
ControlAllocator::apply_custom_allocation(matrix::Vector<float, NUM_ACTUATORS> &actuator_sp)
{
	if (!_custom_allocation_valid) {
		return false;
	}

    // actuator_sp = du + trim
    for (int i = 0; i < NUM_ACTUATORS && i < 6; ++i) {
        actuator_sp(i) = _custom_allocation_result(i) + _custom_trim_vec(i);
    }
    for (int i = 6; i < NUM_ACTUATORS; ++i) {
        actuator_sp(i) = 0.f;
    }

    // 10Hz 限频打印: 分别输出 du、utrim（trim）以及 fx,fz,tau_x,y,z
    static hrt_abstime last_log = 0;
    const hrt_abstime now = hrt_absolute_time();
    if (now - last_log >= 100_ms) {
        // du（伪逆结果）- 电机推力变化量 df1, df2, df3
        PX4_INFO("CA: du_motors(df1,df2,df3)=%.3f, %.3f, %.3f",
                 (double)_custom_allocation_result(0), (double)_custom_allocation_result(1), (double)_custom_allocation_result(2));

        // du（伪逆结果）- 舵机角度变化量 dtheta1, dtheta2, dtheta3
        PX4_INFO("CA: du_servos(dθ1,dθ2,dθ3)=%.3f, %.3f, %.3f",
                 (double)_custom_allocation_result(3), (double)_custom_allocation_result(4), (double)_custom_allocation_result(5));

        // utrim（trim 偏置）- 电机基准值
        PX4_INFO("CA: trim_motors(f1,f2,f3)=%.3f, %.3f, %.3f",
                 (double)_custom_trim_vec(0), (double)_custom_trim_vec(1), (double)_custom_trim_vec(2));

        // utrim（trim 偏置）- 舵机基准值
        PX4_INFO("CA: trim_servos(θ1,θ2,θ3)=%.3f, %.3f, %.3f",
                 (double)_custom_trim_vec(3), (double)_custom_trim_vec(4), (double)_custom_trim_vec(5));

        // 最终执行器指令 = du + trim
        PX4_INFO("CA: final_motors=%.3f, %.3f, %.3f",
                 (double)actuator_sp(0), (double)actuator_sp(1), (double)actuator_sp(2));
        PX4_INFO("CA: final_servos=%.3f, %.3f, %.3f",
                 (double)actuator_sp(3), (double)actuator_sp(4), (double)actuator_sp(5));

        // 同步打印 fx, fz, tau_x, tau_y, tau_z（10Hz）
        PX4_INFO("CA: 输入 fx=%.3f fz=%.3f tau_x=%.3f tau_y=%.3f tau_z=%.3f",
                 (double)_thrust_sp(0), (double)_thrust_sp(2),
                 (double)_torque_sp(0), (double)_torque_sp(1), (double)_torque_sp(2));

        last_log = now;
    }

    return true;
}

/**
 * Control Allocator app start / stop handling function
 */
extern "C" __EXPORT int control_allocator_main(int argc, char *argv[]);

int control_allocator_main(int argc, char *argv[])
{
	return ControlAllocator::main(argc, argv);
}
