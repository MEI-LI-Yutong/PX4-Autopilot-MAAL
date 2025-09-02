uint64 timestamp                        # time since system start (microseconds)

# Utrim data received by control allocator
bool utrim_valid                        # Whether utrim data is valid
float32 horizontal_velocity             # Horizontal velocity from utrim [m/s]
float32[6] polynomial_values            # 6 polynomial calculation results (f1,f2,f3[N], θ1,θ2,θ3[deg])
float32[6] normalized_values            # Normalized values (f1,f2,f3[0-1], θ1,θ2,θ3[-1,1])

# Control allocation du (actuator setpoint changes)
float32[16] actuator_du                 # Change in actuator setpoint (current - previous)
float32[16] actuator_sp_current         # Current actuator setpoint
float32[16] actuator_sp_previous        # Previous actuator setpoint

# Control setpoints received
float32[3] torque_sp                    # Torque setpoint received [Nm]
float32[3] thrust_sp                    # Thrust setpoint received [N]
