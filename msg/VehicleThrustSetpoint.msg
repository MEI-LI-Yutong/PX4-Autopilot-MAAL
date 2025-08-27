
uint64 timestamp        # time since system start (microseconds)
uint64 timestamp_sample # timestamp of the data sample on which this message is based (microseconds)

float32[3] xyz          # thrust setpoint along X, Y, Z body axis [-1, 1]

float32 tilt_extra_angle	# Extra tilt angle for forward thrust (rad)
bool tilt_extra_angle_valid	# true: apply angle; false: disable (start ramp-to-zero)

# TOPICS vehicle_thrust_setpoint
# TOPICS vehicle_thrust_setpoint_virtual_fw vehicle_thrust_setpoint_virtual_mc
