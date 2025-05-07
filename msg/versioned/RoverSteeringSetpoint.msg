# Rover Steering setpoint

uint32 MESSAGE_VERSION = 0

uint64  timestamp 	             # Time since system start [us]
float32 normalized_steering_setpoint # Ackermann: Normalized steering angle, Differential/Mecanum: Normalized speed difference between the left and right wheels [@range -1 (Left), 1 (Right)] [@frame Body]
