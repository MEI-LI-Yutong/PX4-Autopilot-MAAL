# Rover Velocity Setpoint

uint64  timestamp # Time since system start [us]
float32 speed     # Speed setpoint [m/s] [@range -inf (Backwards), inf (Forwards)]
float32 bearing   # Bearing setpoint [rad] [@range -pi,pi] [@frame NED] [invalid: NAN, speed is defined in body x direction]
float32 yaw 	  # Mecanum only: Yaw setpoint [rad] [@range -pi, pi] [@frame NED] [@invalid NAN, Defaults to vehicle yaw]
