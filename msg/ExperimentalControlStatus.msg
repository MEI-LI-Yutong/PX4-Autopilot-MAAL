# Debug/status output for experimental_control module

uint64 timestamp

uint32 inference_time           # [us] time spent inside controller backend (optional)
uint32 controller_time          # [us] total loop time including I/O and backend

float32[15] observation         # controller inputs (pos err, attitude matrix, linear vel, angular vel)
float32[6] network_output       # controller raw outputs before actuator rescaling
