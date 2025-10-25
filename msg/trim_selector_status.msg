uint64 timestamp        # time since system start (microseconds)

float32 gust_raw    # Raw gust value: airspeed - horizontal_groundspeed (m/s)
float32 gust_filt   # 0.5Hz low-pass filtered gust (m/s)  
float32 k           # Anti-wind blending coefficient [0,1]