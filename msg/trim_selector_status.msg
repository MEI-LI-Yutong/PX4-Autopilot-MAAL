uint64 timestamp

# gust 原始值（m/s），取 true airspeed - horizontal groundspeed，负数截为0
float32 gust_raw

# 1Hz 一阶低通后的 gust（m/s）
float32 gust_filt

# 抗风滑移系数 k ∈ [0,1]
float32 k