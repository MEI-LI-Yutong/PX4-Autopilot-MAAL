uint64 timestamp                # 时间戳 [microseconds]
float32 horizontal_velocity    # 水平速度 [m/s]
float32[6] polynomial_values   # 6个多项式计算结果 (原始值: f1,f2,f3[N], θ1,θ2,θ3[deg])
float32[6] normalized_values   # 归一化后的值 (f1,f2,f3[0-1], θ1,θ2,θ3[-1,1])
bool valid                     # 数据是否有效
