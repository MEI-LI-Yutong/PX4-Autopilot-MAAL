# PX4 风阵评估框架 - 使用示例

本文档提供详细的使用示例和命令参考。

## Docker 使用示例

### 基本使用

```bash
# 1. 最简单的用法（使用默认配置）
./Tools/px4_gust_eval/run_gust_eval_container.sh

# 2. 指定任务配置文件
./Tools/px4_gust_eval/run_gust_eval_container.sh \
  Tools/px4_gust_eval/tasks/beaufort_levels_tests.json

# 3. 使用相对路径（从 Tools/px4_gust_eval 开始）
./Tools/px4_gust_eval/run_gust_eval_container.sh \
  tasks/quick_test.json
```

### 自动绘图配置

```bash
# 启用自动绘图（默认）
ENABLE_PLOT=1 ./Tools/px4_gust_eval/run_gust_eval_container.sh

# 禁用自动绘图
ENABLE_PLOT=0 ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### wandb 集成

```bash
# 基本 wandb 配置
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh

# 完整 wandb 配置
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_PROJECT=px4_gust_eval \
WANDB_RUN_NAME=dryden_boundary_layer_test \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh \
  tasks/dryden_boundary_layer_z_levels.json

# 自定义项目名称
WANDB_ENABLE=1 \
WANDB_ENTITY=YourTeam \
WANDB_PROJECT=my_custom_project \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### 渲染引擎配置

```bash
# 使用 OGRE 1.x（推荐，兼容性好）
PX4_GZ_SIM_RENDER_ENGINE=ogre \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh

# 使用 OGRE 2.x（性能更好，但可能不兼容某些环境）
PX4_GZ_SIM_RENDER_ENGINE=ogre2 \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### 组合配置示例

```bash
# 完整的生产环境配置
HEADLESS=1 \
ENABLE_PLOT=1 \
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_PROJECT=px4_gust_eval \
WANDB_RUN_NAME=production_run_$(date +%Y%m%d_%H%M%S) \
PX4_GZ_SIM_RENDER_ENGINE=ogre \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh \
  tasks/beaufort_levels_tests.json
```

### 直接使用 docker run

如果需要更多控制，可以直接使用 docker run：

```bash
# 基本配置
sudo docker run --rm -it \
  --privileged \
  --network host \
  --ipc=host \
  -v $(pwd):/PX4-Autopilot:rw \
  -v /dev:/dev \
  -e HEADLESS=1 \
  -e ENABLE_PLOT=1 \
  -e TASKS_JSON=tasks/quick_test.json \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  raiots/maal_px4_simulation:latest

# 带 wandb 配置
sudo docker run --rm -it \
  --privileged \
  --network host \
  --ipc=host \
  -v $(pwd):/PX4-Autopilot:rw \
  -v /dev:/dev \
  -e HEADLESS=1 \
  -e ENABLE_PLOT=1 \
  -e WANDB_ENABLE=1 \
  -e WANDB_ENTITY=MAALab \
  -e WANDB_PROJECT=px4_gust_eval \
  -e TASKS_JSON=tasks/beaufort_levels_tests.json \
  raiots/maal_px4_simulation:latest

# 调试模式（进入容器 shell）
sudo docker run --rm -it \
  --privileged \
  --network host \
  -v $(pwd):/PX4-Autopilot:rw \
  -e HEADLESS=1 \
  raiots/maal_px4_simulation:latest \
  /bin/bash
```

## 本地环境使用示例

### 运行测试

```bash
cd Tools/px4_gust_eval

# 1. 快速测试（单个测试用例）
uv run main.py tasks/quick_test.json --verbose

# 2. 基础验证测试
uv run main.py tasks/basic_validation_tests.json --verbose

# 3. 完整的风阵等级测试
uv run main.py tasks/beaufort_levels_tests.json --verbose

# 4. Dryden 边界层测试
uv run main.py tasks/dryden_boundary_layer_z_levels.json --verbose

# 5. 恶劣天气测试
uv run main.py tasks/severe_weather_tests.json --verbose

# 6. 稳定性测试
uv run main.py tasks/stability_tests.json --verbose
```

### 生成图表

```bash
cd Tools/px4_gust_eval

# 1. 自动查找最新日志（推荐）
uv run plot_gust_levels.py tasks/beaufort_levels_tests.json

# 2. 指定日志目录
uv run plot_gust_levels.py \
  tasks/beaufort_levels_tests.json \
  logs/run_20250929_131736

# 3. 带 wandb 上传
uv run plot_gust_levels.py \
  tasks/beaufort_levels_tests.json \
  --wandb \
  --wandb-entity MAALab \
  --wandb-project px4_gust_eval

# 4. 完整配置
uv run plot_gust_levels.py \
  tasks/beaufort_levels_tests.json \
  logs/run_20250929_131736 \
  --dpi 600 \
  --wandb \
  --wandb-entity MAALab \
  --wandb-project px4_gust_eval \
  --wandb-run-name my_experiment
```

## 环境变量参考

### Docker 环境变量

| 变量名 | 默认值 | 说明 | 示例 |
|--------|--------|------|------|
| `TASKS_JSON` | `tasks/dryden_boundary_layer_z_levels.json` | 任务配置文件路径 | `tasks/quick_test.json` |
| `HEADLESS` | `1` | 无头模式（1=启用，0=禁用） | `1` |
| `BUILD_TARGET` | `px4_sitl_default` | PX4 构建目标 | `px4_sitl_default` |
| `ENABLE_PLOT` | `1` | 自动生成图表 | `1` |
| `WANDB_ENABLE` | `0` | 启用 wandb 日志 | `1` |
| `WANDB_ENTITY` | ` ` | wandb 实体/团队 | `MAALab` |
| `WANDB_PROJECT` | `px4_gust_eval` | wandb 项目名称 | `px4_gust_eval` |
| `WANDB_RUN_NAME` | ` ` | wandb 运行名称 | `my_experiment` |
| `PX4_GZ_SIM_RENDER_ENGINE` | `ogre` | Gazebo 渲染引擎 | `ogre` / `ogre2` |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | 强制软件渲染 | `1` |

## 常见工作流

### 工作流 1: 快速验证

测试是否一切正常：

```bash
# Docker 方式
./Tools/px4_gust_eval/run_gust_eval_container.sh tasks/quick_test.json

# 本地方式
cd Tools/px4_gust_eval
uv run main.py tasks/quick_test.json --verbose
```

### 工作流 2: 完整测试 + wandb 日志

运行完整测试并上传结果到 wandb：

```bash
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_PROJECT=px4_gust_eval \
WANDB_RUN_NAME=beaufort_levels_$(date +%Y%m%d_%H%M%S) \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh \
  tasks/beaufort_levels_tests.json
```

### 工作流 3: 批量测试不同配置

测试多个配置并比较结果：

```bash
#!/bin/bash
# batch_test.sh

CONFIGS=(
  "tasks/beaufort_levels_tests.json"
  "tasks/dryden_boundary_layer_z_levels.json"
  "tasks/stability_tests.json"
)

for config in "${CONFIGS[@]}"; do
  echo "Running ${config}..."
  WANDB_ENABLE=1 \
  WANDB_ENTITY=MAALab \
  WANDB_RUN_NAME=$(basename ${config} .json)_$(date +%Y%m%d_%H%M%S) \
    ./Tools/px4_gust_eval/run_gust_eval_container.sh "${config}"
done
```

### 工作流 4: 重新生成图表

如果需要重新生成图表（例如调整参数）：

```bash
cd Tools/px4_gust_eval

# 重新生成最新一次运行的图表
uv run plot_gust_levels.py tasks/beaufort_levels_tests.json

# 批量重新生成多个日志的图表
for log_dir in logs/run_*; do
  echo "Processing ${log_dir}..."
  uv run plot_gust_levels.py tasks/beaufort_levels_tests.json "${log_dir}"
done
```

### 工作流 5: 调试失败的测试

当测试失败时，进入容器调试：

```bash
# 启动调试容器
sudo docker run --rm -it \
  --privileged \
  --network host \
  -v $(pwd):/PX4-Autopilot:rw \
  -e HEADLESS=1 \
  -e PX4_GZ_SIM_RENDER_ENGINE=ogre \
  raiots/maal_px4_simulation:latest \
  /bin/bash

# 在容器内手动运行
cd /PX4-Autopilot
export HEADLESS=1
export GZ_SIM_HEADLESS=1
export QT_QPA_PLATFORM=offscreen
export PX4_GZ_SIM_RENDER_ENGINE=ogre

# 手动启动仿真
make px4_sitl gz_tiltrotor_windy

# 或运行测试脚本
bash docker/run_gust_eval.sh
```

## 故障排除

详细的故障排除指南请参考：
- [DOCKER_TROUBLESHOOTING_CN.md](DOCKER_TROUBLESHOOTING_CN.md) - Docker 相关问题
- [README_CN.md](README_CN.md) - 常规使用说明

### 常见问题快速解决

**问题 1: 飞控无法 arm**
```bash
# 解决方案：使用 OGRE 1.x 渲染引擎
PX4_GZ_SIM_RENDER_ENGINE=ogre ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

**问题 2: wandb 上传失败**
```bash
# 确认 wandb 已登录（在容器外执行）
wandb login

# 或在容器内设置 API key
docker run ... -e WANDB_API_KEY=your_api_key ...
```

**问题 3: 图表生成失败**
```bash
# 禁用自动生成，手动运行调试
ENABLE_PLOT=0 ./Tools/px4_gust_eval/run_gust_eval_container.sh

# 然后手动运行绘图脚本
cd Tools/px4_gust_eval
uv run plot_gust_levels.py tasks/beaufort_levels_tests.json --verbose
```

**问题 4: 找不到日志文件**
```bash
# 检查日志目录结构
ls -lR Tools/px4_gust_eval/logs/

# 手动指定日志目录
uv run plot_gust_levels.py \
  tasks/beaufort_levels_tests.json \
  logs/run_20250929_131736
```

## 更多信息

- 项目主页：[PX4-Autopilot-MAAL](https://github.com/MAALab/PX4-Autopilot-MAAL)
- PX4 文档：https://docs.px4.io/
- Gazebo 文档：https://gazebosim.org/docs/
- wandb 文档：https://docs.wandb.ai/


