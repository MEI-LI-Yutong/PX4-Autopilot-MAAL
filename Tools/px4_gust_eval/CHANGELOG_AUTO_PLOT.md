# 自动绘图和 wandb 集成功能 - 修改日志

## 概述

本次更新为 px4_gust_eval 框架添加了以下主要功能：

1. **自动绘图**: 仿真完成后自动生成性能图表
2. **wandb 集成**: 支持将结果和图表上传到 Weights & Biases
3. **智能日志查找**: `plot_gust_levels.py` 可以自动查找最新的日志目录
4. **环境变量配置**: 通过环境变量灵活控制功能开关

## 修改的文件

### 1. `Tools/px4_gust_eval/plot_gust_levels.py`

**修改内容**：
- 将 `results_dir` 参数改为可选（`nargs="?", default=None`）
- 添加 `find_latest_log_dir()` 函数，自动查找 logs 目录下最新的 run_* 文件夹
- 修改 `main()` 函数，当未指定 results_dir 时自动查找最新日志

**使用示例**：
```bash
# 旧用法（仍然支持）
uv run plot_gust_levels.py tasks/test.json logs/run_20250929_131736

# 新用法（自动查找）
uv run plot_gust_levels.py tasks/test.json
```

### 2. `docker/run_gust_eval.sh`

**修改内容**：
- 添加环境变量：
  - `ENABLE_PLOT` (默认: 1) - 是否启用自动绘图
  - `WANDB_ENABLE` (默认: 0) - 是否启用 wandb 日志
  - `WANDB_PROJECT` (默认: px4_gust_eval) - wandb 项目名
  - `WANDB_ENTITY` (默认: 空) - wandb 实体/团队名
  - `WANDB_RUN_NAME` (默认: 空) - wandb 运行名称
- 在仿真完成后添加自动绘图步骤
- 根据 `WANDB_ENABLE` 动态构建绘图命令参数

**工作流程**：
```
运行仿真 → 收集数据 → [自动生成图表] → [上传到 wandb] → 完成
```

### 3. `Dockerfile`

**修改内容**：
- 添加新的环境变量默认值：
  - `ENABLE_PLOT=1`
  - `WANDB_ENABLE=0`
  - `WANDB_PROJECT=px4_gust_eval`
  - `WANDB_ENTITY=""`
  - `WANDB_RUN_NAME=""`

### 4. `Tools/px4_gust_eval/run_gust_eval_container.sh`

**修改内容**：
- 添加环境变量传递
- 添加 wandb 配置状态输出
- 将所有新环境变量传递给 Docker 容器

**使用示例**：
```bash
# 基本使用
./Tools/px4_gust_eval/run_gust_eval_container.sh

# 带 wandb
WANDB_ENABLE=1 WANDB_ENTITY=MAALab \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### 5. 新增文档

#### `Tools/px4_gust_eval/USAGE_EXAMPLES_CN.md`
完整的使用示例文档，包括：
- Docker 使用示例
- 本地环境使用示例
- 环境变量参考
- 常见工作流
- 故障排除

#### `Tools/px4_gust_eval/DOCKER_TROUBLESHOOTING_CN.md` (更新)
添加了：
- 自动绘图配置说明
- wandb 集成使用方法
- 新增环境变量说明

#### `Tools/px4_gust_eval/README_CN.md` (更新)
添加了：
- Docker 快速开始指南
- 自动绘图步骤说明
- 完整的使用流程

## 功能特性

### 1. 自动绘图

默认情况下，仿真完成后会自动调用 `plot_gust_levels.py` 生成性能图表。

**启用/禁用**：
```bash
# 启用（默认）
ENABLE_PLOT=1 ./Tools/px4_gust_eval/run_gust_eval_container.sh

# 禁用
ENABLE_PLOT=0 ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### 2. wandb 集成

支持将测试结果和图表自动上传到 Weights & Biases。

**配置方法**：
```bash
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_PROJECT=px4_gust_eval \
WANDB_RUN_NAME=my_experiment \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

### 3. 智能日志查找

`plot_gust_levels.py` 现在可以自动查找最新的日志目录，无需手动指定。

**查找规则**：
- 在 `logs/` 目录下递归搜索所有 `run_*` 文件夹
- 按修改时间排序，选择最新的一个
- 如果找不到，会提示用户手动指定

## 向后兼容性

所有修改都保持向后兼容：

1. **plot_gust_levels.py**: 仍然支持手动指定 results_dir
2. **环境变量**: 都有合理的默认值
3. **现有脚本**: 无需修改即可继续使用

## 使用场景

### 场景 1: 开发调试

快速迭代，不需要上传结果：

```bash
# 运行测试，自动生成图表
./Tools/px4_gust_eval/run_gust_eval_container.sh tasks/quick_test.json
```

### 场景 2: 正式实验

记录完整的实验数据到 wandb：

```bash
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_RUN_NAME=experiment_$(date +%Y%m%d_%H%M%S) \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh \
  tasks/beaufort_levels_tests.json
```

### 场景 3: 批量测试

运行多个配置并比较：

```bash
#!/bin/bash
for config in tasks/*.json; do
  WANDB_ENABLE=1 \
  WANDB_ENTITY=MAALab \
  WANDB_RUN_NAME=$(basename $config .json) \
    ./Tools/px4_gust_eval/run_gust_eval_container.sh "$config"
done
```

### 场景 4: CI/CD 流水线

在持续集成中自动运行测试：

```bash
# 无头模式，自动绘图，上传到 wandb
HEADLESS=1 \
ENABLE_PLOT=1 \
WANDB_ENABLE=1 \
WANDB_ENTITY=MAALab \
WANDB_RUN_NAME=ci_${CI_COMMIT_SHA} \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh
```

## 环境变量完整列表

| 变量名 | 默认值 | 说明 |
|--------|--------|------|
| **仿真相关** | | |
| `TASKS_JSON` | `tasks/dryden_boundary_layer_z_levels.json` | 任务配置文件 |
| `BUILD_TARGET` | `px4_sitl_default` | PX4 构建目标 |
| `HEADLESS` | `1` | 无头模式 |
| `PX4_GZ_SIM_RENDER_ENGINE` | `ogre` | 渲染引擎 |
| `LIBGL_ALWAYS_SOFTWARE` | `1` | 软件渲染 |
| **后处理相关** | | |
| `ENABLE_PLOT` | `1` | 自动生成图表 |
| **wandb 相关** | | |
| `WANDB_ENABLE` | `0` | 启用 wandb |
| `WANDB_PROJECT` | `px4_gust_eval` | wandb 项目 |
| `WANDB_ENTITY` | ` ` | wandb 实体 |
| `WANDB_RUN_NAME` | ` ` | wandb 运行名 |

## 测试验证

### 测试步骤

1. **基本功能测试**：
```bash
./Tools/px4_gust_eval/run_gust_eval_container.sh tasks/quick_test.json
```
预期：仿真完成后自动生成图表

2. **wandb 集成测试**：
```bash
WANDB_ENABLE=1 WANDB_ENTITY=MAALab \
  ./Tools/px4_gust_eval/run_gust_eval_container.sh tasks/quick_test.json
```
预期：图表上传到 wandb

3. **禁用绘图测试**：
```bash
ENABLE_PLOT=0 ./Tools/px4_gust_eval/run_gust_eval_container.sh
```
预期：仿真完成但不生成图表

4. **手动绘图测试**：
```bash
cd Tools/px4_gust_eval
uv run plot_gust_levels.py tasks/quick_test.json
```
预期：自动找到最新日志并生成图表

## 后续改进建议

1. **多格式输出**: 支持 PDF、SVG 等多种图表格式
2. **实时上传**: 在仿真过程中实时上传数据到 wandb
3. **自动报告**: 自动生成 Markdown 格式的测试报告
4. **邮件通知**: 测试完成后发送邮件通知
5. **结果对比**: 自动对比不同运行的结果

## 相关文档

- [USAGE_EXAMPLES_CN.md](USAGE_EXAMPLES_CN.md) - 详细使用示例
- [DOCKER_TROUBLESHOOTING_CN.md](DOCKER_TROUBLESHOOTING_CN.md) - Docker 故障排除
- [README_CN.md](README_CN.md) - 主要使用说明

## 更新日期

2025-11-23

## 贡献者

- 基于用户需求开发
- 集成了 wandb、自动绘图等功能


