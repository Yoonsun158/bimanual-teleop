# Python 环境

- 本项目的 Python 运行、安装和测试统一使用 Conda 环境 `bimanual-teleop`。
- 首次创建：在项目根目录运行 `PIP_USER=false conda env create -f environment.yml`。
- 运行前执行 `conda activate bimanual-teleop`；非交互命令可用 `conda run -n bimanual-teleop ...`。
- Python 依赖只安装在此环境中，禁止使用系统 Python、`pip --user` 或 `sudo pip` 安装本项目依赖。
