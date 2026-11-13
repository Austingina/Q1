#!/usr/bin/env bash
set -eo pipefail

PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
# 固件里 setup.bash 保留了厂商编译机的绝对路径；显式指向真机安装前缀。
export COLCON_CURRENT_PREFIX=/robot/software/interface
source /robot/software/interface/local_setup.bash
export PYTHONPATH="$PROJECT/.python${PYTHONPATH:+:$PYTHONPATH}"
set -u
exec python3 -m apps.dance_web.dry_run_check "$@"
