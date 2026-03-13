# 北航机器人队十楼实验室导航测试脚本
cd ~/nav2_ws
source install/setup.bash

# --- 参数与模式 ---
# 用法：
#  1) 启动全部（默认）： ./floor10_nav.sh both [behavior_name]
#  2) 仅导航：       ./floor10_nav.sh nav
#  3) 仅行为树：     ./floor10_nav.sh behavior [decision_simple|pb2025_sentry_behavior]

show_usage() {
        cat <<EOF
Usage: $(basename "$0") [options] [mode] [behavior_name]

Options:
    --terminals, -t   open behavior and navigation in separate gnome-terminal windows (then exit)
    --help            show this help

Modes:
    nav               only start navigation
    behavior          only start behavior tree
    both (default)    start navigation then behavior (serial)

Examples:
    $(basename "$0") both decision_simple
    $(basename "$0") --terminals decision_simple
EOF
}

OPEN_TERMINALS=1    # 默认在两个终端中并行打开
while [ "${1-}" != "" ] && [[ "${1-}" == --* || "${1-}" == -?* ]]; do
        case "$1" in
        --terminals|-t)
            OPEN_TERMINALS=1; shift ;;
        --no-terminals)
            OPEN_TERMINALS=0; shift ;;
                --help|-h)
                        show_usage; exit 0 ;;
                *)
                        echo "Unknown option: $1" >&2; show_usage; exit 2 ;;
        esac
done

MODE=${1:-both}                 # nav | behavior | both
BEHAVIOR_TREE=${2:-decision_simple}   # 行为树名称，默认 decision_simple

# 如果用户直接传入行为树名称作为第一个参数（例如
# ./floor10_nav.sh pb2025_sentry_behavior），把它当作 behavior_name 并
# 将 MODE 设为 both，以兼容常见用法。
case "$MODE" in
    decision_simple|pb2025_sentry_behavior)
        BEHAVIOR_TREE=$MODE
        MODE=both
        ;;
    *) ;;
esac

start_nav() {
    echo "启动导航: world=floor10 slam=False"
    ros2 launch pb2025_nav_bringup rm_navigation_reality_launch_nm.py world:=floor10 slam:=False
}

start_behavior() {
    case "$BEHAVIOR_TREE" in
        decision_simple)
            echo "启动 decision_simple 行为树..."
            ros2 launch decision_simple decision_simple.launch.py
            ;;
        pb2025_sentry_behavior)
            echo "启动 pb2025_sentry_behavior 行为树..."
            ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
            ;;
        *)
            echo "未知行为树: $BEHAVIOR_TREE" >&2
            exit 1
            ;;
    esac
}

# 子进程管理：并行启动时记录 PID 并在退出时清理
PIDS=()
cleanup() {
    echo "[floor10_nav.sh] cleanup: killing child processes..."
    for pid in "${PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "[floor10_nav.sh] killing pid $pid"
            kill "$pid" 2>/dev/null || true
        fi
    done
    # 等待子进程退出
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

case "$MODE" in
    nav)
        start_nav
        ;;
    behavior)
        start_behavior
        ;;
    both)
        # 并行启动导航与行为树（默认并行）。若指定 --terminals 则在两个终端窗口打开。
        if [ "$OPEN_TERMINALS" -eq 1 ]; then
            # Open separate gnome-terminal windows for behavior and navigation
            if command -v gnome-terminal >/dev/null 2>&1; then
                WORKDIR="$HOME/nav2_ws"
                # Use absolute script path to avoid $0/relative-path issues in child terminals
                SCRIPT_PATH="$WORKDIR/scripts/floor10_nav.sh"
                BEHAVIOR_CMD="cd $WORKDIR && source install/setup.bash && \"$SCRIPT_PATH\" behavior $BEHAVIOR_TREE; exec bash"
                NAV_CMD="cd $WORKDIR && source install/setup.bash && \"$SCRIPT_PATH\" nav; exec bash"
                gnome-terminal --window --title="behavior" -- bash -lc "$BEHAVIOR_CMD" &
                gnome-terminal --window --title="navigation" -- bash -lc "$NAV_CMD" &
                exit 0
            else
                echo "gnome-terminal not found, falling back to parallel start" >&2
            fi
        fi

        # 并行方式：后台启动并记录 PID，等待两个进程结束
        start_nav &
        pid_nav=$!
        PIDS+=("$pid_nav")

        start_behavior &
        pid_behavior=$!
        PIDS+=("$pid_behavior")

        echo "[floor10_nav.sh] started nav (pid=$pid_nav) and behavior (pid=$pid_behavior), waiting..."

        # 等待两个子进程
        wait "$pid_nav" "$pid_behavior"
        ;;
    *)
        echo "Unknown mode: $MODE" >&2
        echo "Usage: $0 [nav|behavior|both] [behavior_name]" >&2
        exit 2
        ;;
esac