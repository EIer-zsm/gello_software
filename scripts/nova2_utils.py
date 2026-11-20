#!/usr/bin/env python3
"""
Nova 2 机械臂工具箱
====================
所有与 Nova 2 通信的基础操作都在这一个文件里。
纯 socket 实现，不依赖任何 DOBOT SDK 或第三方库。

用法：
    python scripts/nova2_utils.py get_angle          # 读当前关节角（度）
    python scripts/nova2_utils.py get_angle_rad      # 读当前关节角（弧度）
    python scripts/nova2_utils.py check_tcp          # 检测是否处于 TCP 模式
    python scripts/nova2_utils.py tcp_mode           # 切换到 TCP 模式（需先下使能）
    python scripts/nova2_utils.py enable             # 上使能
    python scripts/nova2_utils.py disable            # 下使能
    python scripts/nova2_utils.py clear_error        # 清除报警
    python scripts/nova2_utils.py reset              # 复位
    python scripts/nova2_utils.py status             # 查看机械臂状态
    python scripts/nova2_utils.py home               # 置位到 0 0 90 0 -90 0 度 + 夹爪张开
    python scripts/nova2_utils.py gripper open       # 夹爪张开
    python scripts/nova2_utils.py gripper close      # 夹爪闭合
    python scripts/nova2_utils.py tool_do 1 1        # 直接操作末端 DO:DO_1 置 1
    python scripts/nova2_utils.py get_tool_do 1      # 读末端 DO_1 状态
    python scripts/nova2_utils.py ms42dc open        # MS42DC 步进夹爪：张开（串口）
    python scripts/nova2_utils.py ms42dc close       # MS42DC 步进夹爪：闭合（串口）
"""

import socket
import sys
import math
import time
import argparse

# ============================================================
# 配置：改 IP 只需改这一行
# ============================================================
NOVA2_IP = "192.168.37.3"
DASHBOARD_PORT = 29999   # 仪表盘端口（配置、读角度、使能）
MOVE_PORT = 30003        # 运动指令端口（ServoJ 等）
FEEDBACK_PORT = 30004    # 状态反馈端口


# ============================================================
# 底层通信
# ============================================================
def _send_dashboard(cmd_str: str, timeout: float = 3.0) -> str:
    """
    向 Nova 2 的 29999 仪表盘端口发送指令并接收回复。

    Args:
        cmd_str: 要发送的指令字符串，如 "GetAngle()"
        timeout: 超时秒数
    Returns:
        Nova 2 返回的字符串
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(timeout)
    try:
        s.connect((NOVA2_IP, DASHBOARD_PORT))
        s.send(cmd_str.encode("utf-8"))
        result = s.recv(1024).decode("utf-8").strip()
        return result
    finally:
        s.close()


def _parse_angle(raw: str) -> list:
    """
    解析 GetAngle() 的返回值。
    输入可能是:
      - "0,{1.1869,-0.2338,86.7668,...},GetAngle();"
      - "{1.1869,-0.2338,86.7668,...}"
    输出: [1.1869, -0.2338, 86.7668, ...]
    """
    # 提取花括号里的内容
    start = raw.find("{")
    end = raw.find("}")
    if start != -1 and end != -1:
        raw = raw[start + 1 : end]
    return [float(x) for x in raw.split(",")]


# ============================================================
# 功能函数
# ============================================================
def check_tcp_mode():
    """
    检测当前是否处于 TCP 控制模式。
    原理：发 GetAngle()，如果返回角度值 = TCP 模式，返回 "Control Mode Is Not Tcp" = 非 TCP。
    """
    result = _send_dashboard("GetAngle()")
    if "Control Mode Is Not Tcp" in result:
        print("❌ 当前不在 TCP 模式")
        return False
    elif "{" in result:
        print("✅ 当前处于 TCP 模式")
        return True
    else:
        print(f"⚠️  无法判断，返回值: {result}")
        return None


def request_tcp_mode():
    """
    切换到 TCP 控制模式。
    ⚠️ 必须在下使能状态才能执行！先按 🔴 下使能 按钮，再调用此函数。
    """
    result = _send_dashboard("RequestControl()")
    print(f"RequestControl: {result}")
    return result


def enable_robot():
    """上使能：给机械臂各关节电机通电锁死。"""
    result = _send_dashboard("EnableRobot()")
    print(f"EnableRobot: {result}")
    return result


def disable_robot():
    """下使能：断开电机电流，可以用手掰动机械臂。"""
    result = _send_dashboard("DisableRobot()")
    print(f"DisableRobot: {result}")
    return result


def clear_error():
    """清除报警：当机械臂撞限位或报错后，清除错误码。"""
    result = _send_dashboard("ClearError()")
    print(f"ClearError: {result}")
    return result


def reset_robot():
    """复位机械臂。"""
    result = _send_dashboard("ResetRobot()")
    print(f"ResetRobot: {result}")
    return result


def get_angle():
    """
    读取 Nova 2 当前 6 个关节的角度。
    返回值单位：度（°）
    """
    raw = _send_dashboard("GetAngle()")
    if "Control Mode Is Not Tcp" in raw:
        print("❌ 不在 TCP 模式!")
        print("   解决: 按 🔴 下使能 → tcp_mode → enable → get_angle")
        return None

    try:
        angles_deg = _parse_angle(raw)
        print(f"J1: {angles_deg[0]:.2f}°")
        print(f"J2: {angles_deg[1]:.2f}°")
        print(f"J3: {angles_deg[2]:.2f}°")
        print(f"J4: {angles_deg[3]:.2f}°")
        print(f"J5: {angles_deg[4]:.2f}°")
        print(f"J6: {angles_deg[5]:.2f}°")
        return angles_deg
    except Exception as e:
        print(f"解析失败: {raw}, 错误: {e}")
        return None


def get_angle_rad():
    """读取角度并转换为弧度（标定时用这个）"""
    angles_deg = get_angle()
    if angles_deg is None:
        return None
    import math
    angles_rad = [a * math.pi / 180.0 for a in angles_deg]
    print(f"\n弧度 (用于 --start-joints):")
    print(f"[{', '.join(f'{a:.4f}' for a in angles_rad)}]")
    return angles_rad


# 置位姿态（度）。固定不变，作为标定的基准姿态
HOME_ANGLES_DEG = [0.0, 0.0, 90.0, 0.0, -90.0, 0.0]


def move_to_home(angles_deg=None, speed: int = 30, accel: int = 30, timeout: float = 20.0):
    """
    置位：用 MovJ 把 Nova2 送到固定的基准姿态（默认 0, 0, 90, 0, -90, 0 度）。

    为什么要用指令置位、而不是手拖：
        手拖每次的姿态都有几度差异，start_joints 跟着变，标定出来的 offset
        就不能复用了，每次都得重跑一遍标定。用固定置位指令，姿态每次完全一致，
        标定一次就能长期复用，每次开工只需要「置位 + 主手对齐」。

    ⚠️ 需要先上使能 enable。置位是大范围运动，跑之前请把急停放手边。
    """
    if angles_deg is None:
        angles_deg = HOME_ANGLES_DEG

    joints = ",".join(f"{a:.6f}" for a in angles_deg)
    cmd = "MovJ(joint={{{}}},a={:d},v={:d})".format(joints, int(accel), int(speed))
    print(f"置位目标: {list(angles_deg)} °    速度 {speed}%   加速度 {accel}%")
    result = _send_dashboard(cmd, timeout=5.0)
    print(f"MovJ: {result}")

    # 等机械臂停稳：连续两次 GetAngle 变化都小于 0.05° 就认为到位
    prev = None
    t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(0.3)
        raw = _send_dashboard("GetAngle()")
        if "{" not in raw:
            print(f"⚠️  读取角度失败（可能不在 TCP 模式）: {raw}")
            return None
        cur = _parse_angle(raw)
        if prev is not None and max(abs(c - p) for c, p in zip(cur, prev)) < 0.05:
            print("\n✅ 已到位。实际角度（度）:")
            for i, a in enumerate(cur):
                print(f"J{i+1}: {a:.2f}°")
            print("\n弧度 (用于 --start-joints):")
            print(f"[{', '.join(f'{a * math.pi / 180.0:.4f}' for a in cur)}]")
            return cur
        prev = cur

    print("⚠️  等待超时，机械臂可能还在动，请目视确认后再操作。")
    return None


def tool_do(index: int, status: int, instant: bool = True):
    """
    设置末端（工具）数字输出端口状态 —— 控制末端航插上的夹爪。

    夹爪接在 Nova2 末端航插的 DO_1 / DO_2 上，属「末端 DO」，所以用
    ToolDO 系列指令；末端航插定义：DO_1 → index 1，DO_2 → index 2。

    默认用 ToolDOInstant（立即指令）：不走算法队列，实时遥操时不会被前面
    排队中的 ServoJ / 运动指令阻塞，夹爪响应更跟手。

    index:  末端 DO 编号（1 或 2）
    status: 1 打开（ON），0 关闭（OFF）
    """
    if instant:
        cmd = "ToolDOInstant({:d},{:d})".format(int(index), int(status))
    else:
        cmd = "ToolDO({:d},{:d})".format(int(index), int(status))
    result = _send_dashboard(cmd)
    print(f"{cmd}  →  {result}")
    return result


def get_tool_do(index: int):
    """读末端 DO 端口状态。返回 0 为关闭，1 为打开。"""
    cmd = "GetToolDO({:d})".format(int(index))
    result = _send_dashboard(cmd)
    print(f"{cmd}  →  {result}")
    return result


def gripper(action: str, pulse: float = 0.2):
    """
    易爪柔性夹爪开 / 闭。

    实测（2026-10-06）：
        末端 DO_1: 1 → 0   张开一次
        末端 DO_2: 1 → 0   闭合一次

    夹爪是**边沿触发**的：光置 1 不动，必须把 1 拉回 0 才产生一次动作。
    不管之前处于什么状态，只要「先置 1、再置 0」就能重新触发一次。

    action: open（张开）/ close（闭合）
    pulse:  置 1 之后保持多久再拉回 0，单位秒
    """
    if action == "open":
        idx = 1
    elif action == "close":
        idx = 2
    else:
        print(f"未知动作: {action}（可选 open / close）")
        return None

    tool_do(idx, 1)
    time.sleep(pulse)
    tool_do(idx, 0)


# ============================================================
# MS42DC 步进电机机械爪（走串口 /dev/ttyACM0(改进为/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030368-if00)，不经 Nova2）
# ============================================================
# 通协议以《MS42DC步进电机机械爪用户手册-20260324.pdf》表 4-1 为准。
# 手册同步给出了 5 个完整数据帧示例，BCC 逐一验算均吻合，故以下实现严格
# 按手册的字节顺序，不采用 Individual_control/grapper_two.py 里那版
# （那版把细分值 0x20 放到了位置字节之后、位置又写成低字节在前，与手册不符）。
#
# 11 字节帧结构：
#   [0] 7BH    帧头
#   [1] 01H    设备地址
#   [2] 02H    控制模式（位置控制）
#   [3] DIR    转向：0 = 逆时针 = 张开，1 = 顺时针 = 闭合
#   [4] 20H    细分值（32 细分）
#   [5] POS_H  ┐ 角度数据，放大十倍，高八位在前
#   [6] POS_L  ┘
#   [7] SPD_H  ┐ 转速数据，放大十倍，高八位在前
#   [8] SPD_L  ┘
#   [9] BCC    前九个字节的异或和
#   [10] 7DH   帧尾
MS42DC_PORT = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030368-if00"
MS42DC_BAUDRATE = 115200
MS42DC_TIMEOUT = 0.12

MS42DC_DEVICE = 0x01     # 设备地址，默认 1
MS42DC_SUBDIV = 0x20     # 步进电机细分值：32 细分
MS42DC_SPEED = 0x012C    # 转速 300 = 30 rad/s（原 0x00C8 = 200 = 20 rad/s），与 gello/robots/nova2.py 保持一致
# 位置参数取手册示例：张开/闭合同为 0x4920 = 18720 → 1872.0° = 5.2 圈，
# 靠 DIR 区分正反转（螺丝杆机构，正转 5.2 圈闭合，反转 5.2 圈回原点）。
MS42DC_POS_OPEN = 0x4920
MS42DC_POS_CLOSE = 0x4920


def _ms42dc_frame(direction: int, position: int, speed: int = MS42DC_SPEED) -> bytes:
    """
    按手册表 4-1 组装 11 字节控制帧（帧头 + 9 字节数据 + BCC + 帧尾）。

    direction: 0 = 张开（逆时针），1 = 闭合（顺时针）
    position:  角度值 × 10，范围 0~65535，高八位在前
    speed:     转速值 × 10，高八位在前
    """
    body = [
        0x7B,                          # 帧头
        MS42DC_DEVICE,                 # 设备地址
        0x02,                          # 位置控制模式
        direction & 0xFF,              # 转向
        MS42DC_SUBDIV,                 # 细分值
        (position >> 8) & 0xFF,        # POS_H：高八位在前
        position & 0xFF,               # POS_L
        (speed >> 8) & 0xFF,           # SPD_H：高八位在前
        speed & 0xFF,                  # SPD_L
    ]
    bcc = 0
    for byte in body:                  # BCC = 前九个字节异或
        bcc ^= byte
    return bytes(body + [bcc, 0x7D])   # + BCC + 帧尾


def ms42dc_gripper(action: str,
                   port: str = MS42DC_PORT,
                   baudrate: int = MS42DC_BAUDRATE,
                   timeout: float = MS42DC_TIMEOUT):
    """
    MS42DC 步进电机机械爪 开 / 闭（独立串口，不经过 Nova2 的末端 DO）。

    与上面 gripper() 的区别：
        gripper()        —— 易爪柔性夹爪，走 Nova2 末端 DO_1 / DO_2，边沿触发。
        ms42dc_gripper() —— 步进电机夹爪，走 /dev/ttyACM0，发一帧就走，无反馈。

    注意：本函数只负责「把目标位置发出去」，不做任何回读。
    串口返回的报文也不能当作实际位置反馈使用。

    action: open（张开）/ close（闭合）
    """
    if action == "open":
        direction, position = 0, MS42DC_POS_OPEN
    elif action == "close":
        direction, position = 1, MS42DC_POS_CLOSE
    else:
        print(f"未知动作: {action}（可选 open / close）")
        return None

    try:
        import serial
    except ImportError:
        print("❌ 缺少 pyserial，无法控制 MS42DC 夹爪。")
        print("   安装: pip install pyserial")
        return None

    frame = _ms42dc_frame(direction, position)
    print(f"MS42DC {action}: port={port} @{baudrate}  "
          f"dir={direction} pos={position}(={position / 10:.1f}° 高字节在前) "
          f"speed={MS42DC_SPEED}")
    print("  发送帧: " + " ".join(f"{b:02X}" for b in frame))

    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            time.sleep(0.05)           # 等串口 DTR/RTS 稳定后再发，避免首字节丢失
            ser.write(frame)
    except Exception as e:
        print(f"❌ 串口发送失败: {e}")
        print("   检查: 1) USB 是否插好  2) ls /dev/ttyACM*  3) 是否有权限")
        return None

    print("✅ 已发送")
    return frame


def robot_mode():
    """查看机械臂当前状态。"""
    result = _send_dashboard("RobotMode()")
    print(f"RobotMode: {result}")
    return result


# ============================================================
# 主入口
# ============================================================
def main():
    parser = argparse.ArgumentParser(
        description="Nova 2 机械臂工具箱",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
    python scripts/nova2_utils.py get_angle      # 读当前关节角（度）
    python scripts/nova2_utils.py get_angle_rad   # 读当前关节角（弧度）
    python scripts/nova2_utils.py check_tcp       # 检测是否在 TCP 模式
    python scripts/nova2_utils.py tcp_mode        # 切换到 TCP 模式（需先下使能）
    python scripts/nova2_utils.py enable          # 上使能
    python scripts/nova2_utils.py disable         # 下使能
    python scripts/nova2_utils.py status          # 查看状态
    python scripts/nova2_utils.py home            # 置位到 0 0 90 0 -90 0（度）+ 夹爪张开
    python scripts/nova2_utils.py home --angles 0 0 90 0 -90 0 --speed 20
    python scripts/nova2_utils.py tool_do 1 1     # 末端 DO_1 打开（立即指令）
    python scripts/nova2_utils.py tool_do 2 0     # 末端 DO_2 关闭
    python scripts/nova2_utils.py get_tool_do 1   # 读末端 DO_1 状态
    python scripts/nova2_utils.py gripper open    # 夹爪张开
    python scripts/nova2_utils.py gripper close   # 夹爪闭合
    python scripts/nova2_utils.py ms42dc open     # MS42DC 步进夹爪张开（串口）
    python scripts/nova2_utils.py ms42dc close    # MS42DC 步进夹爪闭合（串口）
        """,
    )
    parser.add_argument(
        "command",
        choices=[
            "get_angle",
            "get_angle_rad",
            "check_tcp",
            "tcp_mode",
            "enable",
            "disable",
            "clear_error",
            "reset",
            "status",
            "home",
            "tool_do",
            "get_tool_do",
            "gripper",
            "ms42dc",
        ],
        help="要执行的操作",
    )
    parser.add_argument(
        "params",
        nargs="*",
        default=[],
        help="tool_do / get_tool_do / gripper / ms42dc 的参数",
    )
    parser.add_argument(
        "--queued",
        action="store_true",
        help="tool_do 改用队列指令 ToolDO（默认用立即指令 ToolDOInstant）",
    )
    parser.add_argument(
        "--angles",
        type=float,
        nargs="+",
        default=None,
        help="home 的置位目标角度（度），默认 0 0 90 0 -90 0",
    )
    parser.add_argument(
        "--speed", type=int, default=30, help="home 的运动速度比例 %%（默认 30）"
    )

    args = parser.parse_args()

    if args.command == "get_angle":
        get_angle()
    elif args.command == "get_angle_rad":
        get_angle_rad()
    elif args.command == "check_tcp":
        check_tcp_mode()
    elif args.command == "tcp_mode":
        request_tcp_mode()
    elif args.command == "enable":
        enable_robot()
    elif args.command == "disable":
        disable_robot()
    elif args.command == "clear_error":
        clear_error()
    elif args.command == "reset":
        reset_robot()
    elif args.command == "status":
        robot_mode()
    elif args.command == "home":
        # home = 六轴回位 + 夹爪张开。先回位再张夹爪，避免臂还在动时夹爪动作。
        move_to_home(args.angles, speed=args.speed)
        ms42dc_gripper("open")
    elif args.command == "tool_do":
        tool_do(int(args.params[0]), int(args.params[1]), instant=not args.queued)
    elif args.command == "get_tool_do":
        get_tool_do(int(args.params[0]))
    elif args.command == "gripper":
        gripper(args.params[0])
    elif args.command == "ms42dc":
        ms42dc_gripper(args.params[0])


if __name__ == "__main__":
    main()
