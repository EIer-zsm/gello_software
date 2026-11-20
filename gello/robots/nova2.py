import time
from typing import Dict

import numpy as np

from gello.robots.robot import Robot

# MS42DC 夹爪目标转速，单位 rad/s，编码时 ×10 写入 SPD_H/SPD_L。
# 原值 200 = 20 rad/s；本次测试改为 300 = 30 rad/s，效果不好就改回 200。
MS42DC_SPEED = 300

# 单帧允许的最大关节跳变（度），用来挡反馈坏帧（见 get_joint_state）。
# 实测依据（23865 帧训练集，已剔除坏帧）：
#   真实运动单帧最大 19.4°（J5 腕部），99.9% 的帧 < 6.2°；
#   坏帧是 345 ~ 415°。
# 取 30°，两边都留余量：正常运动 19.4° < 30° < 坏帧 345°。
# 30 Hz 下 30° ≈ 900 °/s，远高于机构实际能力，不会误伤。
MAX_JOINT_JUMP_DEG = 30.0

# 读帧间隔变长时（例如循环卡顿 0.5 s）允许的跳变按时间比例放大，
# 避免慢读把真实运动误判成坏帧。30 Hz 下等效就是上面的 30°。
MAX_JOINT_SPEED_DEG_PER_S = 900.0

# 连续多少帧都和参考值对不上就认为参考值本身过期、重新锁定。
# 单帧坏帧只会连续被拒 1 次，远达不到这个数；只有"上电第一帧就是坏帧"
# （之后每帧好数据都被拒）这种情况才会触发（30 Hz 下 10 帧 ≈ 0.33 s）。
RELOCK_AFTER_REJECTS = 10


class Nova2Robot(Robot):
    """A class representing the DOBOT Nova 2 robot."""

    def __init__(self, robot_ip: str = "192.168.37.3"):
        import sys

        # 引入 DOBOT 官方 V4 TCP/IP API
        sys.path.insert(0, "/home/zsm/gello_project/TCP-IP-Python-V4")
        from dobot_api import DobotApiDashboard, DobotApiFeedBack

        # 29999：Dashboard 指令接口
        self._dash = DobotApiDashboard(robot_ip, 29999)

        # 获取控制权
        self._dash.sendRecvMsg("RequestControl()")

        # 机器人使能
        self._dash.sendRecvMsg("EnableRobot()")
        time.sleep(2.0)

        # 限制机器人整体速度
        self._dash.sendRecvMsg("SpeedFactor(50)")

        # 30004：状态反馈接口
        self._feedback = DobotApiFeedBack(robot_ip, 30004)

        # 初始化反馈数据
        self._feedback.socket_dobot.recv(1440)
        self._cached_joints = np.zeros(6)

        # 反馈坏帧保护：上一次被接受的读数（度）和累计丢弃次数。
        # None 表示还没读到过有效帧，此时不做跳变判断（避免用零位挡住真实姿态）。
        self._last_valid_deg = None
        self._rejected_frames = 0
        self._consecutive_rejects = 0
        self._last_read_t = None

        # 夹爪状态：None 表示还没收到过指令（首帧只记录、不发脉冲）
        self._gripper_state = None
        self._gripper_pos = 0.0

        # MS42DC 步进夹爪（串口，替代损坏的 DO 边沿触发）
        # 用 /dev/serial/by-id/ 稳定路径：/dev/ttyACM0 的编号随插拔/枚举顺序变，
        # 会和相机一样指到别的设备上，指令发出去但夹爪不动。
        # 原写法（节点会漂移，保留备查）：serial.Serial("/dev/ttyACM0", 115200, timeout=0.12)
        # 打开失败不算致命：打印警告，夹爪不可用但 6 轴照常工作
        self._gripper_ser = None
        try:
            import serial

            self._gripper_ser = serial.Serial(
                "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030368-if00",
                115200,
                timeout=0.12,
            )
            print(
                "[Nova2Robot] MS42DC 串口夹爪已连接: "
                "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B0B030368-if00 @115200"
            )
        except Exception as e:
            print(f"[Nova2Robot] ⚠️ 打开 MS42DC 串口失败（夹爪不可用）: {e}")

        print(f"[Nova2Robot] Connected to {robot_ip}")

    def num_dofs(self) -> int:
        """Get the number of robot joints."""
        return 7

    # ---- 原 DO 边沿触发（控制板损坏，暂时停用；修好后取消注释即可恢复）----
    # def _pulse_tool_do(self, index: int) -> None:
    #     """夹爪是边沿触发：末端 DO 先置 1,再拉回 0,才产生一次开/闭动作。"""
    #     self._dash.send_data("ToolDOInstant({:d},1)".format(index))
    #     time.sleep(0.1)
    #     self._dash.send_data("ToolDOInstant({:d},0)".format(index))

    def _ms42dc_frame(self, direction: int) -> bytes:
        """按 MS42DC 手册表 4-1 组装 11 字节控制帧。

        7B 01 02 DIR 20 POS_H POS_L SPD_H SPD_L BCC 7D
        位置 = 0x4920 = 18720 = 1872.0 度 = 5.2 圈，开/闭同值，靠 DIR 区分正反转。
        DIR: 0 = 逆时针 = 张开，1 = 顺时针 = 闭合。BCC = 前九个字节异或。
        """
        position = 0x4920
        body = [
            0x7B,                        # 帧头
            0x01,                        # 设备地址
            0x02,                        # 位置控制模式
            direction & 0xFF,            # 转向
            0x20,                        # 细分值 32
            (position >> 8) & 0xFF,      # POS_H
            position & 0xFF,             # POS_L
            (MS42DC_SPEED >> 8) & 0xFF,  # SPD_H
            MS42DC_SPEED & 0xFF,         # SPD_L：300 = 30 rad/s（原 0x00C8 = 20 rad/s）
        ]
        bcc = 0
        for byte in body:
            bcc ^= byte
        return bytes(body + [bcc, 0x7D])

    def _pulse_ms42dc(self, direction: int) -> None:
        """向 MS42DC 夹爪发一帧。direction: 0 = 张开，1 = 闭合。"""
        if self._gripper_ser is None:
            print("[Nova2Robot] ⚠️ MS42DC 串口未连接，跳过夹爪指令")
            return
        try:
            self._gripper_ser.write(self._ms42dc_frame(direction))
        except Exception as e:
            print(f"[Nova2Robot] ⚠️ MS42DC 发送失败: {e}")

    def get_joint_state(self) -> np.ndarray:
        """Get the current joint positions (6 轴 + 1 夹爪)。

        反馈 socket 偶发单帧错位：某一帧关节读到不可能的值、下一帧立刻恢复
        （例如 [0,0,90,0,-90,0] -> [0,0,0,-345,-117.5,415.4]）。实测训练集
        47/55 条、测评集 19/20 条各出现 1 帧，全部落在 episode 第 1 帧。
        这里把"跳变超过 MAX_JOINT_JUMP_DEG"的帧判为坏帧并丢弃，沿用上次有效值，
        这样录制数据和策略的输入都不会再被污染。
        """
        data = self._feedback.feedBackData()

        if data is not None:
            joints_deg = np.asarray(data["QActual"][0], dtype=float).reshape(-1)[:6]
            rejected = False

            # 按实际读帧间隔缩放允许的跳变量（正常 30 Hz 时恰好等于 MAX_JOINT_JUMP_DEG）
            now = time.time()
            dt = 0.0 if self._last_read_t is None else max(now - self._last_read_t, 0.0)
            self._last_read_t = now
            jump_limit = max(MAX_JOINT_JUMP_DEG, MAX_JOINT_SPEED_DEG_PER_S * dt)

            if joints_deg.size < 6 or not np.isfinite(joints_deg).all():
                # 包不完整或含 NaN/Inf：没有可用数据，丢弃
                rejected = True
            elif self._last_valid_deg is None:
                # 首帧没有参考值，无条件接受（否则零位会一直挡住真实姿态）
                self._last_valid_deg = joints_deg
                self._cached_joints = np.deg2rad(joints_deg)
            elif np.max(np.abs(joints_deg - self._last_valid_deg)) > jump_limit:
                # 单帧跳变：超出该读帧间隔内可能的运动量，判为坏帧
                self._consecutive_rejects += 1
                if self._consecutive_rejects >= RELOCK_AFTER_REJECTS:
                    # 连续多帧都对不上参考值 → 参考值本身过期（例如上电第一帧就是
                    # 坏帧，之后每帧好数据都被拒），重新锁定到当前读数
                    self._last_valid_deg = joints_deg
                    self._cached_joints = np.deg2rad(joints_deg)
                    self._consecutive_rejects = 0
                    print(
                        "[Nova2Robot] ⚠️ 连续 %d 帧与参考值不符，重新锁定当前读数"
                        % RELOCK_AFTER_REJECTS
                    )
                else:
                    rejected = True
            else:
                self._last_valid_deg = joints_deg
                self._cached_joints = np.deg2rad(joints_deg)
                self._consecutive_rejects = 0

            if rejected:
                self._rejected_frames += 1
                # 只在第 1 次和每 50 次打印，避免刷屏
                if self._rejected_frames == 1 or self._rejected_frames % 50 == 0:
                    print(
                        f"[Nova2Robot] ⚠️ 丢弃反馈坏帧 {self._rejected_frames} 次"
                        f"（沿用上次关节值）"
                    )

        return np.append(self._cached_joints, self._gripper_pos)

    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Command the robot to a given joint state.

        joint_state: 前 6 个是关节角（弧度），第 7 个是夹爪值 [0,1] 0=张开, 1=闭合）。
        """
        # ---- 夹爪：只在状态翻转时发一次脉冲，不影响 6 轴 ----
        gripper_val = joint_state[6]
        desired = "open" if gripper_val < 0.5 else "close"
        if self._gripper_state is None:
            self._gripper_state = desired  # 首帧只记录，避免启动时多打一次
        elif desired != self._gripper_state:
            # 原 DO 边沿触发（硬件损坏，暂时注释，修好后恢复）:
            # self._pulse_tool_do(1 if desired == "open" else 2)
            self._pulse_ms42dc(0 if desired == "open" else 1)
            self._gripper_state = desired
            self._gripper_pos = 0.0 if desired == "open" else 1.0

        # ---- 6 轴：原逻辑，一行未改 ----
        angles_deg = np.rad2deg(joint_state[:6])

        cmd = "ServoJ({:f},{:f},{:f},{:f},{:f},{:f})".format(
            *angles_deg
        )

        self._dash.send_data(cmd)

    def get_observations(self) -> Dict[str, np.ndarray]:
        """Get robot observations."""

        joints = self.get_joint_state()

        return {
            "joint_positions": joints,
            "joint_velocities": np.zeros(7),
            "ee_pos_quat": np.zeros(7),
            "gripper_position": np.array([self._gripper_pos]),
        }