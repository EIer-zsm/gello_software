# Nova 2 遥操作流程

> 系统组成：Dynamixel 舵机组装的主手（leader）+ DOBOT Nova 2 六轴从臂（follower）。
> 标定目标：确定主手码盘 → Nova2 关节角的映射关系，即配置里的 `joint_offsets` 与 `joint_signs`。
> 工作目录：`/home/zsm/gello_project/gello_software`（下文所有命令均在此目录下执行）

---

## 一、硬件接线

| 设备 | 连接方式 | 地址 |
| --- | --- | --- |
| Nova 2 从臂 | 网线直连电脑网口 | `192.168.37.3`（电脑网口需设为同网段静态 IP，如 `192.168.37.100`，掩码 `255.255.255.0`） |
| GELLO 主手 | USB 转串口 | `/dev/ttyUSB0`，波特率 57600 |

确认主手串口存在：

```bash
ls -l /dev/ttyUSB*
```

---

## 二、激活虚拟环境（每个新终端都要做）

```bash
conda activate gello
cd /home/zsm/gello_project/gello_software
```

自检（确认依赖齐全、模块可导入）：

```bash
python -c "import dynamixel_sdk, zmq, omegaconf, tyro, gello; print('env OK')"
python -c "import sys; sys.path.insert(0,'.'); from gello.robots.nova2 import Nova2Robot; print('nova2 import OK')"
```

预期输出：`env OK`、`nova2 import OK`。

> 若报缺依赖：`pip install -e . && pip install -e third_party/DynamixelSDK/python`

---

## 三、切到 TCP 控制模式（顺序不能颠倒）

Nova 2 出厂默认不是 TCP 模式，遥操前必须切换。**切之前必须先下使能。**

```bash
python scripts/nova2_utils.py check_tcp    # 检测当前是否已在 TCP 模式
python scripts/nova2_utils.py disable      # 下使能（切 TCP 前必须）
python scripts/nova2_utils.py tcp_mode     # 请求控制权 RequestControl
python scripts/nova2_utils.py enable       # 重新上使能
```

实测输出：

```
❌ 当前不在 TCP 模式          ← 或 ✅ 当前处于 TCP 模式
DisableRobot: 0,{},DisableRobot();
RequestControl: 0,{},RequestControl();
EnableRobot: 0,{},EnableRobot();
```

返回字符串以 `0,{},` 开头表示执行成功。

---

## 四、置位（代替手动拖拽）

```bash
python scripts/nova2_utils.py home
```

用 `MovJ` 把 Nova 2 送到固定的基准姿态 **0 0 90 0 -90 0（度）**，并自动等机械臂停稳、打印实际角度：

```
置位目标: [0.0, 0.0, 90.0, 0.0, -90.0, 0.0] °    速度 30%   加速度 30%
MovJ: 0,{208},MovJ(joint={0.000000,0.000000,90.000000,0.000000,-90.000000,0.000000},a=30,v=30);

✅ 已到位。实际角度（度）:
J1: 0.00°
J2: 0.00°
J3: 90.00°
J4: 0.00°
J5: -90.00°
J6: 0.00°

弧度 (用于 --start-joints):
[0.0000, 0.0000, 1.5708, 0.0000, -1.5708, 0.0000]
```

可选参数：

```bash
python scripts/nova2_utils.py home --speed 20                 # 慢一点（默认 30%）
python scripts/nova2_utils.py home --angles 0 0 90 0 -90 0    # 自定义目标角度
```

**为什么要用指令置位、而不是手拖：**
手拖每次的姿态都有几度差异，`start_joints` 跟着变，标定出来的 offset 就不能复用，每次都得重跑一遍标定。用固定置位指令，姿态每次完全一致，标定一次就能长期复用。

> ⚠️ 置位是大范围运动。第一次试建议 `--speed 10`，并把急停放手边。

---

## 五、日常开工流程（标定过之后，每次都这样）

```bash
# ① 激活环境
conda activate gello && cd /home/zsm/gello_project/gello_software

# ② 切 TCP 模式
python scripts/nova2_utils.py check_tcp
python scripts/nova2_utils.py disable
python scripts/nova2_utils.py tcp_mode
python scripts/nova2_utils.py enable

# ③ 置位
python scripts/nova2_utils.py home

# ④ 用手把主手摆成和 Nova 2 一样的姿态（目视对齐即可）

# ⑤ 启动遥操作
python experiments/launch_yaml.py --left-config-path configs/nova2_real.yaml
```

因为置位姿态固定，**`start_joints` 永远是 `[0, 0, 1.5708, 0, -1.5708, 0]`，配置不用再改**。

---

## 六、首次标定流程（只在主手零点变动后需要重做）

### 6.1 读取基准角度

置位之后读一次，确认角度：

```bash
python scripts/nova2_utils.py get_angle_rad
```

### 6.2 主手对齐

用手把主手摆成和 Nova 2 一样的姿态（0 0 90 0 -90 0 度）。

**对齐质量直接决定标定精度** —— 两者相差越大，offset 越容易跳到隔壁的 π/2 格上。

### 6.3 运行标定

```bash
python scripts/gello_get_offset.py \
    --port /dev/ttyUSB0 \
    --start-joints 0.0000 0.0000 1.5708 0.0000 -1.5708 0.0000 \
    --joint-signs 1 1 -1 1 1 1 \
    --no-gripper
```

**两个参数必须显式指定，不能省略：**

- `--no-gripper`：脚本默认 `gripper=True`，会去找第 7 个电机（夹爪）。本系统只有 6 轴、没接夹爪，不写这个参数会直接报错。
- `--joint-signs`：**必须和 config 里的 `joint_signs` 完全一致**。脚本默认值是 `1 1 -1 1 1 1`，若 config 里改了，这里也要同步改，否则算出来的 offset 跟配置对不上。

实测输出：

```
best offsets               :  ['0.000', '1.571', '7.854', '4.712', '3.142', '1.571']
best offsets function of pi: [0*np.pi/2, 1*np.pi/2, 5*np.pi/2, 3*np.pi/2, 2*np.pi/2, 1*np.pi/2 ]
```

### 6.4 标定原理

脚本暴力搜索 `offset`（限定为 `π/2` 的整数倍，搜索范围 `±8π`），使下式误差最小：

```
映射值 = joint_signs[i] × (主手码盘[i] − offset)
误差   = |映射值 − start_joints[i]|
```

也就是找一个「π/2 倍数」的平移量，把主手读数对齐到 Nova 2 的实际角度。

**两个重要结论：**

- **单姿态标定在数学上无法确定 `joint_signs`（方向）** —— 同一个姿态下，`+1` 和 `-1` 都能靠调 offset 拟合上。方向必须靠第七步实际转动关节来验证。
- **offset 只有 π/2 精度**，会有几度残差（正常现象）。残余的**整圈误差（±2π）不用管**，运行时会被自动补齐。

> 举例：主手 J3 码盘 356°、Nova2 J3 实际 89.53°，sign 取 -1 时理想 offset = 356 + 89.53 = 445.5°，取最近的 π/2 格即 450° = `5π/2` = `7.854 rad`，残差 4.5°。

### 6.5 回填配置

编辑 `configs/nova2_real.yaml`：

```yaml
robot:
  _target_: gello.robots.nova2.Nova2Robot
  robot_ip: "192.168.37.3"

agent:
  _target_: gello.agents.gello_agent.GelloAgent
  port: "/dev/ttyUSB0"
  dynamixel_config:
    _target_: gello.agents.gello_agent.DynamixelRobotConfig
    joint_ids: [1, 2, 3, 4, 5, 6]
    # 2026-09-22 标定: offsets=[0, 90, 450, 270, 180, 90]°
    joint_offsets: [0.000, 1.571, 7.854, 4.712, 3.142, 1.571]
    joint_signs: [1, 1, -1, 1, 1, 1]
    gripper_config: null
  # 固定置位姿态，不用再改了
  start_joints: [0.0000, 0.0000, 1.5708, 0.0000, -1.5708, 0.0000]

hz: 30
```

| 字段 | 填什么 |
| --- | --- |
| `joint_offsets` | 第 6.3 步输出的 `best offsets` |
| `joint_signs` | 暂定值，需第七步实测验证 |
| `start_joints` | 固定为置位姿态 `[0, 0, 1.5708, 0, -1.5708, 0]`，**不用改** |

> 关于 J3：`4.712` 和 `-1.571` 相差 `2π`，**完全等价**，哪个都行，不用纠结。

---

## 七、方向验证（每次改过 joint_signs 后必须做）

启动遥操后，**一次只转主手的一个关节**，核对 Nova 2 的对应关节：

| 现象 | 处理 |
| --- | --- |
| Nova 2 反向跟随 | 把该轴 `joint_signs[i]` 取反（`1` ↔ `-1`），重跑 |
| 零位差 π/2 的整数倍 | 微调该轴 `joint_offsets[i]`，加/减 `π/2` 的整数倍 |
| 转一个关节时别的关节跟着动 | 检查电机 ID 顺序是否与 `joint_ids` 一致 |

6 个轴逐个过完，把最终值写回 `configs/nova2_real.yaml`。

---

## 八、退出与急停

- `Ctrl+C` 退出，会走 cleanup。
- 命令下使能：`python scripts/nova2_utils.py disable`（电机断电，机械臂会因重力松垂）。
- 也可按 Nova 2 本体/示教器上的实体按钮：下使能 / 停止 / 清除报警。

---

## 九、什么时候需要重新标定

只有**主手的物理零点变了**才需要重标，例如：

- 主手被人掰着转过整圈
- 主手舵机拆装、换过件
- 改过 `joint_ids` 顺序

正常每天开机**不需要**重标。

---

## 十、辅助工具速查

| 命令 | 作用 |
| --- | --- |
| `python scripts/nova2_utils.py check_tcp` | 检测是否处于 TCP 模式 |
| `python scripts/nova2_utils.py get_angle` | 读当前关节角（度） |
| `python scripts/nova2_utils.py get_angle_rad` | 读当前关节角（弧度） |
| `python scripts/nova2_utils.py home` | 置位到 0 0 90 0 -90 0（度） |
| `python scripts/nova2_utils.py enable` / `disable` | 上使能 / 下使能 |
| `python scripts/nova2_utils.py clear_error` | 清除报警 |
| `python scripts/nova2_utils.py status` | 查看机械臂状态 |
| `python scripts/gello_get_offset.py` | 标定脚本，暴力搜索 `joint_offsets` |
| `python scripts/gello_raw.py` | 主手码盘诊断：只读主手、不连从臂、实时打印 6 轴码盘 |
| `experiments/launch_yaml.py` | 启动遥操作控制循环 |

---

## 附：各文件职责

| 文件 | 作用 |
| --- | --- |
| `scripts/nova2_utils.py` | Nova2 调试工具箱：读角度、上下使能、切 TCP、置位、看状态 |
| `scripts/gello_get_offset.py` | 官方标定脚本，暴力搜索 `joint_offsets` |
| `scripts/gello_raw.py` | 主手码盘诊断（只读主手） |
| `gello/robots/nova2.py` | Nova2 从臂接口（初始化、读反馈、发 ServoJ） |
| `configs/nova2_real.yaml` | 真机配置，标定结果写在这里 |

---

