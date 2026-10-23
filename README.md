# 兆瓦级风电机组液压变桨系统数字孪生仿真机 (Pitch Sim)

本项目为大型兆瓦级风电机组三面独立变桨液压系统的高精度物理动力学数字孪生与实时仿真系统，支持通过 CODESYS OPC UA 协议与下位机 PLC（`PRG_pitch_sim` 及底层硬件采样通道 `Common.blade[i].io.ai.raw.*`）实现 **200Hz (5ms)** 超高速微周期双向闭环硬件在环 (HIL) / 软件在环 (SIL) 测试。

---

## 一、目录结构与文件组织

```text
pitch_sim/
│
├── pitch_gui.py                 # PyQt6 实时图形化数字孪生控制台 (支持矢量动态拓扑图/示波器)
├── pitch_soft_realtime.py       # 软实时多物理场调度引擎 (5ms/200Hz 物理时钟 1:1 对齐)
├── pitch_simulation.py          # 高精度液压动力学求解器 (双油缸、差动、蓄能器、锁紧机构)
├── pitch_opcua_bridge.py        # CODESYS OPC UA 200Hz 高速双向异步通讯网关 (自适应节点检测)
│
├── start_gui.bat                # 独立启动控制台的批处理快捷脚本
├── test_pitch_sim_opcua.py      # OPC UA 接口批量读写与心跳自测脚本
├── test_opcua_rw.py             # 简易 OPC UA 连通性测试脚本
├── test_udp_echo.py             # 辅助调试脚本
└── requirements.txt             # Python 依赖清单
```

---

## 二、快速启动运行

### 1. 环境依赖安装
```bash
pip install -r requirements.txt
```

### 2. 启动方式

#### 方式 1：双击批处理脚本启动
* 在当前目录双击运行 `start_gui.bat`

#### 方式 2：命令行启动
```powershell
python pitch_gui.py
```
若需要启动时直接进入远程 OPC UA 闭环模式，可附加参数：
```powershell
python pitch_gui.py --remote
```

---

## 三、核心功能与双模式运行机制

### 1. 本地控制模式 (LOCAL)
* 用户可通过 UI 面板直接设定各叶片比例阀电压（$\pm 10.0\text{V}$）、测试紧急顺桨（120/103/116）、手动插拔变桨锁等。
* 内置 200Hz 高保真动力学积分，消除 Windows 任务调度抖动（采用粗休眠 + 微秒级自旋混合调度）。

### 2. 远程闭环控制模式 (REMOTE - OPC UA)
* 点击界面顶部工具栏的 **【远程控制】** 按钮：
  1. 自动启动后台 `PitchOpcUaBridge` 异步网关；
  2. 自动连接 CODESYS OPC UA 服务端（`opc.tcp://127.0.0.1:4840`）；
  3. 动态探测下位机 `PRG_pitch_sim` 变量表及 `Common.blade[i].io.ai.raw.*` 通道；
  4. 以 **200Hz (5ms)** 周期批量读取 PLC 阀门指令并推进物理动力学，实时将高精物理遥测回写至 PLC；
  5. 自动维护双向自增心跳看门狗（`udiHeartbeatPlc` / `udiHeartbeatSim`）。
* 点击 **【本地控制】** 按钮时，网关平滑断开并释放资源，无内存泄漏与死锁。

---

## 四、主要物理模型与参数规范

| 物理模块 | 关键参数 / 物理特性 |
| :--- | :--- |
| **比例方向阀 (102A)** | 额定规格：**压差 7 bar 时流量 75 L/min**（$A_{\text{max}} \approx 47.94\text{ mm}^2$）；响应时间常数 $\tau = 15\text{ ms}$；带 $\pm 0.01\text{V}$ 零位抗漂移死区；非对称差动回路（伸出位封死 B/T，强制走 104 单向阀形成差动增速）。 |
| **变桨双油缸 (124 & 125)** | 缸径 $\Phi 200\text{ mm}$，杆径 $\Phi 125\text{ mm}$，最大行程 $822\text{ mm}$，并联驱动。 |
| **蓄能器组 (106)** | 8 只 25L 并联，总容积 $200\text{ L}$，预充氮气压力 $100\text{ bar}$，绝热指数 $\kappa = 1.4$。 |
| **液压动力源 (HPU)** | 3 台定量泵并联，额定输出流量 $64.5\text{ L/min}$ @ 1450 rpm；25# 旁路加压阀闭环（$P_{107} < 235\text{ bar}$ 建压，$P_{107} > 265\text{ bar}$ 卸荷）。 |
| **紧急顺桨回路** | 120/103 阀（并联常开 NO）+ $\Phi 4.7\text{ mm}$ 节流孔 121 + 单向阀 110；116 顺桨泄荷阀（常开 NO）+ $\Phi 3.0\text{ mm}$ 阻尼孔 122。 |
| **变桨插销锁** | $\Phi 40/\Phi 20 \times 60\text{ mm}$ 锁缸 + 弹簧强力自锁机构；130 充压解锁阀 + $\Phi 0.8\text{ mm}$ 孔 131；132 泄油插销阀（$180\text{ mm}^2$）。 |
