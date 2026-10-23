"""
=============================================================================
软实时 (Soft Real-Time) 调度引擎 - 风电变桨双缸液压系统
=============================================================================
功能:
  1. 物理时钟 1:1 对齐 (10ms / 100Hz 工控节拍)
  2. 粗休眠 (time.sleep) + 微秒级自旋 (Spin-Wait) 混合调度，消除 Windows 任务抖动
  3. 线程安全读写接口 (set_inputs / get_telemetry)
  4. 支持【本地控制 (LOCAL)】与【远程控制 (REMOTE)】双权限模式
  5. 远程控制接口:
     - 输入: 每个电磁阀的指令 (109, 119A, 119B, 120, 103, 116, 130, 132), 比例阀 102A 电压
     - 输出: 油缸位移 (mm), 107 传感器母管压力 (bar), 变桨锁状态 (锁定/销位移/接近开关)
=============================================================================
"""

import sys
import time
import threading
from dataclasses import dataclass, field
from typing import Dict, Optional, Union
from pitch_simulation import PitchHydraulicSystem


@dataclass
class PitchControlInputs:
    """本地控制输入参数"""
    u_prop: float = 0.0          # 比例方向阀 102A 控制电压 [-10.0V, +10.0V]
    u_enable: float = 1.0        # 主使能阀 109 & 隔离阀 119A/B (1.0 开, 0.0 关)
    u_safe_feather: float = 0.0  # 应急顺桨电磁阀 (0.0 正常闭合, 1.0 触发顺桨)
    f_aero: float = 0.0          # 外部叶片气动推力扰动 (N)
    u_valve_130: float = 1.0     # 变桨锁充压阀 130 (1.0 得电开, 0.0 断电关)
    u_valve_132: float = 1.0     # 变桨锁泄荷阀 132 (1.0 得电关, 0.0 断电开通油箱)


@dataclass
class RemoteValveCommand:
    """
    远程控制输入接口数据结构 (每个电磁阀指令 + 比例阀电压)
    对应系统原理图元件:
    """
    prop_voltage: float = 0.0          # 比例伺服阀 102A 输入模拟量电压 [-10.0V, +10.0V]
    valve_109: bool = True             # 109 主供油使能电磁座阀 (True=得电开, False=失电隔离)
    valve_119A: bool = True            # 119A A腔安全隔离座阀 (True=得电开)
    valve_119B: bool = True            # 119B A腔安全隔离座阀 (True=得电开)
    valve_120: bool = False            # 120 顺桨充液电磁阀 (True=得电闭合保压, False=失电常开快速顺桨)
    valve_103: bool = False            # 103 顺桨充液电磁阀 (True=得电闭合保压, False=失电常开快速顺桨)
    valve_116: bool = True             # 116 B腔顺桨泄荷阀 (True=得电闭合保压, False=失电常开排油)
    valve_130: bool = True             # 130 变桨锁充压解锁阀 (True=得电充压解锁, False=断电)
    valve_132: bool = True             # 132 变桨锁泄荷自锁阀 (True=得电闭合保压, False=断电常开自锁)


@dataclass
class RemoteBladeFeedback:
    """单个变桨机构的远程状态遥测输出结构"""
    displacement_mm: float = 50.0      # 油缸位移 (mm)
    velocity_mmps: float = 0.0         # 油缸活塞运动线速度 (mm/s)
    velocity_mps: float = 0.0          # 油缸活塞运动速度 (m/s)
    valve_opening_volt: float = 0.0    # 比例伺服阀 102A 阀芯实际开度反馈电压 (±10.0V)
    valve_opening_pct: float = 0.0     # 比例伺服阀 102A 开度百分比 (-100.0% ~ +100.0%, 正向伸出顺桨, 负向缩回)
    valve_spool_pos: float = 0.0       # 比例伺服阀阀芯开度反馈电压 (±10.0V)
    is_locked: bool = False            # 变桨锁状态 (True=已插销机械锁死, False=拔销解锁)
    lock_pin_mm: float = 0.0           # 变桨锁销位移 (0.0=完全拔销解锁, 60.0=完全插销锁死)
    switch_129: bool = False           # 90° 顺桨对准开关 (True=主缸在808~822mm羽化位对准锁孔)
    switch_S_unlocked: bool = True     # 锁销接近开关 S: 拔销到位信号
    switch_S_locked: bool = False      # 锁销接近开关 S: 插销到位信号
    pressure_107_bar: float = 260.0    # 该面 107 蓄能器传感器压力 (bar)
    p_A_bar: float = 15.0              # A腔压力 (bar)
    p_B_bar: float = 15.0              # B腔压力 (bar)


@dataclass
class RemoteSystemFeedback:
    """全机组远程遥测反馈汇总 (包含 107 压力与各叶片状态)"""
    control_mode: str = 'LOCAL'        # 'LOCAL' (本地控制) 或 'REMOTE' (远程控制)
    pressure_107_bar: float = 260.0    # 107 蓄能器综合/最低压力 (bar)
    pressure_107_bar_A: float = 260.0  # A 面 107 蓄能器压力 (bar)
    pressure_107_bar_B: float = 260.0  # B 面 107 蓄能器压力 (bar)
    pressure_107_bar_C: float = 260.0  # C 面 107 蓄能器压力 (bar)
    p_supply_bar: float = 2.5          # 泵站供油口 P 实际压力 (bar)
    valve_25_state: bool = False       # 25# 电磁阀打压/卸荷状态 (True=打压, False=卸荷)
    q_hpu_lpm: float = 0.0             # 泵站实际供油流量 (L/min)
    motor_rpm: float = 1450.0          # 电机转速 (rpm)
    timestamp: float = 0.0             # 仿真时钟 (s)
    blades: Dict[str, RemoteBladeFeedback] = field(default_factory=dict)


@dataclass
class PitchTelemetryData:
    """本地 UI 绑定的遥测数据"""
    timestamp: float = 0.0       # 仿真时钟 (s)
    displacement_mm: float = 50.0# 双缸当前位移 (mm)
    velocity_mps: float = 0.0    # 活塞运动速度 (m/s)
    p_acc_bar: float = 260.0     # 蓄能器母管/107传感器压力 (bar)
    p_A_bar: float = 15.0        # A腔(无杆腔)压力 (bar)
    p_B_bar: float = 15.0        # B腔(有杆腔)压力 (bar)
    valve_position_pct: float = 0.0 # 比例阀实际开度 (%)
    cycle_time_ms: float = 0.0   # 单步实际计算耗时 (ms)
    jitter_ms: float = 0.0       # 调度抖动误差 (ms)
    overruns: int = 0            # 超时次数统计
    lock_pin_mm: float = 0.0     # 变桨锁销行程 (0.0=解锁, 60.0=锁死)
    is_locked: bool = False      # 机械闭锁标志
    switch_129: bool = False     # 90° 顺桨对准开关
    switch_S_unlocked: bool = True # 接近开关 S: 拔销解锁
    switch_S_locked: bool = False# 接近开关 S: 插销到位
    p_PP_bar: float = 260.0      # 锁缸有杆腔 PP 压力 (bar)
    # 液压站 (HPU) 动力源遥测
    valve_25_state: bool = False # 25# 电磁阀状态 (True=得电打压, False=失电卸荷)
    p_supply_bar: float = 2.5    # P口/泵口供油压力 (bar)
    q_hpu_lpm: float = 0.0       # 泵送向主回路的实际流量 (L/min)
    motor_rpm: float = 1450.0    # 电机转速 (rpm, 常通电)


class SoftRealtimePitchEngine:
    def __init__(self, cycle_time_s=0.010):
        self.cycle_time = cycle_time_s
        self.sub_dt = 0.001
        self.sub_steps = max(1, int(round(self.cycle_time / self.sub_dt)))

        self.blades = ['A', 'B', 'C']
        self.plants = {k: PitchHydraulicSystem(dt=self.sub_dt) for k in self.blades}
        self._lock = threading.Lock()
        
        # 控制权限模式: 'LOCAL' (本地控制, 默认) 或 'REMOTE' (远程控制)
        self.control_mode = 'LOCAL'
        self._inputs = {k: PitchControlInputs() for k in self.blades}
        self._remote_commands = {k: RemoteValveCommand() for k in self.blades}
        self._telemetries = {k: PitchTelemetryData() for k in self.blades}

        # 集中动力站 HPU 共享参数 (3台油泵合并 64.5 L/min @ 1450 rpm)
        self.num_pumps = 3
        self.V_disp_pump = 15.6e-6
        self.n_motor_rpm = 1450.0
        self.eta_v = 0.95
        self.Q_pump_total = self.num_pumps * self.V_disp_pump * (self.n_motor_rpm / 60.0) * self.eta_v
        self.P_set_hpu_relief = 280.0e5
        self.P_tank = 2.0e5
        self.valve_25_state = False
        self.P_supply = 260.0e5
        self.Q_hpu_actual = 0.0

        self._running = False
        self._thread = None
        self.sim_time = 0.0
        self.overruns = 0

    def _worker(self):
        if sys.platform == "win32":
            try:
                import ctypes
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception:
                pass

        target_next_time = time.perf_counter()
        last_t = target_next_time

        while self._running:
            t_start = time.perf_counter()

            # 1. 线程安全读取最新控制输入 (区分本地模式与远程接口模式)
            with self._lock:
                current_mode = self.control_mode
                if current_mode == 'REMOTE':
                    # 远程模式：根据下发的远程各电磁阀与比例阀指令合成动力学输入
                    current_inputs = {}
                    for k in self.blades:
                        rcmd = self._remote_commands[k]
                        u_prop = float(rcmd.prop_voltage)
                        u_enable = 1.0 if (rcmd.valve_109 and (rcmd.valve_119A or rcmd.valve_119B)) else 0.0
                        # 120/103/116 为失电常开顺桨，任意失电触发顺桨
                        u_safe_feather = 1.0 if (not rcmd.valve_120 or not rcmd.valve_103 or not rcmd.valve_116) else 0.0
                        u_valve_130 = 1.0 if rcmd.valve_130 else 0.0
                        u_valve_132 = 1.0 if rcmd.valve_132 else 0.0
                        current_inputs[k] = PitchControlInputs(
                            u_prop=u_prop,
                            u_enable=u_enable,
                            u_safe_feather=u_safe_feather,
                            f_aero=self._inputs[k].f_aero,
                            u_valve_130=u_valve_130,
                            u_valve_132=u_valve_132
                        )
                else:
                    # 本地模式：由本地 UI 面板驱动
                    current_inputs = {
                        k: PitchControlInputs(
                            u_prop=self._inputs[k].u_prop,
                            u_enable=self._inputs[k].u_enable,
                            u_safe_feather=self._inputs[k].u_safe_feather,
                            f_aero=self._inputs[k].f_aero,
                            u_valve_130=self._inputs[k].u_valve_130,
                            u_valve_132=self._inputs[k].u_valve_132
                        )
                        for k in self.blades
                    }

            # 2. 细分子步积分 (保证高刚性液压连续性方程的数值稳定)
            step_outputs = {}
            for _ in range(self.sub_steps):
                # 集中 HPU 迟滞打压/卸荷逻辑 (以3个机构蓄能器压力为触发判定)
                # 规则：任何一个 107 小于 235 bar 开始打压；三个都大于 265 bar 停止打压
                p_acc_bars = {k: self.plants[k].P_acc / 1.0e5 for k in self.blades}
                p_min_acc = min(p_acc_bars.values())
                p_max_acc = max(p_acc_bars.values())

                if p_min_acc < 235.0:
                    self.valve_25_state = True
                elif p_min_acc > 265.0:
                    self.valve_25_state = False

                if self.valve_25_state:
                    # 打压时供油压力高于蓄能器母管压力 (单向阀开启压差约 2 bar)
                    p_supply_calc = min(self.P_set_hpu_relief / 1.0e5, p_max_acc + 2.0)
                    self.P_supply = p_supply_calc * 1.0e5
                    self.Q_hpu_actual = self.Q_pump_total
                    # 计算需要补油受压的机构面 (低于 265.0 bar 者受油)
                    charging_blades = [k for k in self.blades if p_acc_bars[k] < 265.2]
                    if not charging_blades:
                        charging_blades = list(self.blades)
                    q_share = self.Q_pump_total / len(charging_blades)
                else:
                    self.P_supply = self.P_tank + 0.5e5
                    self.Q_hpu_actual = 0.0
                    charging_blades = []
                    q_share = 0.0

                # 分别推进 A、B、C 三机构动力学
                for k in self.blades:
                    plant = self.plants[k]
                    inp = current_inputs[k]
                    q_in_k = q_share if (self.valve_25_state and k in charging_blades) else 0.0
                    x, v, p_acc, p_A, p_B, valve_pos = plant.step(
                        u_prop_voltage=inp.u_prop,
                        u_enable=inp.u_enable,
                        u_safe_feather=inp.u_safe_feather,
                        f_aero=inp.f_aero,
                        u_valve_130=inp.u_valve_130,
                        u_valve_132=inp.u_valve_132,
                        valve_25_state=self.valve_25_state,
                        q_pump_in=q_in_k
                    )
                    plant.P_supply = self.P_supply
                    step_outputs[k] = (x, v, p_acc, p_A, p_B, valve_pos)

            self.sim_time += self.cycle_time

            # 3. 统计实时性指标
            now = time.perf_counter()
            cycle_calc_ms = (now - t_start) * 1000.0
            actual_period = t_start - last_t
            jitter_ms = abs(actual_period - self.cycle_time) * 1000.0
            last_t = t_start

            with self._lock:
                for k in self.blades:
                    x, v, p_acc, p_A, p_B, valve_pos = step_outputs[k]
                    plant = self.plants[k]
                    self._telemetries[k] = PitchTelemetryData(
                        timestamp=self.sim_time,
                        displacement_mm=x * 1000.0,
                        velocity_mps=v,
                        p_acc_bar=p_acc / 1.0e5,
                        p_A_bar=p_A / 1.0e5,
                        p_B_bar=p_B / 1.0e5,
                        valve_position_pct=valve_pos * 100.0,
                        cycle_time_ms=cycle_calc_ms,
                        jitter_ms=jitter_ms,
                        overruns=self.overruns,
                        lock_pin_mm=plant.x_lock * 1000.0,
                        is_locked=plant.is_locked,
                        switch_129=plant.switch_129,
                        switch_S_unlocked=plant.switch_S_unlocked,
                        switch_S_locked=plant.switch_S_locked,
                        p_PP_bar=plant.P_PP / 1.0e5,
                        valve_25_state=self.valve_25_state,
                        p_supply_bar=self.P_supply / 1.0e5,
                        q_hpu_lpm=self.Q_hpu_actual * 60000.0,
                        motor_rpm=self.n_motor_rpm
                    )

            # 4. 高精度软实时时钟对齐
            target_next_time += self.cycle_time
            now = time.perf_counter()

            if now < target_next_time:
                rem = target_next_time - now
                if rem > 0.0015:
                    time.sleep(rem - 0.001)
                while time.perf_counter() < target_next_time:
                    pass
            else:
                self.overruns += 1
                target_next_time = now

        if sys.platform == "win32":
            try:
                import ctypes
                ctypes.windll.winmm.timeEndPeriod(1)
            except Exception:
                pass

    def start(self):
        if not self._running:
            self._running = True
            self.overruns = 0
            self.sim_time = 0.0
            self._thread = threading.Thread(target=self._worker, daemon=True)
            self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)

    # -------------------------------------------------------------------------
    # 控制权限模式接口: 本地控制 (LOCAL) / 远程控制 (REMOTE)
    # -------------------------------------------------------------------------
    def set_control_mode(self, mode: str):
        """设置控制权模式: 'LOCAL' 或 'REMOTE'"""
        with self._lock:
            self.control_mode = 'REMOTE' if mode.upper().startswith('REM') else 'LOCAL'

    def get_control_mode(self) -> str:
        with self._lock:
            return self.control_mode

    # -------------------------------------------------------------------------
    # 远程控制输入接口 (Remote Control Input Interface)
    # 输入: 每个电磁阀的指令 + 比例阀电压
    # -------------------------------------------------------------------------
    def send_remote_command(self, blade='A', prop_voltage=0.0,
                            valve_109=True, valve_119A=True, valve_119B=True,
                            valve_120=False, valve_103=False, valve_116=True,
                            valve_130=True, valve_132=True):
        """
        下发远程控制指令给指定叶片或全部叶片 ('ALL')
        :param blade: 'A', 'B', 'C' 或 'ALL'
        :param prop_voltage: 比例伺服阀 102A 输入电压 (-10.0V ~ +10.0V)
        :param valve_109: 109 主供油使能电磁座阀 (True=通电开)
        :param valve_119A: 119A A腔安全隔离切断座阀 (True=通电开)
        :param valve_119B: 119B A腔安全隔离切断座阀 (True=通电开)
        :param valve_120: 120 顺桨快速充液阀 (True=保压闭合, False=失电常开快速顺桨)
        :param valve_103: 103 顺桨快速充液阀 (True=保压闭合, False=失电常开快速顺桨)
        :param valve_116: 116 B腔顺桨泄荷阀 (True=保压闭合, False=失电常开排油)
        :param valve_130: 130 变桨锁充压解锁阀 (True=得电充压解锁, False=断电)
        :param valve_132: 132 变桨锁泄荷自锁阀 (True=得电保压, False=断电常开自锁)
        """
        target_blades = self.blades if (blade == 'ALL' or blade is None) else [blade]
        with self._lock:
            for b in target_blades:
                self._remote_commands[b] = RemoteValveCommand(
                    prop_voltage=float(prop_voltage),
                    valve_109=bool(valve_109),
                    valve_119A=bool(valve_119A),
                    valve_119B=bool(valve_119B),
                    valve_120=bool(valve_120),
                    valve_103=bool(valve_103),
                    valve_116=bool(valve_116),
                    valve_130=bool(valve_130),
                    valve_132=bool(valve_132)
                )

    # -------------------------------------------------------------------------
    # 远程状态输出接口 (Remote Status Output Interface)
    # 输出: 油缸位移、107压力、变桨锁状态
    # -------------------------------------------------------------------------
    def get_remote_feedback(self, blade=None) -> Union[RemoteSystemFeedback, dict]:
        """
        获取远程遥测反馈
        :param blade: 若为 None，返回包含全机组数据的 RemoteSystemFeedback；
                      若为 'A'/'B'/'C'，返回对应叶片的独立精简字典
        """
        with self._lock:
            # 107 压力读取
            p_107 = max(self.plants[k].P_acc for k in self.blades) / 1.0e5
            blade_fbs = {}
            for k in self.blades:
                plant = self.plants[k]
                blade_fbs[k] = RemoteBladeFeedback(
                    displacement_mm=plant.x * 1000.0,
                    velocity_mmps=plant.v * 1000.0,
                    velocity_mps=plant.v,
                    valve_opening_volt=plant.valve_pos * 10.0,
                    valve_opening_pct=plant.valve_pos * 100.0,
                    valve_spool_pos=plant.valve_pos * 10.0,
                    is_locked=plant.is_locked,
                    lock_pin_mm=plant.x_lock * 1000.0,
                    switch_129=plant.switch_129,
                    switch_S_unlocked=plant.switch_S_unlocked,
                    switch_S_locked=plant.switch_S_locked,
                    pressure_107_bar=plant.P_acc / 1.0e5,
                    p_A_bar=plant.P_A / 1.0e5,
                    p_B_bar=plant.P_B / 1.0e5
                )

            if blade is not None and blade in self.blades:
                bf = blade_fbs[blade]
                return {
                    'displacement_mm': bf.displacement_mm,
                    'velocity_mmps': bf.velocity_mmps,
                    'velocity_mps': bf.velocity_mps,
                    'valve_opening_volt': bf.valve_opening_volt,
                    'valve_opening_pct': bf.valve_opening_pct,
                    'valve_spool_pos': bf.valve_spool_pos,
                    'pressure_107_bar': bf.pressure_107_bar,
                    'is_locked': bf.is_locked,
                    'lock_pin_mm': bf.lock_pin_mm,
                    'switch_S_unlocked': bf.switch_S_unlocked,
                    'switch_S_locked': bf.switch_S_locked,
                    'switch_129': bf.switch_129,
                    'p_A_bar': bf.p_A_bar,
                    'p_B_bar': bf.p_B_bar,
                    'control_mode': self.control_mode
                }

            return RemoteSystemFeedback(
                control_mode=self.control_mode,
                pressure_107_bar=p_107,
                pressure_107_bar_A=blade_fbs['A'].pressure_107_bar if 'A' in blade_fbs else p_107,
                pressure_107_bar_B=blade_fbs['B'].pressure_107_bar if 'B' in blade_fbs else p_107,
                pressure_107_bar_C=blade_fbs['C'].pressure_107_bar if 'C' in blade_fbs else p_107,
                p_supply_bar=self.P_supply / 1.0e5,
                valve_25_state=self.valve_25_state,
                q_hpu_lpm=self.Q_hpu_actual * 60000.0,
                motor_rpm=self.n_motor_rpm,
                timestamp=self.sim_time,
                blades=blade_fbs
            )

    # -------------------------------------------------------------------------
    # 本地控制输入接口 (本地 UI 交互调用)
    # -------------------------------------------------------------------------
    def set_inputs(self, blade='A', u_prop=None, u_enable=None, u_safe_feather=None, f_aero=None,
                   u_valve_130=None, u_valve_132=None):
        target_blades = self.blades if (blade == 'ALL' or blade is None) else [blade]
        with self._lock:
            for b in target_blades:
                inp = self._inputs[b]
                if u_prop is not None:
                    inp.u_prop = float(u_prop)
                if u_enable is not None:
                    inp.u_enable = float(u_enable)
                if u_safe_feather is not None:
                    inp.u_safe_feather = float(u_safe_feather)
                if f_aero is not None:
                    inp.f_aero = float(f_aero)
                if u_valve_130 is not None:
                    inp.u_valve_130 = float(u_valve_130)
                if u_valve_132 is not None:
                    inp.u_valve_132 = float(u_valve_132)

    def set_feather_orifices(self, d_121_mm, d_122_mm, blade='ALL'):
        target_blades = self.blades if (blade == 'ALL' or blade is None) else [blade]
        with self._lock:
            for b in target_blades:
                self.plants[b].set_feather_orifices(d_121_mm, d_122_mm)

    def get_telemetry(self, blade='A') -> PitchTelemetryData:
        with self._lock:
            return self._telemetries[blade]

    def get_all_telemetry(self) -> dict:
        with self._lock:
            return dict(self._telemetries)
