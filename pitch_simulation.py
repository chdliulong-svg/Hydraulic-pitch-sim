"""
=============================================================================
兆瓦级风电机组变桨系统
液压物理动力学高精度仿真模型
=============================================================================
元件清单对照:
  - 动力源: 进油口 P (高压源 235~265 bar，受25#旁路卸荷电磁阀闭环控制), 回油口 T (低压回油 2 bar)
  - 124 & 125: 双并联变桨液压缸 (200/125 x 822 mm, 缸径 200mm, 杆径 125mm, 行程 822mm)
  - 106: 蓄能器组 (8只 25L 并联 = 200L, 预充氮气压力 100 bar)
  - 101: 系统主安全溢流阀 (设定值 280 bar)
  - 102A: 4/3位比例伺服方向阀 (-10V ~ +10V; 伸出位P->A且B与T内部封堵，强制走104构成差动回路)
  - 109: 主供油使能电磁座阀 (得电后接通主压力油进入 102A 的 P 口)
  - 119A / 119B: A腔双冗余安全隔离切断电磁座阀 (并联布置在A腔进油母线上)
  - 120 / 103: 紧急顺桨快速充液电磁阀 (失电常开快速灌充A腔)
  - 121: 原图进油节流孔 (直径 Ø4.7 mm) + 110: 充液单向阀
  - 116: B腔顺桨泄荷电磁阀 (正常得电闭合保压，失电常开排油) + 122: 原图阻尼排油节流孔 (直径 Ø3.0 mm)
  - 104: B腔单向阀 (正常调桨伸出时构成差动回路 / 突风外力下拉动活塞时的气动超压泄放)
  - 123: A腔抗冲击缓冲溢流阀 (设定值 280 bar)
  - 118: P口供油单向阀 (防止母线高压向泵站倒流)
  - 130 / 131 / 132: 变桨机械插销锁控制回路 (40/20x60 插销缸 + Ø0.8mm节流孔 131)
=============================================================================
"""

import math
import numpy as np


class PitchHydraulicSystem:
    def __init__(self, dt=0.001):
        self.dt = dt

        # 1. 油源与液压站 (HPU) 动力源参数
        # 3个油泵合并供油, 每个泵排量 15.6 ml/r, 电机转速 1450 rpm (常通电运行)
        self.num_pumps = 3
        self.V_disp_pump = 15.6e-6    # 单泵排量 15.6 ml/r (m^3/rev)
        self.n_motor_rpm = 1450.0     # 电机转速 1450 rpm (常通电运转，不启闭电机)
        self.eta_v = 0.95             # 容积效率
        # 3泵理论合并有效供油流量: 3 * 15.6e-6 * (1450 / 60) * 0.95 = 1.074e-3 m3/s (~64.5 L/min)
        self.Q_pump_total = self.num_pumps * self.V_disp_pump * (self.n_motor_rpm / 60.0) * self.eta_v

        # 液压站回路溢流阀 (设定 280 bar)
        self.P_set_hpu_relief = 280.0e5
        self.K_rv_hpu = 2.0e-10

        # 25# 旁路二位二通常闭电磁阀与打压/卸荷迟滞控制 (以 107 压力为反馈)
        self.P_load_on = 235.0e5      # 当 P_107 < 235 bar 时 25# 得电开始打压
        self.P_load_off = 265.0e5     # 当 P_107 > 265 bar 时 25# 失电停止建压 (旁路卸荷)
        self.valve_25_state = False   # 25# 电磁阀当前状态 (True = 得电打压, False = 失电卸荷)
        self.Q_hpu_actual = 0.0       # 实际供往高压管路流量 (m^3/s)

        self.P_supply = 260.0e5       # P口供油管路压力 (Pa)
        self.P_tank = 2.0e5           # T口回油背压 2 bar
        self.rho = 870.0              # 46号抗磨液压油密度 (kg/m3)
        self.beta_oil = 1.2e9         # 液压油有效体积弹性模量 1200 MPa

        # 2. 蓄能器组 (106: 8 x 25L = 200L, 预充 100 bar)
        self.V0_acc = 8 * 0.025       # 总容积 0.200 m3 (200 L)
        self.p0_acc = 100.0e5         # 预充氮气压力 100 bar (10.0 MPa)
        self.kappa = 1.4              # 绝热气体指数
        self.V_rail = 0.015           # 阀块与主管路有效死区容积 (15 L)

        # 3. 节流与安全元件 (121: 充液孔, 122: 顺桨快放调速孔, 101/123: 280 bar)
        self.Cd = 0.65
        # 默认匹配原厂原理图标准设计尺寸
        self.d_121 = 0.0047           # 原图进油节流孔 121 (Ø4.7mm)
        self.A_121 = math.pi * (self.d_121**2) / 4.0

        self.d_122 = 0.0030           # 原图阻尼排油节流孔 122 (Ø3.0mm)
        self.A_122 = math.pi * (self.d_122**2) / 4.0

        self.A_118 = 4.0e-4           # 单向阀 118 全开截面
        self.A_104 = 3.0e-4           # 差动单向阀 104 截面
        self.P_set_101 = 280.0e5      # 主溢流阀 280 bar
        self.K_rv_101 = 2.0e-10       # 溢流刚度 m^3/(s*Pa)
        self.P_set_123 = 280.0e5      # A腔缓冲阀 280 bar
        self.K_rv_123 = 1.5e-10

        # 4. 比例方向阀 (102A) 参数 (实际规格: 压差 7 bar 时额定流量 75 L/min)
        self.dp_rated_102 = 7.0e5     # 额定测试压差 7 bar (Pa)
        self.Q_rated_102 = 75.0e-3 / 60.0 # 额定流量 75 L/min (m^3/s)
        # 根据 Q = Cd * A_max * sqrt(2 * dp_rated / rho):
        self.A_102_max = self.Q_rated_102 / (self.Cd * math.sqrt(2.0 * self.dp_rated_102 / self.rho))
        self.tau_valve = 0.015        # 阀芯响应时间常数 15ms
        self.valve_pos = 0.0          # 当前阀芯开度 [-1.0, 1.0]

        # 5. 双变桨油缸 124 & 125 (并联驱动同一叶片, 200/125 x 822)
        self.stroke_max = 0.822       # 最大行程 822 mm
        self.D_piston = 0.200         # 缸径 200 mm
        self.d_rod = 0.125            # 杆径 125 mm
        
        # 单缸面积
        A_A_single = math.pi * (self.D_piston**2) / 4.0
        A_B_single = math.pi * (self.D_piston**2 - self.d_rod**2) / 4.0
        # 双缸并联总有效受压面积
        self.A_A = 2.0 * A_A_single   # 双缸无杆腔总面积 (~ 0.0628 m2)
        self.A_B = 2.0 * A_B_single   # 双缸有杆腔总面积 (~ 0.0383 m2)

        self.V0_A = self.A_A * 0.015  # A腔初始/管路死区容积
        self.V0_B = self.A_B * 0.015  # B腔初始/管路死区容积

        # 6. 机械结构与折算负载
        self.m_load = 8500.0          # 活塞杆 + 双缸连杆 + 叶片折算质量 (8.5 吨)
        self.b_damp = 45000.0         # 机械与液压阻尼 N/(m/s)

        # 7. 系统当前状态
        self.x = 0.050                # 当前位移 (m), 初始 50mm
        self.v = 0.0                  # 速度 (m/s)
        self.P_acc = 260.0e5          # 蓄能器与高压母管压力 (Pa), 初始 260 bar
        self.P_A = 15.0e5             # 无杆腔压力 (Pa)
        self.P_B = 15.0e5             # 有杆腔压力 (Pa)

        # 8. 变桨机械锁紧装置 (40/20x60 油缸 + 弹簧自锁机构)
        self.D_lock = 0.040           # 缸径 40 mm
        self.d_lock_rod = 0.020       # 杆径 20 mm
        self.stroke_lock_max = 0.060  # 销行程 60 mm
        self.A_lock_ring = math.pi * (self.D_lock**2 - self.d_lock_rod**2) / 4.0 # 承压环形面积 (~9.42e-4 m2)
        self.m_lock_pin = 12.0        # 锁销质量 12 kg
        self.b_lock_damp = 1500.0     # 导向阻尼 N/(m/s)
        self.F_lock_spring0 = 2200.0  # 弹簧初始预紧力 2200 N (失压时强力自锁插销)
        self.k_lock_spring = 35000.0  # 弹簧刚度 35 N/mm

        # 锁控制元件 (130: 常闭NC, 131: Φ0.8mm, 132: 常开NO)
        self.d_131 = 0.0008           # 0.8 mm 充液节流孔
        self.A_131 = math.pi * (self.d_131**2) / 4.0
        self.A_132_max = 1.8e-4       # 132 泄油阀通流面积

        # 锁状态变量
        self.x_lock = 0.0             # 0.0 = 完全缩回拔销(解锁), 0.060 = 完全伸出插销(锁死)
        self.v_lock = 0.0
        self.P_PP = 260.0e5           # PP 腔压力 (初始充压解锁)
        self.is_locked = False        # 机械锁定标志位 (True = 锁死主油缸)
        self.switch_129 = False       # 90° 顺桨对准开关 (主缸接近 822mm 视为对准锁孔)
        self.switch_S_unlocked = True # 接近开关S: 拔销到位
        self.switch_S_locked = False  # 接近开关S: 插销到位

    def set_feather_orifices(self, d_121_mm, d_122_mm):
        """动态配置顺桨充油孔 121 与快放调速孔 122 直径 (mm)"""
        self.d_121 = float(d_121_mm) / 1000.0
        self.A_121 = math.pi * (self.d_121**2) / 4.0
        self.d_122 = float(d_122_mm) / 1000.0
        self.A_122 = math.pi * (self.d_122**2) / 4.0

    def _orifice_flow(self, area, delta_p):
        """标准孔口节流流量公式"""
        if area <= 0.0:
            return 0.0
        if delta_p > 0:
            return self.Cd * area * math.sqrt(2.0 * delta_p / self.rho)
        else:
            return -self.Cd * area * math.sqrt(2.0 * abs(delta_p) / self.rho)

    def step(self, u_prop_voltage, u_enable=1.0, u_safe_feather=0.0, f_aero=0.0,
             u_valve_130=1.0, u_valve_132=1.0, valve_25_state=None, q_pump_in=None):
        """
        单步动力学求解 (微分方程前向积分)
        :param u_prop_voltage: 比例阀 102A 输入电压 [-10.0V, +10.0V]
        :param u_enable: 主使能阀 109 与隔离阀 119A/B (1.0 = 开启导通, 0.0 = 关闭隔离)
        :param u_safe_feather: 应急顺桨阀 120/103/116 状态 (0.0 = 正常闭合, 1.0 = 触发紧急顺桨)
        :param f_aero: 外部气动阻力/风载 (N)
        :param u_valve_130: 锁紧充压电磁阀 130 (1.0 = 得电导通高压供油 P->PP, 0.0 = 断电关闭)
        :param u_valve_132: 锁紧泄荷电磁阀 132 (1.0 = 得电关闭泄油, 0.0 = 断电常开连通油箱 PP->T)
        :param valve_25_state: 外部传入的 25# 电磁阀打压/卸荷状态 (为 None 时使用内部单机构逻辑)
        :param q_pump_in: 外部传入的分配至该支路的实际打压供油流量 (m^3/s)
        :return: (位移x, 速度v, P_acc, P_A, P_B, 阀开度)
        """
        # 1. 液压站 25# 旁路电磁阀打压/卸荷迟滞控制 (传感器 107 监测母管蓄能器压力)
        if valve_25_state is not None:
            self.valve_25_state = bool(valve_25_state)
        else:
            p_107_bar = self.P_acc / 1.0e5
            if p_107_bar < (self.P_load_on / 1.0e5):
                self.valve_25_state = True
            elif p_107_bar > (self.P_load_off / 1.0e5):
                self.valve_25_state = False

        # 2. 液压站供油流量与 P 口压力求解
        if self.valve_25_state:
            # 25# 得电打压: 经单向阀 118 压入蓄能器母管
            q_pump_avail = self.Q_pump_total if q_pump_in is None else float(q_pump_in)
            self.P_supply = min(self.P_set_hpu_relief, self.P_acc + 1.0e5)
            # 液压站回路溢流阀 (280 bar) 泄压保护
            q_rv_hpu = self.K_rv_hpu * max(0.0, self.P_supply - self.P_set_hpu_relief)
            Q_118 = max(0.0, q_pump_avail - q_rv_hpu)
            self.Q_hpu_actual = Q_118
        else:
            # 25# 失电停止建压: 泵流量直接经旁路回油箱卸荷
            self.P_supply = self.P_tank + 0.5e5  # 卸荷管路背压约 2.5 bar
            Q_118 = 0.0                          # 单向阀 118 承受反向压差截止
            self.Q_hpu_actual = 0.0

        # 3. 比例阀阀芯动态
        u_norm = max(-1.0, min(1.0, u_prop_voltage / 10.0))
        self.valve_pos += (u_norm - self.valve_pos) * (self.dt / self.tau_valve)

        # 4. 主溢流阀 101 泄油 (280 bar)
        Q_rv101 = self.K_rv_101 * max(0.0, self.P_acc - self.P_set_101)

        # 4. 比例方向阀 102A 主油路流量 (受使能阀 109 和隔离阀 119A/B 串联控制)
        if u_enable > 0.5:
            spool_area = abs(self.valve_pos) * self.A_102_max
            if self.valve_pos > 0.001:   # 正向伸出(顺桨): P->A, B与T在102A内部封死(T-stop)
                Q_102_A = self._orifice_flow(spool_area, self.P_acc - self.P_A)
                Q_102_B = 0.0            # 阀芯封死 B 口与 T 口
            elif self.valve_pos < -0.001:# 反向缩回(开桨): P->B, A->T (全推力缩回，非差动)
                Q_102_A = -self._orifice_flow(spool_area, self.P_A - self.P_tank)
                Q_102_B = self._orifice_flow(spool_area, self.P_acc - self.P_B)
            else:
                Q_102_A = 0.0
                Q_102_B = 0.0
        else:
            Q_102_A = 0.0
            Q_102_B = 0.0

        # 5. 快速顺桨支路 (电磁阀120/103 + 节流孔121[4.7mm] + 单向阀110)
        # 紧急顺桨时高压油直接灌入 A 腔
        if u_safe_feather > 0.5 and self.P_acc > self.P_A:
            Q_fast_A = self._orifice_flow(self.A_121, self.P_acc - self.P_A)
        else:
            Q_fast_A = 0.0

        # 6. B腔顺桨泄荷支路 (电磁阀116 + 节流孔122[3.0mm] -> T口)
        # 正常调桨时 116 得电闭合(保压); 紧急顺桨时 116 失电常开泄油
        if u_safe_feather > 0.5 and self.P_B > self.P_tank:
            Q_dump_B = self._orifice_flow(self.A_122, self.P_B - self.P_tank)
        else:
            Q_dump_B = 0.0

        # 7. B腔单向阀 104 (差动回路核心 / 气动超压泄放)
        # 导通方向：从 B 腔 -> 109 阀后 P 口供油母线
        # 当正常调桨油缸伸出时，116闭合、102A的B口封死，B腔油液受压推开104，回流至P母线合流灌入A腔(构成差动回路)
        if self.P_B > self.P_acc:
            Q_diff_104 = self._orifice_flow(self.A_104, self.P_B - self.P_acc)
        else:
            Q_diff_104 = 0.0

        # 8. A腔安全溢流阀 123 (280 bar)
        Q_rv123 = self.K_rv_123 * max(0.0, self.P_A - self.P_set_123)

        # 9. 状态微分方程计算
        # (a) 蓄能器母线压力
        # 统计从主母线流出的净流量 (差动回流 Q_diff_104 减小了母线对外界油源的消耗)
        q_out_rail = Q_fast_A
        if Q_102_A > 0:
            q_out_rail += Q_102_A
        if Q_102_B > 0:
            q_out_rail += Q_102_B
        q_out_rail -= Q_diff_104

        p_clamped = max(self.P_acc, self.p0_acc * 0.5)
        v_gas = self.V0_acc * (self.p0_acc / p_clamped) ** (1.0 / self.kappa)
        c_acc = v_gas / (self.kappa * p_clamped)
        c_eff = c_acc + (self.V_rail / self.beta_oil)
        
        dP_acc = ((Q_118 - Q_rv101 - q_out_rail) / c_eff) * self.dt
        self.P_acc = max(self.p0_acc, self.P_acc + dP_acc)

        # (b) 双缸 A 腔压力 (无杆腔)
        x_clamped = max(0.0, min(self.stroke_max, self.x))
        V_A = self.V0_A + self.A_A * x_clamped
        q_net_A = Q_102_A + Q_fast_A - Q_rv123
        dP_A = ((self.beta_oil / V_A) * (q_net_A - self.A_A * self.v)) * self.dt
        self.P_A = max(0.0, min(300.0e5, self.P_A + dP_A))

        # (c) 双缸 B 腔压力 (有杆腔)
        # 流入 B 腔为正: 比例阀供油 Q_102_B - 差动排油 Q_diff_104 - 顺桨泄油 Q_dump_B
        V_B = self.V0_B + self.A_B * (self.stroke_max - x_clamped)
        q_net_B = Q_102_B - Q_diff_104 - Q_dump_B
        dP_B = ((self.beta_oil / V_B) * (q_net_B + self.A_B * self.v)) * self.dt
        self.P_B = max(0.0, min(300.0e5, self.P_B + dP_B))

        # (d) 变桨锁紧装置 40/20x60 流体力学与弹簧动作求解
        # 变桨锁充压高压油源来自蓄能器主高压母管 (P_acc / 118单向阀后高压回路)
        P_lock_source = self.P_acc
        # 130 为常闭阀 (u_valve_130 > 0.5 导通高压供油 -> PP)
        if u_valve_130 > 0.5:
            Q_lock_in = self._orifice_flow(self.A_131, P_lock_source - self.P_PP)
        else:
            Q_lock_in = 0.0

        # 132 为常开阀 (u_valve_132 > 0.5 关闭泄油; u_valve_132 < 0.5 导通油箱 PP -> T)
        if u_valve_132 < 0.5:
            Q_lock_dump = self._orifice_flow(self.A_132_max, self.P_PP - self.P_tank)
        else:
            Q_lock_dump = 0.0

        q_net_PP = Q_lock_in - Q_lock_dump
        V_PP = 0.00005 + self.A_lock_ring * (self.stroke_lock_max - self.x_lock)
        dP_PP = ((self.beta_oil / V_PP) * (q_net_PP + self.A_lock_ring * self.v_lock)) * self.dt
        self.P_PP = max(self.P_tank, min(P_lock_source, self.P_PP + dP_PP))

        # 锁销受力: 弹簧推力促使插销 (x_lock -> 0.060), PP腔液压力促使拔销 (x_lock -> 0.0)
        f_lock_spring = self.F_lock_spring0 + self.k_lock_spring * (self.stroke_lock_max - self.x_lock)
        f_lock_hyd = self.P_PP * self.A_lock_ring
        f_net_lock = f_lock_spring - f_lock_hyd - self.b_lock_damp * self.v_lock

        # 对准传感器 129 判定: 主油缸在 90° 羽化位置 (808mm~822mm) 时孔轴精确对准
        self.switch_129 = (self.x >= 0.808)

        # 锁紧销运动学积分
        if not self.switch_129 and self.x_lock <= 0.005 and f_net_lock > 0:
            # 未对准顺桨锁孔时，销轴顶在实体盘端面上，受阻无法插销
            acc_lock = 0.0
            self.v_lock = 0.0
        else:
            if (self.x_lock <= 0.0 and f_net_lock < 0) or (self.x_lock >= self.stroke_lock_max and f_net_lock > 0):
                acc_lock = 0.0
                self.v_lock = 0.0
            else:
                acc_lock = f_net_lock / self.m_lock_pin

        self.v_lock += acc_lock * self.dt
        self.x_lock += self.v_lock * self.dt
        self.x_lock = max(0.0, min(self.stroke_lock_max, self.x_lock))

        # 接近开关 S 与机械锁死标志判定
        self.switch_S_unlocked = (self.x_lock <= 0.008)  # 完全缩回，拔销解锁
        self.switch_S_locked = (self.x_lock >= 0.050 and self.switch_129) # 插销到位锁死
        self.is_locked = self.switch_S_locked

        # (e) 双缸机械运动方程 (与变桨锁进行物理互锁)
        if self.is_locked:
            # 变桨锁插入：主油缸被机械刚性锁死在 822mm，速度与加速度强制清零
            accel = 0.0
            self.v = 0.0
            self.x = self.stroke_max
        else:
            f_hydraulic = (self.P_A * self.A_A) - (self.P_B * self.A_B)
            f_damping = self.b_damp * self.v
            f_net = f_hydraulic - f_damping + f_aero

            # 机械行程两端硬止挡保护
            if (self.x <= 0.0 and f_net < 0) or (self.x >= self.stroke_max and f_net > 0):
                accel = 0.0
                self.v = 0.0
            else:
                accel = f_net / self.m_load

            self.v += accel * self.dt
            self.x += self.v * self.dt

            if self.x < 0.0:
                self.x = 0.0
                self.v = max(0.0, self.v)
            elif self.x > self.stroke_max:
                self.x = self.stroke_max
                self.v = min(0.0, self.v)

        return self.x, self.v, self.P_acc, self.P_A, self.P_B, self.valve_pos
