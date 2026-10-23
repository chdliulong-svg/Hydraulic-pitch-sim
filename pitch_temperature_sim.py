"""
=============================================================================
6MW 风电机组轮毂与液压系统热力学/温度动态仿真模型
=============================================================================
功能特征:
1. 液压油温动态方程:
   - 供油泵打压 (25#阀得电) 满载做功节流产热
   - 变桨双缸伺服运动比例阀微调做功产热
   - 停机/卸荷自然冷却对流向环境散热
   - 辅助风冷/水冷器与电加热器动态
2. 轮毂主控制柜温度动态方程:
   - PAC PLC / 电源 / 驱动器内部发热
   - 柜顶散热风扇 (cabinetFanConnected) 强制排风对流
3. 控制器 CPU 核心温度动态方程:
   - 运算负载发热 (稳态高于柜温 12~15°C)
4. 发电机气隙、主轴承/变桨轴承温度动态:
   - 轴承摩擦与润滑状态耦合 (润滑充分时温升低，缺油时温升快)
5. 故障注入模式:
   - 极低温工况 (-30°C ~ -15°C) 触发 oilTempBelowLim / hubTempBelowLim
   - 高温报警工况 (> 70°C) 触发 oilTempAboveLim / PACTempAboveLim
   - 散热风扇故障停机
   - PT100 传感器断线 / 开路故障 (Open Line)
=============================================================================
"""

import math
from dataclasses import dataclass


@dataclass
class TemperatureTelemetry:
    # 模拟量温度 (°C)
    temp_oil_deg: float = 42.5          # 液压站油箱油温 (工作区间 15 ~ 65°C)
    temp_hub_deg: float = 25.0          # 轮毂内部环境温度 (-30 ~ +50°C)
    temp_cabinet_deg: float = 32.0      # PAC 变桨电气柜内部温度 (-20 ~ +60°C)
    temp_airgap_deg: float = 38.0       # 发电机气隙温度
    temp_bearing_deg: float = 35.0      # 变桨轴承 / 主轴承温度
    temp_cpu_deg: float = 45.0          # PAC 控制器 CPU 核心温度

    # 开关量状态
    cabinet_fan: bool = True            # 柜顶散热风扇运行状态 (pac.io.dIn.cabinetFanConnected)
    oil_heater: bool = False            # 油箱辅助电加热器启动状态
    oil_cooler: bool = False            # 油箱辅助冷却器启动状态

    # 告警标志
    alarm_oil_high: bool = False        # 油温超上限告警 (> 70°C)
    alarm_oil_low: bool = False         # 油温超下限告警 (< -20°C)
    alarm_cab_high: bool = False        # 柜温超上限告警 (> 60°C)
    alarm_cab_low: bool = False         # 柜温超下限告警 (< -10°C)
    alarm_hub_high: bool = False        # 轮毂超上限告警 (> 55°C)
    alarm_hub_low: bool = False         # 轮毂超下限告警 (< -25°C)
    alarm_cpu_high: bool = False        # CPU 超上限告警 (> 80°C)
    sensor_fault_open: bool = False     # PT100 传感器断线故障


class TemperatureSystemSim:
    def __init__(self, dt: float = 0.010):
        """
        :param dt: 仿真计算步长 (秒)
        """
        self.dt = dt

        # 环境设定值 (°C)
        self.temp_ambient = 25.0

        # 当前温度状态 (°C)
        self.t_oil = 42.5
        self.t_hub = 25.0
        self.t_cab = 32.0
        self.t_airgap = 38.0
        self.t_bearing = 35.0
        self.t_cpu = 45.0

        # 热容量与时间常数 (分钟级热惯性在仿真中适度加速以方便调试观察，时间尺度约 1:20)
        self.tau_oil_heat = 35.0        # 打压满载升温时间常数 (s)
        self.tau_oil_cool = 60.0        # 卸荷自然散热时间常数 (s)
        self.tau_cab_heat = 25.0        # 柜内蓄热时间常数 (s)
        self.tau_cab_cool = 15.0        # 风扇开启排热时间常数 (s)
        self.tau_bearing = 40.0         # 轴承温升时间常数 (s)

        # 辅助温控设备状态
        self.cabinet_fan = True
        self.oil_heater = False
        self.oil_cooler = False

        # 故障注入开关
        self.fault_oil_overheat = False     # 强制油温过高
        self.fault_extreme_cold = False     # 强制极低寒冷环境
        self.fault_fan_fail = False         # 强制风扇损坏
        self.fault_pt100_open = False       # 模拟 PT100 断线开路

    def set_ambient_temperature(self, temp_deg: float):
        """设置外部环境温度 (-40°C ~ +50°C)"""
        self.temp_ambient = float(temp_deg)

    def set_faults(self, oil_overheat=None, extreme_cold=None, fan_fail=None, pt100_open=None):
        """故障注入设置"""
        if oil_overheat is not None:
            self.fault_oil_overheat = bool(oil_overheat)
        if extreme_cold is not None:
            self.fault_extreme_cold = bool(extreme_cold)
        if fan_fail is not None:
            self.fault_fan_fail = bool(fan_fail)
        if pt100_open is not None:
            self.fault_pt100_open = bool(pt100_open)

    def step(self, valve_25_charging: bool = False,
             cyl_motion_intensity: float = 0.0,
             lub_pumps_on: bool = False) -> TemperatureTelemetry:
        """
        推进单步热力学动态
        :param valve_25_charging: 液压站 25# 电磁阀是否得电打压建压 (True=高压满载 260bar)
        :param cyl_motion_intensity: 变桨油缸伺服运动剧烈程度 [0.0, 1.0]
        :param lub_pumps_on: 润滑系统泵是否运行
        """
        amb = -28.0 if self.fault_extreme_cold else self.temp_ambient

        # 1. 轮毂内部环境温度 (微滞后于外部环境)
        target_hub = amb + (2.0 if not self.fault_extreme_cold else 0.0)
        self.t_hub += (target_hub - self.t_hub) * (self.dt / 10.0)

        # 2. 液压油温热力学模型
        if self.fault_oil_overheat:
            # 注入油温过高故障，直奔 76°C
            self.t_oil += (76.5 - self.t_oil) * (self.dt / 5.0)
        elif self.fault_extreme_cold:
            # 极低冷态，直奔 -24°C
            self.t_oil += (-24.0 - self.t_oil) * (self.dt / 8.0)
        else:
            # 正常动力学升温:
            # 打压产热: 64.5 L/min @ 260 bar 经溢流/单向阀节流，发热功率较大
            # 伺服动作节流产热: 比例阀压降热损耗
            p_pump_heat = 28.0 if valve_25_charging else 6.0
            p_servo_heat = cyl_motion_intensity * 12.0
            
            # 目标平衡温度
            target_t_oil = self.t_hub + p_pump_heat + p_servo_heat
            
            # 辅助温控逻辑:
            # 当油温高于 60°C 自动开启冷却器
            self.oil_cooler = (self.t_oil > 58.0)
            if self.oil_cooler:
                target_t_oil -= 14.0
                
            # 当油温低于 15°C 启动油箱加热器
            self.oil_heater = (self.t_oil < 15.0)
            if self.oil_heater:
                target_t_oil += 25.0

            tau_oil = self.tau_oil_heat if (target_t_oil > self.t_oil) else self.tau_oil_cool
            self.t_oil += (target_t_oil - self.t_oil) * (self.dt / max(1.0, tau_oil))

        # 3. 变桨控制电气柜温度模型
        if self.fault_fan_fail:
            self.cabinet_fan = False
        else:
            # 柜温高于 35°C 或正常通电下风扇常开
            self.cabinet_fan = True

        # 柜内元器件发热 (约 +12°C 温升)
        cab_internal_heat = 10.0 if self.cabinet_fan else 28.0
        target_cab = self.t_hub + cab_internal_heat
        tau_cab = self.tau_cab_cool if self.cabinet_fan else self.tau_cab_heat
        self.t_cab += (target_cab - self.t_cab) * (self.dt / max(1.0, tau_cab))

        # 4. PLC CPU 核心温度模型 (CPU 高于柜内温度 12 ~ 15°C)
        target_cpu = self.t_cab + 13.5
        self.t_cpu += (target_cpu - self.t_cpu) * (self.dt / 5.0)

        # 5. 发电机气隙温度模型 (通常受发电机机舱环境影响，稳态在 36~45°C)
        target_airgap = self.t_hub + 14.0 + (5.0 if valve_25_charging else 0.0)
        self.t_airgap += (target_airgap - self.t_airgap) * (self.dt / 20.0)

        # 6. 变桨/主轴承温度模型
        # 变桨动作时摩擦做功产热，若润滑泵运行则散热润滑良好
        lub_friction = 0.5 if lub_pumps_on else 2.5
        bearing_heat = 8.0 + cyl_motion_intensity * 10.0 * lub_friction
        target_bearing = self.t_hub + bearing_heat
        self.t_bearing += (target_bearing - self.t_bearing) * (self.dt / self.tau_bearing)

        # 7. 告警阈值计算 (对照 CODESYS FB_AlarmConfigLoader / PRG_pv2csv)
        alarm_oil_high = (self.t_oil >= 70.0)
        alarm_oil_low = (self.t_oil <= -20.0)
        alarm_cab_high = (self.t_cab >= 60.0)
        alarm_cab_low = (self.t_cab <= -10.0)
        alarm_hub_high = (self.t_hub >= 55.0)
        alarm_hub_low = (self.t_hub <= -25.0)
        alarm_cpu_high = (self.t_cpu >= 80.0)

        # 输出温度遥测数值 (若断线故障，输出 -999.0 代表 Open Line)
        out_oil = -999.0 if self.fault_pt100_open else round(self.t_oil, 2)
        out_hub = -999.0 if self.fault_pt100_open else round(self.t_hub, 2)
        out_cab = -999.0 if self.fault_pt100_open else round(self.t_cab, 2)
        out_airgap = -999.0 if self.fault_pt100_open else round(self.t_airgap, 2)
        out_bearing = -999.0 if self.fault_pt100_open else round(self.t_bearing, 2)
        out_cpu = round(self.t_cpu, 2)

        return TemperatureTelemetry(
            temp_oil_deg=out_oil,
            temp_hub_deg=out_hub,
            temp_cabinet_deg=out_cab,
            temp_airgap_deg=out_airgap,
            temp_bearing_deg=out_bearing,
            temp_cpu_deg=out_cpu,
            cabinet_fan=self.cabinet_fan,
            oil_heater=self.oil_heater,
            oil_cooler=self.oil_cooler,
            alarm_oil_high=alarm_oil_high,
            alarm_oil_low=alarm_oil_low,
            alarm_cab_high=alarm_cab_high,
            alarm_cab_low=alarm_cab_low,
            alarm_hub_high=alarm_hub_high,
            alarm_hub_low=alarm_hub_low,
            alarm_cpu_high=alarm_cpu_high,
            sensor_fault_open=self.fault_pt100_open
        )
