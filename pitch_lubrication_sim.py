"""
兆瓦级 (6MW) 轮毂双泵递进式集中润滑系统动力学仿真模型
=============================================================
功能特征:
- 支持 6MW 双泵独立回路 (1# 主润滑泵 与 2# 冗余/副润滑泵)
- 递进式油脂分配器 (Progressive Metering Feeder) 顺序活塞运动时序仿真
- 叶片 A/B/C 三面活塞循环传感器 (PistonMoving) 动态脉冲生成
- 油脂箱液位动态消耗与低液位故障模拟 (OilLevelOk)
- 管路压力充压与泄压动态 (0 ~ 200 bar)
- 仿真故障注入: 模拟低液位、模拟分配器堵塞溢流
"""

import math
from dataclasses import dataclass


@dataclass
class LubricationTelemetry:
    pump1_on: bool = False
    pump2_on: bool = False
    pump1_pressure_bar: float = 0.0
    pump2_pressure_bar: float = 0.0
    tank1_level_pct: float = 85.0
    tank2_level_pct: float = 90.0
    tank1_level_ok: bool = True
    tank2_level_ok: bool = True
    piston1_a: bool = False
    piston1_b: bool = False
    piston1_c: bool = False
    piston2_a: bool = False
    piston2_b: bool = False
    piston2_c: bool = False
    cycle_count_1: int = 0
    cycle_count_2: int = 0
    fault_tank1_empty: bool = False
    fault_tank2_empty: bool = False
    fault_block_1: bool = False
    fault_block_2: bool = False


class LubricationSystemSim:
    def __init__(self, dt: float = 0.005):
        self.dt = dt

        # 1# 泵系统状态
        self.pump1_on = False
        self.p1_bar = 0.0
        self.tank1_level_pct = 85.0
        self.timer_1 = 0.0
        self.piston1_a = False
        self.piston1_b = False
        self.piston1_c = False
        self.cycle_count_1 = 0

        # 2# 泵系统状态
        self.pump2_on = False
        self.p2_bar = 0.0
        self.tank2_level_pct = 90.0
        self.timer_2 = 0.0
        self.piston2_a = False
        self.piston2_b = False
        self.piston2_c = False
        self.cycle_count_2 = 0

        # 故障注入开关
        self.fault_tank1_empty = False
        self.fault_tank2_empty = False
        self.fault_block_1 = False
        self.fault_block_2 = False

        # 分配器全循环周期
        self.T_cycle_1 = 2.4  # 秒 (每个叶片分配器段耗时 0.8s)
        self.T_cycle_2 = 2.7  # 秒

    def set_faults(self, tank1_empty=None, tank2_empty=None, block_1=None, block_2=None):
        if tank1_empty is not None:
            self.fault_tank1_empty = bool(tank1_empty)
        if tank2_empty is not None:
            self.fault_tank2_empty = bool(tank2_empty)
        if block_1 is not None:
            self.fault_block_1 = bool(block_1)
        if block_2 is not None:
            self.fault_block_2 = bool(block_2)

    def step(self, cmd_pump1: bool, cmd_pump2: bool) -> LubricationTelemetry:
        self.pump1_on = bool(cmd_pump1)
        self.pump2_on = bool(cmd_pump2)

        # -------------------------------------------------------------
        # 1# 润滑泵回路动态推进
        # -------------------------------------------------------------
        if self.pump1_on:
            target_p1 = 260.0 if self.fault_block_1 else 185.0
            self.p1_bar += (target_p1 - self.p1_bar) * (self.dt / 0.4)
            # 缓慢消耗油脂 (约 0.01% 每秒)
            self.tank1_level_pct = max(0.0, self.tank1_level_pct - 0.005 * self.dt)

            if not self.fault_block_1:
                self.timer_1 += self.dt
                if self.timer_1 >= self.T_cycle_1:
                    self.timer_1 -= self.T_cycle_1
                    self.cycle_count_1 += 1

                t1 = self.timer_1
                seg = self.T_cycle_1 / 3.0
                # 递进式分配器 A -> B -> C 依次动作脉冲 (每个区间内脉冲占空比约 50%)
                self.piston1_a = (0.15 <= t1 < 0.15 + seg * 0.6)
                self.piston1_b = (seg + 0.15 <= t1 < seg + 0.15 + seg * 0.6)
                self.piston1_c = (seg * 2.0 + 0.15 <= t1 < seg * 2.0 + 0.15 + seg * 0.6)
            else:
                # 堵塞故障: 活塞停止动作
                self.piston1_a = False
                self.piston1_b = False
                self.piston1_c = False
        else:
            self.p1_bar += (0.0 - self.p1_bar) * (self.dt / 0.25)
            self.piston1_a = False
            self.piston1_b = False
            self.piston1_c = False

        # -------------------------------------------------------------
        # 2# 润滑泵回路动态推进
        # -------------------------------------------------------------
        if self.pump2_on:
            target_p2 = 260.0 if self.fault_block_2 else 190.0
            self.p2_bar += (target_p2 - self.p2_bar) * (self.dt / 0.4)
            self.tank2_level_pct = max(0.0, self.tank2_level_pct - 0.005 * self.dt)

            if not self.fault_block_2:
                self.timer_2 += self.dt
                if self.timer_2 >= self.T_cycle_2:
                    self.timer_2 -= self.T_cycle_2
                    self.cycle_count_2 += 1

                t2 = self.timer_2
                seg2 = self.T_cycle_2 / 3.0
                self.piston2_a = (0.15 <= t2 < 0.15 + seg2 * 0.6)
                self.piston2_b = (seg2 + 0.15 <= t2 < seg2 + 0.15 + seg2 * 0.6)
                self.piston2_c = (seg2 * 2.0 + 0.15 <= t2 < seg2 * 2.0 + 0.15 + seg2 * 0.6)
            else:
                self.piston2_a = False
                self.piston2_b = False
                self.piston2_c = False
        else:
            self.p2_bar += (0.0 - self.p2_bar) * (self.dt / 0.25)
            self.piston2_a = False
            self.piston2_b = False
            self.piston2_c = False

        tank1_ok = (self.tank1_level_pct > 15.0) and (not self.fault_tank1_empty)
        tank2_ok = (self.tank2_level_pct > 15.0) and (not self.fault_tank2_empty)

        return LubricationTelemetry(
            pump1_on=self.pump1_on,
            pump2_on=self.pump2_on,
            pump1_pressure_bar=max(0.0, self.p1_bar),
            pump2_pressure_bar=max(0.0, self.p2_bar),
            tank1_level_pct=self.tank1_level_pct,
            tank2_level_pct=self.tank2_level_pct,
            tank1_level_ok=tank1_ok,
            tank2_level_ok=tank2_ok,
            piston1_a=self.piston1_a,
            piston1_b=self.piston1_b,
            piston1_c=self.piston1_c,
            piston2_a=self.piston2_a,
            piston2_b=self.piston2_b,
            piston2_c=self.piston2_c,
            cycle_count_1=self.cycle_count_1,
            cycle_count_2=self.cycle_count_2,
            fault_tank1_empty=self.fault_tank1_empty,
            fault_tank2_empty=self.fault_tank2_empty,
            fault_block_1=self.fault_block_1,
            fault_block_2=self.fault_block_2
        )
