"""
=============================================================================
兆瓦级风电机组液压变桨控制系统
CODESYS OPC UA <-> Python 变桨数字孪生仿真机 高速双向桥接网关
=============================================================================
功能:
1. 建立与 CODESYS 运行时内置 OPC UA 服务器 (opc.tcp://127.0.0.1:4840) 的异步连接。
2. 200Hz (5ms) 周期从 CODESYS 批量读取 PRG_pitch_sim 下发的各电磁阀与比例阀指令，传给 SoftRealtimePitchEngine。
3. 200Hz (5ms) 周期将仿真机生成的物理遥测数据 (油缸位移、各腔压力、锁状态、开度等) 批量回写至 PRG_pitch_sim 与 Common.blade[i].io.ai.raw.*。
4. 维护双向心跳检测 (udiHeartbeatPlc / udiHeartbeatSim) 与线程生命周期管理。
=============================================================================
"""

import asyncio
import ctypes
import logging
import sys
import threading
import time
from typing import Dict, Any, Optional, Callable

try:
    from asyncua import Client, Node, ua
except ImportError:
    Client = None
    ua = None

from pitch_soft_realtime import SoftRealtimePitchEngine

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logging.getLogger("asyncua").setLevel(logging.WARNING)
logger = logging.getLogger("PitchOpcUaBridge")


def _safe_float(val: Any, default: float = 0.0) -> float:
    """安全转换为 float，防止 CODESYS 符号重载返回 None 时报 TypeError"""
    if val is None:
        return default
    try:
        return float(val)
    except (TypeError, ValueError):
        return default


def _safe_int(val: Any, default: int = 0) -> int:
    """安全转换为 int，防止 CODESYS 符号重载返回 None 时报 TypeError"""
    if val is None:
        return default
    try:
        return int(val)
    except (TypeError, ValueError):
        return default


def _safe_bool(val: Any, default: bool = False) -> bool:
    """安全转换为 bool，防止 CODESYS 符号重载返回 None 时报 TypeError"""
    if val is None:
        return default
    return bool(val)


class PitchOpcUaBridge:
    def __init__(self, endpoint: str = "opc.tcp://127.0.0.1:4840",
                 engine: Optional[SoftRealtimePitchEngine] = None,
                 cycle_time_s: float = 0.005,
                 on_status_change: Optional[Callable[[bool, str], None]] = None):
        """
        :param endpoint: CODESYS OPC UA 终端地址 (默认本机 4840 端口)
        :param engine: 软实时液压仿真内核实例
        :param cycle_time_s: 同步周期 (默认 5ms / 200Hz 超高速微周期同步)
        :param on_status_change: 状态改变回调函数 callback(is_connected: bool, msg: str)
        """
        self.endpoint = endpoint
        self.engine = engine
        self.cycle_time_s = cycle_time_s
        self.on_status_change = on_status_change

        self.is_connected = False
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._stop_event = threading.Event()

        # 心跳统计
        self.heartbeat_sim = 0
        self.heartbeat_plc = 0
        self.last_error = ""

    def start(self):
        """启动后台同步工作线程"""
        if self._running or (self._thread and self._thread.is_alive()):
            logger.warning("OPC UA 网关已在运行中，忽略重复启动。")
            return

        self._running = True
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._worker_thread_main, daemon=True, name="PitchOpcUaBridgeWorker")
        self._thread.start()
        logger.info("OPC UA 桥接后台线程已启动。")

    def stop(self):
        """停止后台同步线程并安全断开连接"""
        if not self._running:
            return

        logger.info("正在停止 OPC UA 桥接...")
        self._running = False
        self._stop_event.set()

        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=3.0)

        self.is_connected = False
        logger.info("OPC UA 桥接已安全停止。")
        self._notify_status(False, "已停止")

    def _notify_status(self, connected: bool, msg: str):
        self.is_connected = connected
        self.last_error = msg if not connected else ""
        if self.on_status_change:
            try:
                self.on_status_change(connected, msg)
            except Exception as e:
                logger.error(f"状态回调执行异常: {e}")

    def _worker_thread_main(self):
        """工作线程入口：创建并运行独立 asyncio 事件循环"""
        if sys.platform == "win32":
            try:
                ctypes.windll.winmm.timeBeginPeriod(1)
            except Exception:
                pass

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._async_sync_runner())
        except Exception as e:
            logger.error(f"OPC UA 事件循环退出: {e}")
        finally:
            if sys.platform == "win32":
                try:
                    ctypes.windll.winmm.timeEndPeriod(1)
                except Exception:
                    pass
            self._loop.close()
            self._loop = None
            self.is_connected = False

    async def _async_sync_runner(self):
        """主异步协程：建立连接并执行高速批量同步"""
        if Client is None:
            err = "缺少 asyncua 依赖库"
            logger.error(err)
            self._notify_status(False, err)
            return

        while self._running and not self._stop_event.is_set():
            try:
                self._notify_status(False, f"正在连接 {self.endpoint} ...")
                async with Client(url=self.endpoint, timeout=3.0) as client:
                    prefix = "ns=4;s=|var|CODESYS Control Win V3.PAC.PRG_pitch_sim."

                    # 0. 动态探测 PLC PRG_pitch_sim 下实际导出的变量清单 (支持多版本兼容)
                    parent_node = client.get_node("ns=4;s=|var|CODESYS Control Win V3.PAC.PRG_pitch_sim")
                    children = await parent_node.get_children()
                    available_names = set()
                    for ch in children:
                        bn = await ch.read_browse_name()
                        available_names.add(bn.Name)

                    # 1. 预先解析并缓存需要读取的 PLC 控制指令节点 (Read Nodes)
                    candidate_cmds = []
                    for b in ['A', 'B', 'C']:
                        candidate_cmds.extend([
                            f"rPropVoltage_{b}",
                            f"bValve109_{b}",
                            f"bValve119A_{b}",
                            f"bValve119B_{b}",
                            f"bValve120_{b}",
                            f"bValve103_{b}",
                            f"bValve116_{b}",
                            f"bValve130_{b}",
                            f"bValve132_{b}",
                        ])
                    candidate_cmds.extend(["bValve25_Cmd", "udiHeartbeatPlc", "sControlModeCmd"])
                    active_cmd_names = [n for n in candidate_cmds if n in available_names]
                    cmd_nodes = [client.get_node(f"{prefix}{n}") for n in active_cmd_names]

                    # 2. 预先构建并自适应过滤需要写入的遥测反馈模式表 (Write Nodes Schema)
                    candidate_fb_items = []
                    for b in ['A', 'B', 'C']:
                        candidate_fb_items.extend([
                            (f"rDisplacement_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].displacement_mm), ua.VariantType.Float),
                            (f"rVelocity_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].velocity_mmps), ua.VariantType.Float),
                            (f"rValveOpeningVolt_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].valve_opening_volt), ua.VariantType.Float),
                            (f"rValveOpeningPct_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].valve_opening_pct), ua.VariantType.Float),
                            (f"rValveSpoolPos_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].valve_spool_pos), ua.VariantType.Float),
                            (f"bIsLocked_{b}", lambda fb, st, hb, b=b: bool(fb.blades[b].is_locked), ua.VariantType.Boolean),
                            (f"rLockPin_mm_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].lock_pin_mm), ua.VariantType.Float),
                            (f"bSwitch129_{b}", lambda fb, st, hb, b=b: bool(fb.blades[b].switch_129), ua.VariantType.Boolean),
                            (f"bSwitchS_Unlocked_{b}", lambda fb, st, hb, b=b: bool(fb.blades[b].switch_S_unlocked), ua.VariantType.Boolean),
                            (f"bSwitchS_Locked_{b}", lambda fb, st, hb, b=b: bool(fb.blades[b].switch_S_locked), ua.VariantType.Boolean),
                            (f"rPressure107_bar_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].pressure_107_bar), ua.VariantType.Float),
                            (f"rPressureA_bar_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].p_A_bar), ua.VariantType.Float),
                            (f"rPressureB_bar_{b}", lambda fb, st, hb, b=b: float(fb.blades[b].p_B_bar), ua.VariantType.Float),
                        ])
                    candidate_fb_items.extend([
                        ("rPressure107_bar", lambda fb, st, hb: float(fb.pressure_107_bar), ua.VariantType.Float),
                        ("rPressureSupply_bar", lambda fb, st, hb: float(fb.p_supply_bar), ua.VariantType.Float),
                        ("bValve25_State", lambda fb, st, hb: bool(fb.valve_25_state), ua.VariantType.Boolean),
                        ("rFlowHpu_lpm", lambda fb, st, hb: float(fb.q_hpu_lpm), ua.VariantType.Float),
                        ("rMotorRpm", lambda fb, st, hb: float(fb.motor_rpm), ua.VariantType.Float),
                        ("rSimTime", lambda fb, st, hb: float(st), ua.VariantType.Float),
                        ("sControlModeActive", lambda fb, st, hb: "REMOTE", ua.VariantType.String),
                        ("udiHeartbeatSim", lambda fb, st, hb: int(hb), ua.VariantType.UInt32),
                    ])

                    # 仅保留 PLC 当前符号表中实际存在的变量节点，彻底杜绝 BadNodeIdInvalid
                    active_fb_items = [(name, getter, vtype) for name, getter, vtype in candidate_fb_items if name in available_names]
                    fb_nodes = [client.get_node(f"{prefix}{name}") for name, _, _ in active_fb_items]

                    # 3. 预先构建并自适应挂载 B&R X20 硬件采样 raw 写入节点 (B&R X20DC2190 / AI 模块)
                    # 彻底解决 PRG_main_pac 与 PRG_pitch_sim 周期扫描对 Common.blade[i].io.ai.* 争用跳变
                    raw_fb_candidates = []
                    bld_map = [('A', 0), ('B', 1), ('C', 2)]
                    for b, idx in bld_map:
                        raw_fb_candidates.extend([
                            # B&R X20DC2190 SSI 编码器: 1 count = 1um (0.001mm), DINT (Int32)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.cylPos",
                                lambda fb, st, hb, b=b: int(round(fb.blades[b].displacement_mm * 1000.0)),
                                ua.VariantType.Int32
                            ),
                            # 速度反馈: 1 count = 0.1 mm/s, INT (Int16)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.cylSpd",
                                lambda fb, st, hb, b=b: int(max(-32768, min(32767, round(fb.blades[b].velocity_mmps * 10.0)))),
                                ua.VariantType.Int16
                            ),
                            # 107 蓄能器压力: 400.0 / 32768 = 0.01220703125 bar/count, INT (Int16)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.presAccu",
                                lambda fb, st, hb, b=b: int(max(0, min(32767, round(fb.blades[b].pressure_107_bar / 0.01220703125)))),
                                ua.VariantType.Int16
                            ),
                            # 无杆腔 A 压力: INT (Int16)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.presPiston",
                                lambda fb, st, hb, b=b: int(max(0, min(32767, round(fb.blades[b].p_A_bar / 0.01220703125)))),
                                ua.VariantType.Int16
                            ),
                            # 有杆腔 B 压力: INT (Int16)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.presRod",
                                lambda fb, st, hb, b=b: int(max(0, min(32767, round(fb.blades[b].p_B_bar / 0.01220703125)))),
                                ua.VariantType.Int16
                            ),
                            # 比例阀阀芯开度反馈: 100.0 / 32768 = 0.0030517578125 %/count, INT (Int16)
                            (
                                f"ns=4;s=|var|CODESYS Control Win V3.PAC.Common.blade[{idx}].io.ai.raw.spoolPosFb",
                                lambda fb, st, hb, b=b: int(max(-32768, min(32767, round(fb.blades[b].valve_opening_pct / 0.0030517578125)))),
                                ua.VariantType.Int16
                            ),
                        ])

                    # 探测 raw 节点有效性与可写性
                    active_raw_items = []
                    for nid, getter, vtype in raw_fb_candidates:
                        try:
                            n = client.get_node(nid)
                            await n.read_data_value()
                            active_raw_items.append((n, getter, vtype))
                        except Exception as e:
                            logger.debug(f"Raw 节点 {nid} 不可用或未导出: {e}")

                    # 合并所有写入项 (PRG_pitch_sim + raw)，一次性预构建以优化循环性能
                    all_write_items = [(client.get_node(f"{prefix}{name}"), getter, vtype) for name, getter, vtype in active_fb_items] + active_raw_items
                    all_write_nodes = [item[0] for item in all_write_items]

                    # 探测连接成功
                    self._notify_status(True, f"已连接 (opc.tcp://127.0.0.1:4840)")
                    logger.info(f"✔ CODESYS OPC UA 建立连接！探测到 {len(available_names)} 个符号，挂载 {len(cmd_nodes)} 读取节点与 {len(all_write_nodes)} 写入节点 (含 {len(active_raw_items)} 个 X20 raw 底层通道)。")

                    # 高速微周期循环同步 (5ms / 200Hz 超高频对齐)
                    sim_start_time = time.time()
                    target_next_time = time.perf_counter()
                    while self._running and not self._stop_event.is_set():
                        target_next_time += self.cycle_time_s

                        # --- 步骤 A: 批量读取 PLC 发给仿真机的控制指令 ---
                        cmd_vals = await client.read_values(cmd_nodes)
                        cmd_map = dict(zip(active_cmd_names, cmd_vals))

                        # 解析 PLC 心跳 (防 None 异常)
                        self.heartbeat_plc = _safe_int(cmd_map.get("udiHeartbeatPlc"), 0)

                        # 将各面阀门指令传给仿真内核
                        if self.engine:
                            for b in ['A', 'B', 'C']:
                                self.engine.send_remote_command(
                                    blade=b,
                                    prop_voltage=_safe_float(cmd_map.get(f"rPropVoltage_{b}"), 0.0),
                                    valve_109=_safe_bool(cmd_map.get(f"bValve109_{b}"), True),
                                    valve_119A=_safe_bool(cmd_map.get(f"bValve119A_{b}"), True),
                                    valve_119B=_safe_bool(cmd_map.get(f"bValve119B_{b}"), True),
                                    valve_120=_safe_bool(cmd_map.get(f"bValve120_{b}"), False),
                                    valve_103=_safe_bool(cmd_map.get(f"bValve103_{b}"), False),
                                    valve_116=_safe_bool(cmd_map.get(f"bValve116_{b}"), True),
                                    valve_130=_safe_bool(cmd_map.get(f"bValve130_{b}"), True),
                                    valve_132=_safe_bool(cmd_map.get(f"bValve132_{b}"), True),
                                )

                        # --- 步骤 B: 收集仿真内核物理遥测并批量写入 PLC ---
                        self.heartbeat_sim = (self.heartbeat_sim + 1) % 0xFFFFFFFF
                        sim_time = time.time() - sim_start_time

                        if self.engine:
                            fb = self.engine.get_remote_feedback()
                        else:
                            fb = None

                        if fb:
                            all_write_values = [
                                ua.DataValue(ua.Variant(getter(fb, sim_time, self.heartbeat_sim), vtype))
                                for _, getter, vtype in all_write_items
                            ]
                            # 单次调用批量写回，避免多余网络开销与时延
                            await client.write_values(all_write_nodes, all_write_values)

                        # --- 步骤 C: 微秒级高精度软实时对齐 5ms (200Hz) ---
                        now = time.perf_counter()
                        rem = target_next_time - now
                        if rem > 0.0015:
                            await asyncio.sleep(rem - 0.001)
                        while time.perf_counter() < target_next_time:
                            pass
                        if time.perf_counter() - target_next_time > self.cycle_time_s:
                            target_next_time = time.perf_counter()

            except Exception as e:
                err_msg = f"连接断开或异常: {e}"
                logger.warning(err_msg)
                self._notify_status(False, err_msg)
                if not self._running or self._stop_event.is_set():
                    break
                await asyncio.sleep(1.0)  # 异常后重试间隔 1.0s


if __name__ == "__main__":
    from pitch_soft_realtime import SoftRealtimePitchEngine
    print("测试启动独立 OPC UA 桥接...")
    test_engine = SoftRealtimePitchEngine(cycle_time_s=0.010)
    test_engine.start()
    test_engine.set_control_mode("REMOTE")

    bridge = PitchOpcUaBridge(engine=test_engine)
    bridge.start()
    try:
        for _ in range(5):
            time.sleep(1)
            print(f"Bridge 状态: connected={bridge.is_connected}, plc_hb={bridge.heartbeat_plc}, sim_hb={bridge.heartbeat_sim}")
    finally:
        bridge.stop()
        test_engine.stop()
        print("测试完成。")
