"""
兆瓦级风机液压变桨仿真系统 - PyQt6 工业级主监控窗口 (PitchMainWindow)
组装协调左侧控制面板、叶片A/B/C拓扑曲线、集中润滑监控、温度热力学仿真及 OPC UA 网关
"""

import sys
import time
import numpy as np

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSplitter, QFrame, QTabWidget, QScrollArea
)
from PyQt6.QtCore import Qt, QTimer

from pitch_soft_realtime import SoftRealtimePitchEngine
from pitch_opcua_bridge import PitchOpcUaBridge
from pitch_conn_dialog import ConnectionSettingsDialog, load_connection_config

from gui.left_control_panel import LeftControlPanel
from gui.blade_face_tab import BladeFaceTabWidget
from gui.lubrication_tab import LubricationTabWidget
from gui.temperature_tab import TemperatureTabWidget


class PitchMainWindow(QMainWindow):
    """兆瓦级风机液压变桨系统数字孪生仿真机主视窗"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("兆瓦级风力发电机组液压变桨系统 - 数字孪生实时仿真监控机")
        self.resize(1440, 940)
        self.setMinimumSize(1200, 780)

        self.blades = ['A', 'B', 'C']
        self.active_blade = 'A'
        self.is_sync_all = False
        self.control_mode = 'LOCAL'  # 'LOCAL' or 'REMOTE'

        # 启动底层 200Hz (5ms) 软实时多面液压仿真内核
        self.engine = SoftRealtimePitchEngine(cycle_time_s=0.005)
        self.engine.start()

        # 读取持久化 OPC UA 连接参数配置并初始化桥接网关
        self.conn_config = load_connection_config()
        self.opcua_endpoint = self.conn_config.get("endpoint", "opc.tcp://127.0.0.1:4840")
        self.opcua_cycle_time_s = float(self.conn_config.get("cycle_time_ms", 5.0)) / 1000.0

        self.opcua_bridge = PitchOpcUaBridge(
            endpoint=self.opcua_endpoint,
            engine=self.engine,
            cycle_time_s=self.opcua_cycle_time_s
        )

        # 控制状态 (支持多面独立或同步联动)
        self.is_auto_mode = True
        self.target_pos_mm = {k: 150.0 for k in self.blades}
        self.manual_voltage = {k: 0.0 for k in self.blades}
        self.is_feathering = {k: False for k in self.blades}

        # 变桨锁控制电平 (初始: 拔销解锁 130=1, 132=1)
        self.u_130_cmd = {k: 1.0 for k in self.blades}
        self.u_132_cmd = {k: 1.0 for k in self.blades}

        # PID 控制参数与积分历史
        self.kp, self.ki, self.kd = 45.0, 1.8, 5.0
        self.integral = {k: 0.0 for k in self.blades}
        self.last_err = {k: 0.0 for k in self.blades}

        # 示波器历史缓存 (300点, 每个面独立维护)
        self.buf_size = 300
        self.time_hist = np.linspace(-3.0, 0.0, self.buf_size)
        self.target_hist = {k: np.full(self.buf_size, 150.0) for k in self.blades}
        self.stroke_hist = {k: np.full(self.buf_size, 50.0) for k in self.blades}
        self.pA_hist = {k: np.full(self.buf_size, 15.0) for k in self.blades}
        self.pB_hist = {k: np.full(self.buf_size, 15.0) for k in self.blades}
        self.pacc_hist = {k: np.full(self.buf_size, 260.0) for k in self.blades}
        self.pSupply_hist = {k: np.full(self.buf_size, 2.5) for k in self.blades}

        self._init_ui()
        self._apply_global_styles()

        # 50 FPS (20ms) UI 刷新定时器
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_ui_tick)
        self.timer.start(20)

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(10, 8, 10, 8)
        main_layout.setSpacing(6)

        # 1. 顶部状态栏
        top_bar = QWidget()
        top_bar.setObjectName("topBar")
        top_bar.setStyleSheet("""
            #topBar {
                background-color: #1A1C24;
                border: 1px solid #2B2E3B;
                border-radius: 6px;
            }
        """)
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(12, 6, 12, 6)
        top_layout.setSpacing(10)

        title_lbl = QLabel("⚡ 变桨液压仿真系统")
        title_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #EDF2F4;")
        top_layout.addWidget(title_lbl)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.VLine)
        sep1.setStyleSheet("color: #3D4357; max-height: 20px;")
        top_layout.addWidget(sep1)

        # 控制权限切换
        self.btn_local_ctrl = QPushButton("📍 本地控制")
        self.btn_local_ctrl.setCheckable(True)
        self.btn_local_ctrl.setChecked(True)
        self.btn_local_ctrl.setFixedHeight(28)
        self.btn_local_ctrl.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_local_ctrl.setStyleSheet("background-color: #06D6A0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px; padding: 0 12px;")
        self.btn_local_ctrl.clicked.connect(lambda: self._set_control_authority("LOCAL"))

        self.btn_remote_ctrl = QPushButton("🌐 远程控制")
        self.btn_remote_ctrl.setCheckable(True)
        self.btn_remote_ctrl.setChecked(False)
        self.btn_remote_ctrl.setFixedHeight(28)
        self.btn_remote_ctrl.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_remote_ctrl.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px; padding: 0 12px;")
        self.btn_remote_ctrl.clicked.connect(lambda: self._set_control_authority("REMOTE"))

        top_layout.addWidget(self.btn_local_ctrl)
        top_layout.addWidget(self.btn_remote_ctrl)

        self.lbl_auth_status = QLabel("● 本地就绪")
        self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        top_layout.addWidget(self.lbl_auth_status)

        # 连接设置小齿轮图标按钮
        self.btn_conn_settings = QPushButton("⚙")
        self.btn_conn_settings.setFixedHeight(28)
        self.btn_conn_settings.setFixedWidth(32)
        self.btn_conn_settings.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_conn_settings.setToolTip("OPC UA 通讯参数与连接诊断设置")
        self.btn_conn_settings.setStyleSheet("""
            QPushButton {
                background-color: #212530;
                color: #A0AAB8;
                border: 1px solid #363D4E;
                border-radius: 4px;
                font-size: 14px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2E3445;
                color: #4CC9F0;
                border-color: #4CC9F0;
            }
            QPushButton:pressed {
                background-color: #1A1D26;
                color: #06D6A0;
            }
        """)
        self.btn_conn_settings.clicked.connect(self._open_conn_settings)
        top_layout.addWidget(self.btn_conn_settings)

        top_layout.addStretch()

        self.lbl_clock_stat = QLabel("时钟: 0.0s | 200Hz (5.0ms) | 抖动: 0.0ms")
        self.lbl_clock_stat.setStyleSheet("font-family: Consolas; font-size: 12px; color: #06D6A0; font-weight: bold;")
        top_layout.addWidget(self.lbl_clock_stat)
        main_layout.addWidget(top_bar)

        # 2. 中间主体分割器
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(8)

        # 左侧控制面板 (包装在无边框自适应滚动区内)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setStyleSheet("background-color: transparent; border: none;")
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.left_panel = LeftControlPanel()
        self.left_panel.faceSelected.connect(self._select_face)
        self.left_panel.syncChanged.connect(self._on_sync_changed)
        self.left_panel.autoModeChanged.connect(self._set_mode)
        self.left_panel.targetChanged.connect(self._on_target_changed)
        self.left_panel.voltageChanged.connect(self._on_voltage_changed)
        self.left_panel.unlockTriggered.connect(self._cmd_unlock)
        self.left_panel.lockTriggered.connect(self._cmd_lock)
        self.left_panel.featherToggled.connect(self._on_toggle_feather)
        self.left_panel.motorToggled.connect(self._on_toggle_motor)
        scroll_area.setWidget(self.left_panel)
        splitter.addWidget(scroll_area)

        # 右侧多选项卡视窗
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)

        self.tab_widget = QTabWidget()
        self.tab_widget.setDocumentMode(True)
        self.tab_widget.currentChanged.connect(self._on_tab_changed)

        # Tabs 0, 1, 2: 叶片 A, B, C
        self.face_tabs = {}
        for k in self.blades:
            t = BladeFaceTabWidget(face_name=k)
            self.face_tabs[k] = t
            self.tab_widget.addTab(t, f"叶片 {k}")

        # Tab 3: 集中润滑系统
        self.lub_tab = LubricationTabWidget(buf_size=self.buf_size)
        self.lub_tab.togglePump1.connect(self._on_toggle_lub1_pump)
        self.lub_tab.togglePump2.connect(self._on_toggle_lub2_pump)
        self.lub_tab.faultsChanged.connect(self._on_lub_faults_changed)
        self.tab_widget.addTab(self.lub_tab, "集中润滑系统")

        # Tab 4: 温度监控系统
        self.temp_tab = TemperatureTabWidget(buf_size=self.buf_size)
        self.temp_tab.ambientChanged.connect(lambda t: self.engine.set_ambient_temperature(t))
        self.temp_tab.faultsChanged.connect(self._on_temp_faults_changed)
        self.tab_widget.addTab(self.temp_tab, "温度监控系统")

        right_layout.addWidget(self.tab_widget)
        splitter.addWidget(right_panel)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 1060])
        main_layout.addWidget(splitter, stretch=1)

    def _apply_global_styles(self):
        """统一暗黑工业级 UI 视觉样式表"""
        self.setStyleSheet("""
            QMainWindow {
                background-color: #121318;
            }
            QWidget {
                color: #EDF2F4;
                font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
            }
            QGroupBox {
                background-color: #1A1C24;
                border: 1px solid #2B2E3B;
                border-radius: 6px;
                margin-top: 10px;
                font-size: 12px;
                font-weight: bold;
                color: #EDF2F4;
                padding-top: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 4px;
                background-color: #1A1C24;
            }
            QTabWidget::pane {
                border: 1px solid #2B2E3B;
                background-color: #1A1C24;
                border-radius: 6px;
            }
            QTabBar::tab {
                background-color: #1F222C;
                color: #8D99AE;
                padding: 6px 20px;
                margin-right: 4px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
                font-size: 11px;
                font-weight: bold;
            }
            QTabBar::tab:selected {
                background-color: #2B2E3B;
                color: #4CC9F0;
                border-bottom: 2px solid #4CC9F0;
            }
            QSplitter::handle {
                background-color: #2B2E3B;
                border-radius: 2px;
            }
            QSplitter::handle:hover {
                background-color: #4CC9F0;
            }
        """)

    # =========================================================================
    # 连接配置对话框
    # =========================================================================
    def _open_conn_settings(self):
        dlg = ConnectionSettingsDialog(parent=self, current_config=self.conn_config, on_apply=self._apply_conn_config)
        dlg.exec()

    def _apply_conn_config(self, new_config: dict):
        self.conn_config = new_config
        new_endpoint = new_config.get("endpoint", "opc.tcp://127.0.0.1:4840")
        new_cycle_s = float(new_config.get("cycle_time_ms", 5.0)) / 1000.0

        endpoint_changed = (new_endpoint != self.opcua_endpoint)
        cycle_changed = (abs(new_cycle_s - self.opcua_cycle_time_s) > 1e-4)

        self.opcua_endpoint = new_endpoint
        self.opcua_cycle_time_s = new_cycle_s

        if endpoint_changed or cycle_changed:
            was_running = False
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
                was_running = self.opcua_bridge.is_connected or (self.control_mode == 'REMOTE')
                self.opcua_bridge.stop()

            self.opcua_bridge = PitchOpcUaBridge(
                endpoint=self.opcua_endpoint,
                engine=self.engine,
                cycle_time_s=self.opcua_cycle_time_s
            )

            if was_running and self.control_mode == 'REMOTE':
                self.opcua_bridge.start()

    # =========================================================================
    # 控制交互与逻辑
    # =========================================================================
    def _set_control_authority(self, mode: str):
        is_local = (mode == "LOCAL")
        self.control_mode = mode
        self.engine.set_control_mode(mode)
        self.btn_local_ctrl.setChecked(is_local)
        self.btn_remote_ctrl.setChecked(not is_local)

        if is_local:
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
                self.opcua_bridge.stop()

            self.btn_local_ctrl.setStyleSheet("background-color: #06D6A0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.btn_remote_ctrl.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.lbl_auth_status.setText("● 本地就绪")
            self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
            self.left_panel.set_remote_authority(False)
        else:
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
                self.opcua_bridge.start()

            self.btn_local_ctrl.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.btn_remote_ctrl.setStyleSheet("background-color: #4CC9F0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.lbl_auth_status.setText("🔄 连接中 (4840)...")
            self.lbl_auth_status.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #2B2317; border-radius: 4px; border: 1px solid #5A4015;")
            self.left_panel.set_remote_authority(True)

    def _select_face(self, key):
        self.active_blade = key
        idx = self.blades.index(key)
        if self.tab_widget.currentIndex() != idx:
            self.tab_widget.setCurrentIndex(idx)
        self.left_panel.update_face_buttons_style(self.active_blade, self.is_sync_all)
        self._sync_left_controls_to_active_blade()

    def _on_tab_changed(self, idx):
        if 0 <= idx < len(self.blades):
            self.active_blade = self.blades[idx]
            self.left_panel.update_face_buttons_style(self.active_blade, self.is_sync_all)
            self._sync_left_controls_to_active_blade()
        else:
            for b in self.left_panel.btn_faces.values():
                b.setChecked(False)
                b.setStyleSheet("background-color: #21232C; color: #8D99AE; border: 1px solid #363A4A; border-radius: 4px;")

    def _on_sync_changed(self, state):
        self.is_sync_all = state
        if self.is_sync_all:
            cur_t = self.target_pos_mm[self.active_blade]
            cur_v = self.manual_voltage[self.active_blade]
            for k in self.blades:
                self.target_pos_mm[k] = cur_t
                self.manual_voltage[k] = cur_v
        self.left_panel.update_face_buttons_style(self.active_blade, self.is_sync_all)

    def _sync_left_controls_to_active_blade(self):
        k = self.active_blade
        self.left_panel.spin_target.setValue(self.target_pos_mm[k])
        self.left_panel.spin_voltage.setValue(self.manual_voltage[k])
        self.left_panel.update_feather_button_style(self.is_feathering[k])

    def _set_mode(self, is_auto):
        self.is_auto_mode = is_auto
        self.left_panel.update_mode_styles(is_auto)
        if is_auto:
            self.manual_voltage = {k: 0.0 for k in self.blades}
            self.left_panel.spin_voltage.setValue(0.0)
        else:
            self.integral = {k: 0.0 for k in self.blades}
            for k in self.blades:
                self.engine.set_inputs(blade=k, u_prop=0.0)

    def _on_target_changed(self, val):
        v = float(val)
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.target_pos_mm[k] = v

    def _on_voltage_changed(self, val):
        v = float(val)
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.manual_voltage[k] = v
            if not self.is_auto_mode and not self.is_feathering[k]:
                self.engine.set_inputs(blade=k, u_prop=v)

    def _on_toggle_feather(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        new_feather = not self.is_feathering[self.active_blade]
        for k in target_keys:
            self.is_feathering[k] = new_feather
            if new_feather:
                self.engine.set_inputs(blade=k, u_safe_feather=1.0, u_prop=0.0, u_enable=0.0)
            else:
                self.engine.set_inputs(blade=k, u_safe_feather=0.0, u_enable=1.0)
        self.left_panel.update_feather_button_style(new_feather)

    def _cmd_unlock(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.u_130_cmd[k] = 1.0
            self.u_132_cmd[k] = 1.0
            self.engine.set_lock_commands(blade=k, u_130=1.0, u_132=1.0)
        self.left_panel.update_lock_buttons_style(False)

    def _cmd_lock(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.u_130_cmd[k] = 0.0
            self.u_132_cmd[k] = 0.0
            self.engine.set_lock_commands(blade=k, u_130=0.0, u_132=0.0)
        self.left_panel.update_lock_buttons_style(True)

    # 液压动力源 HPU 电机交互
    def _on_toggle_motor(self):
        cur = self.engine.get_motor_running()
        new_st = not cur
        self.engine.set_motor_running(new_st)
        self.left_panel.update_motor_button_style(new_st, 1450.0 if new_st else 0.0)

    # 润滑交互
    def _on_toggle_lub1_pump(self):
        st = not self.engine._lub_cmd_pump1_local
        self.engine.set_local_lub_commands(pump1_on=st)
        self.lub_tab.update_pump_toggle_btn(1, st)

    def _on_toggle_lub2_pump(self):
        st = not self.engine._lub_cmd_pump2_local
        self.engine.set_local_lub_commands(pump2_on=st)
        self.lub_tab.update_pump_toggle_btn(2, st)

    def _on_lub_faults_changed(self, t1, t2, b1, b2):
        self.engine.set_lub_faults(tank1_empty=t1, tank2_empty=t2, block_1=b1, block_2=b2)

    def _on_temp_faults_changed(self, oil_oh, cold, fan_fail, pt100):
        self.engine.set_temp_faults(oil_overheat=oil_oh, extreme_cold=cold, fan_fail=fan_fail, pt100_open=pt100)

    # =========================================================================
    # 50 FPS 核心循环
    # =========================================================================
    def _on_ui_tick(self):
        telems = self.engine.get_all_telemetry()

        # 1. 本地控制时计算闭环 PID 运算或手动开环
        u_outputs = {}
        for k in self.blades:
            t_k = telems[k]
            x_k = t_k.displacement_mm
            if self.control_mode == 'LOCAL':
                if self.is_auto_mode and not self.is_feathering[k]:
                    if t_k.is_locked:
                        u_calc = 0.0
                        self.engine.set_inputs(blade=k, u_prop=0.0)
                    else:
                        err = (self.target_pos_mm[k] - x_k) / 1000.0
                        self.integral[k] = max(-1.0, min(1.0, self.integral[k] + err * 0.02))
                        deriv = (err - self.last_err[k]) / 0.02
                        self.last_err[k] = err
                        u_calc = max(-10.0, min(10.0, self.kp * err + self.ki * self.integral[k] + self.kd * deriv))
                        self.engine.set_inputs(blade=k, u_prop=u_calc, u_enable=1.0)
                    u_outputs[k] = u_calc
                else:
                    u_calc = self.manual_voltage[k] if (not self.is_feathering[k] and not t_k.is_locked) else 0.0
                    if not self.is_auto_mode:
                        self.engine.set_inputs(blade=k, u_prop=u_calc)
                    u_outputs[k] = u_calc
            else:
                # 远程模式: 比例阀实际动作由远程指令驱动，本地算法不覆盖输出
                u_outputs[k] = t_k.valve_position_pct / 10.0

            # 滚动示波器历史缓存
            self.target_hist[k] = np.roll(self.target_hist[k], -1)
            self.target_hist[k][-1] = self.target_pos_mm[k]
            self.stroke_hist[k] = np.roll(self.stroke_hist[k], -1)
            self.stroke_hist[k][-1] = x_k
            self.pA_hist[k] = np.roll(self.pA_hist[k], -1)
            self.pA_hist[k][-1] = t_k.p_A_bar
            self.pB_hist[k] = np.roll(self.pB_hist[k], -1)
            self.pB_hist[k][-1] = t_k.p_B_bar
            self.pacc_hist[k] = np.roll(self.pacc_hist[k], -1)
            self.pacc_hist[k][-1] = t_k.p_acc_bar
            self.pSupply_hist[k] = np.roll(self.pSupply_hist[k], -1)
            self.pSupply_hist[k][-1] = t_k.p_supply_bar

            # 刷新对应叶片选项卡
            self.face_tabs[k].update_telemetry(
                t_k, self.is_feathering[k],
                self.time_hist, self.target_hist[k], self.stroke_hist[k],
                self.pA_hist[k], self.pB_hist[k], self.pacc_hist[k], self.pSupply_hist[k]
            )

        # 2. 刷新左侧面板
        active_t = telems[self.active_blade]
        self.left_panel.update_telemetry(active_t, active_t.is_locked, u_outputs[self.active_blade])

        # 3. 远程控制状态与心跳更新
        if self.control_mode == 'REMOTE':
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge and self.opcua_bridge.is_connected:
                hb_plc = self.opcua_bridge.heartbeat_plc
                hb_sim = self.opcua_bridge.heartbeat_sim
                self.lbl_auth_status.setText(f"🟢 远程在线 (PLC: {hb_plc} | Sim: {hb_sim})")
                self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
            else:
                self.lbl_auth_status.setText("🔄 连接中 (4840)...")
                self.lbl_auth_status.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #2B2317; border-radius: 4px; border: 1px solid #5A4015;")

        # 4. 刷新润滑系统选项卡
        lub_tele = self.engine.get_lub_telemetry()
        self.lub_tab.update_telemetry(lub_tele, self.time_hist)

        # 5. 刷新温度监控选项卡
        tmp_tele = self.engine.get_temp_telemetry()
        self.temp_tab.update_telemetry(tmp_tele, self.time_hist)

        # 6. 顶部时钟与抖动更新
        self.lbl_clock_stat.setText(
            f"时钟: {active_t.timestamp:5.1f}s | 周期: {active_t.cycle_time_ms:4.1f}ms | 抖动: {active_t.jitter_ms:4.1f}ms"
        )

    def closeEvent(self, event):
        self.timer.stop()
        if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
            self.opcua_bridge.stop()
        self.engine.stop()
        event.accept()
