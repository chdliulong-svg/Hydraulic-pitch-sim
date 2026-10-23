"""
=============================================================================
兆瓦级风电变桨液压系统 - PyQt6 软实时工业数字孪生监控台
=============================================================================
联动与闭锁功能升级:
  1. 40/20X60 变桨机械锁紧销油缸物理建模与真实流体力学求解 (阀130, 阀132, 节流孔131)
  2. 机械互锁物理约束:
     - 变桨锁未解锁 (插销到位) 时，主油缸实施强制刚性闭锁，速度/加速度清零，禁止动作
     - 90°顺桨限位开关 129 联动: 仅当主缸顺桨到位 (810mm~822mm) 时锁孔对齐，才允许机械插销
  3. 控制界面重构:
     - 新增独立【变桨机械锁紧装置 (40/20X60)】控制台 (支持一键拔销解锁 / 机械插销锁紧)
     - 拓扑图直观动态绘制 40/20X60 锁紧缸、机械插销下压锁紧、传感器 129 与开关 S
     - 动态安全警示横幅: 机械锁死状态下强行操作变桨时高亮告警
=============================================================================
"""

import sys
import math
from collections import deque
import numpy as np

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QScrollArea,
    QGroupBox, QDoubleSpinBox, QSplitter, QFrame, QGridLayout,
    QTabWidget, QCheckBox, QSizePolicy, QProgressBar
)
from PyQt6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QLinearGradient
)
from PyQt6.QtCore import Qt, QTimer

import pyqtgraph as pg
from pitch_soft_realtime import SoftRealtimePitchEngine
from pitch_opcua_bridge import PitchOpcUaBridge
from pitch_conn_dialog import ConnectionSettingsDialog, load_connection_config


# =============================================================================
# 1. 自定义高精度动态机械/液压拓扑渲染画布 (含 40/20X60 变桨机械锁)
# =============================================================================
class HydraulicSchematicWidget(QWidget):
    def __init__(self, face_name="A", parent=None):
        super().__init__(parent)
        self.face_name = face_name
        self.setFixedHeight(285)
        self.setStyleSheet("background-color: #14151B; border-radius: 8px;")

        # 主油缸状态
        self.stroke_mm = 50.0
        self.stroke_max = 822.0
        self.p_A_bar = 15.0
        self.p_B_bar = 15.0
        self.p_acc_bar = 275.0
        self.valve_pos_pct = 0.0
        self.is_feathering = False

        # 调速孔径
        self.d_121_mm = 4.7
        self.d_122_mm = 3.0

        # 40/20X60 变桨锁状态
        self.lock_pin_mm = 0.0          # 0.0 = 拔销解锁, 60.0 = 锁紧
        self.is_locked = False
        self.switch_129 = False         # 90° 对准
        self.switch_S_unlocked = True
        self.p_PP_bar = 260.0

        # 液压动力站 (HPU) 状态
        self.valve_25_state = False     # 25# 电磁阀打压/卸荷
        self.p_supply_bar = 2.5         # P口供油压力
        self.q_hpu_lpm = 0.0            # 供油流量 L/min

    def update_telemetry(self, stroke_mm, p_A, p_B, p_acc, valve_pos_pct, is_feathering,
                         lock_pin_mm, is_locked, switch_129, switch_S_unlocked, p_PP_bar,
                         valve_25_state=False, p_supply_bar=2.5, q_hpu_lpm=0.0):
        self.stroke_mm = stroke_mm
        self.p_A_bar = p_A
        self.p_B_bar = p_B
        self.p_acc_bar = p_acc
        self.valve_pos_pct = valve_pos_pct
        self.is_feathering = is_feathering

        self.lock_pin_mm = lock_pin_mm
        self.is_locked = is_locked
        self.switch_129 = switch_129
        self.switch_S_unlocked = switch_S_unlocked
        self.p_PP_bar = p_PP_bar
        self.valve_25_state = valve_25_state
        self.p_supply_bar = p_supply_bar
        self.q_hpu_lpm = q_hpu_lpm
        self.update()

    def _p_to_color(self, p_bar):
        ratio = max(0.0, min(1.0, p_bar / 260.0))
        r = int(35 + ratio * 205)
        g = int(120 - ratio * 80)
        b = int(225 - ratio * 185)
        return QColor(r, g, b, 220)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # 1. 顶部状态横幅 (支持安全顺桨 & 机械锁紧综合报警，带半透明底色卡片，避免遮挡)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor("#1A1C24")))
        painter.drawRoundedRect(12, 6, max(600, w - 24), 26, 4, 4)

        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        tag = f"【叶片 {self.face_name}】"
        if self.is_locked:
            painter.setPen(QColor("#FFB703"))
            painter.drawText(
                22, 23,
                f"{tag} 🔒 机械锁已锁死：锁销插入闭锁孔，主油缸闭锁禁止动作"
            )
        elif self.is_feathering:
            painter.setPen(QColor("#EF233C"))
            painter.drawText(
                22, 23,
                f"{tag} 🛑 应急顺桨中：蓄能器充液顺桨，快放回路高速泄压"
            )
        else:
            painter.setPen(QColor("#06D6A0"))
            painter.drawText(
                22, 23,
                f"{tag} 🟢 正常运行：比例阀伺服微调控制，双缸并联驱动"
            )

        # 2. 坐标布局
        start_x = 90
        cyl_w = max(200, min(240, int((w - 320) / 2)))
        cyl_h = 40
        c1_y = 90
        c2_y = 155

        ratio = max(0.0, min(1.0, self.stroke_mm / self.stroke_max))
        piston_x = start_x + 10 + ratio * (cyl_w - 50)

        col_A = self._p_to_color(self.p_A_bar)
        col_B = self._p_to_color(self.p_B_bar)

        # 绘制主缸 1 与主缸 2
        for cy, name in [(c1_y, "主油缸 1"), (c2_y, "主油缸 2")]:
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(col_A))
            painter.drawRect(int(start_x + 3), int(cy + 3), int(piston_x - start_x - 3), int(cyl_h - 6))

            painter.setBrush(QBrush(col_B))
            painter.drawRect(int(piston_x + 16), int(cy + 3), int(start_x + cyl_w - piston_x - 19), int(cyl_h - 6))

            # 活塞
            piston_grad = QLinearGradient(piston_x, cy, piston_x + 16, cy)
            piston_grad.setColorAt(0.0, QColor("#8D99AE"))
            piston_grad.setColorAt(0.5, QColor("#EDF2F4"))
            piston_grad.setColorAt(1.0, QColor("#6C757D"))
            painter.setBrush(QBrush(piston_grad))
            painter.setPen(QPen(QColor("#EDF2F4"), 1))
            painter.drawRoundedRect(int(piston_x), int(cy + 2), 16, int(cyl_h - 4), 2, 2)

            # 活塞杆
            rod_len = (cyl_w - 40)
            rod_end_x = piston_x + 16 + rod_len
            rod_grad = QLinearGradient(0, cy + 12, 0, cy + cyl_h - 12)
            rod_grad.setColorAt(0.0, QColor("#E0E1DD"))
            rod_grad.setColorAt(0.5, QColor("#FFFFFF"))
            rod_grad.setColorAt(1.0, QColor("#ADB5BD"))
            painter.setBrush(QBrush(rod_grad))
            painter.drawRect(int(piston_x + 16), int(cy + 12), int(rod_end_x - (piston_x + 16)), int(cyl_h - 24))

            # 缸筒外壳
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.setPen(QPen(QColor("#CED4DA"), 2.5))
            painter.drawRect(int(start_x), int(cy), int(cyl_w), int(cyl_h))

            # 标签
            painter.setPen(QColor("#A8DADC"))
            painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
            painter.drawText(18, cy + 25, name)

        # 双杆机械同步连杆 (带有机械锁孔)
        link_x = piston_x + 16 + (cyl_w - 40)
        painter.setPen(QPen(QColor("#FFD166"), 6))
        painter.drawLine(int(link_x), int(c1_y + 20), int(link_x), int(c2_y + 20))

        # 铰支轴承
        painter.setPen(QPen(QColor("#EDF2F4"), 2))
        painter.setBrush(QBrush(QColor("#06D6A0")))
        mid_cy = int((c1_y + c2_y) / 2 + 20)
        painter.drawEllipse(int(link_x - 7), mid_cy - 7, 14, 14)
        painter.setPen(QColor("#06D6A0"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        painter.drawText(int(link_x + 14), mid_cy + 5, "▶ 驱动叶片")

        # -------------------------------------------------------------
        # 3. 动态绘制变桨机械锁紧装置与锁销动作
        # -------------------------------------------------------------
        lock_stand_x = start_x + 10 + (cyl_w - 50) + 16 + (cyl_w - 40) # 90°顺桨时连杆所在固定位置
        lock_box_x = int(lock_stand_x - 14)
        lock_box_y = c1_y - 44
        lock_box_w = 28
        lock_box_h = 36

        # 销轴伸缩 (0~60mm 映射为 0~24px)
        pin_travel_px = int((self.lock_pin_mm / 60.0) * 24)
        pin_tip_y = lock_box_y + lock_box_h + pin_travel_px

        # 锁油缸外壳
        painter.setPen(QPen(QColor("#FFD166"), 1.8))
        painter.setBrush(QBrush(QColor("#242734")))
        painter.drawRoundedRect(lock_box_x, lock_box_y, lock_box_w, lock_box_h, 3, 3)

        # 内置弹簧示意
        painter.setPen(QPen(QColor("#06D6A0" if self.lock_pin_mm < 10 else "#FFB703"), 1.5))
        for sp_y in range(lock_box_y + 6, lock_box_y + 24, 4):
            painter.drawLine(lock_box_x + 4, sp_y, lock_box_x + lock_box_w - 4, sp_y + 2)

        # 锁紧销
        painter.setPen(QPen(QColor("#EDF2F4"), 1))
        painter.setBrush(QBrush(QColor("#E0E1DD" if not self.is_locked else "#EF233C")))
        painter.drawRect(lock_box_x + 8, lock_box_y + 12, 12, lock_box_h - 12 + pin_travel_px)

        # 锁孔套筒
        painter.setPen(QPen(QColor("#8D99AE"), 2))
        painter.drawRect(lock_box_x + 5, c1_y - 2, 18, 14)

        # 锁状态标志
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        if self.is_locked:
            painter.setPen(QColor("#EF233C"))
            painter.drawText(lock_box_x - 15, lock_box_y - 6, "🔒 已锁死")
        elif self.switch_129:
            painter.setPen(QColor("#06D6A0"))
            painter.drawText(lock_box_x - 20, lock_box_y - 6, "🟢 对准可锁")
        else:
            painter.setPen(QColor("#A8DADC"))
            painter.drawText(lock_box_x - 15, lock_box_y - 6, "🔓 已解锁")

        # 锁缸标注
        painter.setPen(QColor("#EDF2F4"))
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(lock_box_x + 34, lock_box_y + 20, "机械锁缸")

        # 油口标注
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(start_x + 8, c1_y - 8, "A口 (无杆腔)")
        painter.drawText(start_x + cyl_w - 78, c1_y - 8, "B口 (有杆腔)")

        # -------------------------------------------------------------
        # 4. 蓄能器组
        # -------------------------------------------------------------
        acc_x = max(int(lock_box_x + 120), w - 105)
        acc_y = 65
        painter.setPen(QPen(QColor("#FFD166"), 2))
        painter.setBrush(QBrush(QColor("#2B2D42")))
        painter.drawEllipse(acc_x, acc_y, 56, 56)
        painter.drawRect(acc_x + 10, acc_y + 44, 36, 46)

        acc_ratio = max(0.0, min(1.0, (self.p_acc_bar - 100.0) / 175.0))
        oil_h = int(acc_ratio * 40)
        painter.setBrush(QBrush(QColor("#FB8500")))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRect(acc_x + 12, acc_y + 46 + (40 - oil_h), 32, oil_h)

        painter.setPen(QColor("#EDF2F4"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(acc_x + 10, acc_y + 26, "蓄能器组")
        painter.setPen(QColor("#FFD166"))
        painter.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        painter.drawText(acc_x + 2, acc_y + 112, f"{self.p_acc_bar:.1f} bar")

        # -------------------------------------------------------------
        # 5. 底部阀组拓扑卡片
        # -------------------------------------------------------------
        valve_box_y = 220
        # 比例伺服阀
        v_col = QColor("#06D6A0") if not self.is_feathering and not self.is_locked else QColor("#8D99AE")
        painter.setPen(QPen(v_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(start_x, valve_box_y, 125, 50, 4, 4)
        painter.setPen(v_col)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 14, valve_box_y + 20, "比例伺服阀")
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(start_x + 20, valve_box_y + 38, f"开度: {self.valve_pos_pct:+.1f} %")

        # 应急顺桨阀组
        f_col = QColor("#EF233C") if self.is_feathering else QColor("#4A4E69")
        painter.setPen(QPen(f_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(start_x + 135, valve_box_y, 125, 50, 4, 4)
        painter.setPen(QColor("#EDF2F4"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 143, valve_box_y + 20, "应急顺桨回路")
        painter.setFont(QFont("Segoe UI", 8))
        painter.setPen(QColor("#8D99AE") if not self.is_feathering else QColor("#FF4D6D"))
        painter.drawText(start_x + 143, valve_box_y + 38, "充液 / 快放阀")

        # 锁缸控制阀
        lock_valv_col = QColor("#06D6A0") if self.switch_S_unlocked else QColor("#FFB703")
        painter.setPen(QPen(lock_valv_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(start_x + 270, valve_box_y, 125, 50, 4, 4)
        painter.setPen(lock_valv_col)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 280, valve_box_y + 20, "锁缸控制阀")
        painter.setFont(QFont("Consolas", 9))
        painter.setPen(QColor("#EDF2F4"))
        painter.drawText(start_x + 280, valve_box_y + 38, f"锁压: {self.p_PP_bar:4.1f} bar")

        # 液压动力源 (HPU)
        hpu_box_x = start_x + 405
        hpu_box_w = max(180, w - hpu_box_x - 15)
        hpu_border_col = QColor("#06D6A0") if self.valve_25_state else QColor("#4A4E69")
        painter.setPen(QPen(hpu_border_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(hpu_box_x, valve_box_y, hpu_box_w, 50, 4, 4)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        hpu_mode_txt = "🟢 打压供油" if self.valve_25_state else "⚪ 卸荷循环"
        painter.setPen(hpu_border_col)
        painter.drawText(hpu_box_x + 10, valve_box_y + 20, f"动力源: {hpu_mode_txt}")

        painter.setFont(QFont("Consolas", 8))
        painter.setPen(QColor("#4CC9F0"))
        painter.drawText(hpu_box_x + 10, valve_box_y + 38, f"供油: {self.p_supply_bar:4.1f} bar | {self.q_hpu_lpm:4.1f} L/min")


# =============================================================================
# 2. PyQt6 主窗口与变桨机械锁联动控制台
# =============================================================================
class PitchMainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("兆瓦级风电液压变桨系统 - PyQt6 软实时工业数字孪生")
        self.resize(1440, 940)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #16171D;
            }
            QLabel {
                color: #EDF2F4;
                font-family: 'Segoe UI', 'Microsoft YaHei';
            }
            QGroupBox {
                border: 1px solid #363A4A;
                border-radius: 8px;
                margin-top: 10px;
                font-weight: bold;
                color: #A8DADC;
                font-size: 12px;
                background-color: #21232C;
                padding-top: 6px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
                background-color: #21232C;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #323544;
                border-radius: 4px;
            }
            QSlider::sub-page:horizontal {
                background: #4CC9F0;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #EDF2F4;
                border: 2px solid #4CC9F0;
                width: 18px;
                margin-top: -6px;
                margin-bottom: -6px;
                border-radius: 9px;
            }
            QDoubleSpinBox {
                background-color: #2B2E3B;
                color: #EDF2F4;
                border: 1px solid #4E5266;
                border-radius: 5px;
                padding: 4px 8px;
                font-family: Consolas;
                font-size: 13px;
                font-weight: bold;
            }
        """)

        # 机构面配置 (A面 / B面 / C面)
        self.blades = ['A', 'B', 'C']
        self.active_blade = 'A'
        self.is_sync_all = False

        # 控制权限模式: 'LOCAL' (本地控制, 默认) 或 'REMOTE' (远程控制)
        self.control_mode = 'LOCAL'

        # 启动底层 200Hz (5ms) 超高速软实时多面液压仿真内核
        self.engine = SoftRealtimePitchEngine(cycle_time_s=0.005)
        self.engine.start()

        # 读取持久化 OPC UA 连接参数配置
        self.conn_config = load_connection_config()
        self.opcua_endpoint = self.conn_config.get("endpoint", "opc.tcp://127.0.0.1:4840")
        self.opcua_cycle_time_s = float(self.conn_config.get("cycle_time_ms", 5.0)) / 1000.0

        # 初始化 CODESYS OPC UA 桥接网关 (微周期纳秒级对齐，选择远程控制时自动开启通讯)
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

        # 变桨锁控制电平 (初始状态: 拔销解锁 130=1, 132=1)
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
        self.actual_hist = {k: np.full(self.buf_size, 50.0) for k in self.blades}
        self.pA_hist = {k: np.full(self.buf_size, 15.0) for k in self.blades}
        self.pB_hist = {k: np.full(self.buf_size, 15.0) for k in self.blades}
        self.pacc_hist = {k: np.full(self.buf_size, 260.0) for k in self.blades}
        self.pSupply_hist = {k: np.full(self.buf_size, 2.5) for k in self.blades}

        # 6MW 轮毂润滑系统示波器历史缓存 (300点)
        self.lub_p1_hist = np.zeros(self.buf_size)
        self.lub_p2_hist = np.zeros(self.buf_size)
        self.lub_piston1_a_hist = np.zeros(self.buf_size)
        self.lub_piston1_b_hist = np.zeros(self.buf_size)
        self.lub_piston1_c_hist = np.zeros(self.buf_size)

        # 6MW 轮毂温度热力学示波器历史缓存 (300点)
        self.temp_oil_hist = np.full(self.buf_size, 42.5)
        self.temp_cab_hist = np.full(self.buf_size, 32.0)
        self.temp_hub_hist = np.full(self.buf_size, 25.0)
        self.temp_bearing_hist = np.full(self.buf_size, 35.0)
        self.temp_cpu_hist = np.full(self.buf_size, 45.0)

        self._init_ui()

        # UI 50FPS 刷新
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_ui_tick)
        self.timer.start(20)

    def _init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QVBoxLayout(main_widget)
        main_layout.setContentsMargins(14, 10, 14, 10)
        main_layout.setSpacing(10)

        # --- 顶部紧凑导航与状态栏 ---
        top_bar = QFrame()
        top_bar.setStyleSheet("background-color: #21232C; border-radius: 6px; border: 1px solid #363A4A;")
        top_layout = QHBoxLayout(top_bar)
        top_layout.setContentsMargins(14, 5, 14, 5)
        top_layout.setSpacing(12)

        title_lbl = QLabel("⚡ 变桨液压仿真系统")
        title_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #EDF2F4;")
        top_layout.addWidget(title_lbl)

        sep1 = QFrame()
        sep1.setFrameShape(QFrame.Shape.VLine)
        sep1.setStyleSheet("color: #3D4357; max-height: 20px;")
        top_layout.addWidget(sep1)

        # 控制权限切换 (置顶全局控制)
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
        self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px;")
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

        # --- 中间分割器 ---
        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.setHandleWidth(8)

        # 左侧控制面板 (独立滚动区)
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)
        scroll_area.setStyleSheet("background-color: transparent;")
        scroll_area.setMinimumWidth(380)

        left_panel = self._build_left_panel()
        scroll_area.setWidget(left_panel)
        splitter.addWidget(scroll_area)

        # 右侧显示面板
        right_panel = self._build_right_panel()
        splitter.addWidget(right_panel)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([380, 1060])
        main_layout.addWidget(splitter)

    def _build_left_panel(self):
        panel = QWidget()
        panel.setStyleSheet("background-color: transparent;")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 8, 8)
        layout.setSpacing(8)

        # -------------------------------------------------------------
        # 1. 叶片选择
        # -------------------------------------------------------------
        face_box = QGroupBox(" 叶片选择 ")
        face_layout = QHBoxLayout(face_box)
        face_layout.setContentsMargins(8, 6, 8, 6)
        face_layout.setSpacing(6)

        self.btn_faces = {}
        for k, text in [('A', '叶片 A'), ('B', '叶片 B'), ('C', '叶片 C')]:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setFixedHeight(28)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _, key=k: self._select_face(key))
            self.btn_faces[k] = b
            face_layout.addWidget(b)

        self.chk_sync_all = QCheckBox("三叶片同步")
        self.chk_sync_all.setStyleSheet("""
            QCheckBox {
                color: #FFD166;
                font-size: 11px;
                font-weight: bold;
                spacing: 5px;
            }
            QCheckBox::indicator {
                width: 14px;
                height: 14px;
                border: 1px solid #4E5266;
                border-radius: 3px;
                background-color: #262936;
            }
            QCheckBox::indicator:checked {
                background-color: #06D6A0;
                border-color: #06D6A0;
            }
        """)
        self.chk_sync_all.stateChanged.connect(self._on_sync_changed)
        face_layout.addWidget(self.chk_sync_all)

        layout.addWidget(face_box)

        # -------------------------------------------------------------
        # 2. 运动控制
        # -------------------------------------------------------------
        self.motion_box = QGroupBox(" 运动控制 ")
        motion_layout = QVBoxLayout(self.motion_box)
        motion_layout.setContentsMargins(8, 8, 8, 8)
        motion_layout.setSpacing(6)

        # 模式切换分段按钮
        mode_btn_row = QHBoxLayout()
        mode_btn_row.setSpacing(6)
        self.btn_mode_auto = QPushButton("闭环位置 (PID)")
        self.btn_mode_auto.setCheckable(True)
        self.btn_mode_auto.setChecked(True)
        self.btn_mode_auto.setFixedHeight(30)
        self.btn_mode_auto.setCursor(Qt.CursorShape.PointingHandCursor)

        self.btn_mode_manual = QPushButton("手动开环 (±10V)")
        self.btn_mode_manual.setCheckable(True)
        self.btn_mode_manual.setChecked(False)
        self.btn_mode_manual.setFixedHeight(30)
        self.btn_mode_manual.setCursor(Qt.CursorShape.PointingHandCursor)

        self.btn_mode_auto.clicked.connect(lambda: self._set_mode(True))
        self.btn_mode_manual.clicked.connect(lambda: self._set_mode(False))
        mode_btn_row.addWidget(self.btn_mode_auto)
        mode_btn_row.addWidget(self.btn_mode_manual)
        motion_layout.addLayout(mode_btn_row)

        # --- 自动模式子面板 ---
        self.widget_auto_ctrl = QWidget()
        auto_layout = QVBoxLayout(self.widget_auto_ctrl)
        auto_layout.setContentsMargins(0, 2, 0, 0)
        auto_layout.setSpacing(5)

        t_row1 = QHBoxLayout()
        lbl_target_tip = QLabel("目标位移:")
        lbl_target_tip.setStyleSheet("color: #A8DADC; font-size: 11px; font-weight: bold;")
        self.slider_target = QSlider(Qt.Orientation.Horizontal)
        self.slider_target.setRange(0, 822)
        self.slider_target.setValue(150)

        self.spin_target = QDoubleSpinBox()
        self.spin_target.setRange(0.0, 822.0)
        self.spin_target.setValue(150.0)
        self.spin_target.setSuffix(" mm")
        self.spin_target.setFixedWidth(90)
        self.spin_target.setStyleSheet("background-color: #2B2E3B; color: #4CC9F0; font-weight: bold; border: 1px solid #4E5266; border-radius: 4px; padding: 2px 4px; font-family: Consolas; font-size: 12px;")

        self.slider_target.valueChanged.connect(lambda v: self.spin_target.setValue(float(v)))
        self.spin_target.valueChanged.connect(self._on_target_changed)

        t_row1.addWidget(lbl_target_tip)
        t_row1.addWidget(self.slider_target)
        t_row1.addWidget(self.spin_target)
        auto_layout.addLayout(t_row1)

        t_row2 = QHBoxLayout()
        t_row2.setSpacing(4)
        for preset_mm, label in [(0.0, "0 mm"), (150.0, "150 mm"), (400.0, "400 mm"), (822.0, "822 mm")]:
            btn = QPushButton(label)
            btn.setFixedHeight(24)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2C303E;
                    color: #A8DADC;
                    border: 1px solid #44495C;
                    border-radius: 3px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #3D4357;
                    color: #FFFFFF;
                }
            """)
            btn.clicked.connect(lambda _, p=preset_mm: self.spin_target.setValue(p))
            t_row2.addWidget(btn)
        auto_layout.addLayout(t_row2)
        motion_layout.addWidget(self.widget_auto_ctrl)

        # --- 手动模式子面板 ---
        self.widget_manual_ctrl = QWidget()
        manual_layout = QVBoxLayout(self.widget_manual_ctrl)
        manual_layout.setContentsMargins(0, 2, 0, 0)
        manual_layout.setSpacing(5)

        m_row1 = QHBoxLayout()
        lbl_manual_tip = QLabel("阀控电压:")
        lbl_manual_tip.setStyleSheet("color: #FFD166; font-size: 11px; font-weight: bold;")
        self.slider_voltage = QSlider(Qt.Orientation.Horizontal)
        self.slider_voltage.setRange(-100, 100)
        self.slider_voltage.setValue(0)

        self.spin_voltage = QDoubleSpinBox()
        self.spin_voltage.setRange(-10.0, 10.0)
        self.spin_voltage.setSingleStep(0.1)
        self.spin_voltage.setValue(0.0)
        self.spin_voltage.setSuffix(" V")
        self.spin_voltage.setFixedWidth(90)
        self.spin_voltage.setStyleSheet("background-color: #2B2E3B; color: #FFD166; font-weight: bold; border: 1px solid #4E5266; border-radius: 4px; padding: 2px 4px; font-family: Consolas; font-size: 12px;")

        self.slider_voltage.valueChanged.connect(lambda v: self.spin_voltage.setValue(v / 10.0))
        self.spin_voltage.valueChanged.connect(self._on_voltage_changed)

        m_row1.addWidget(lbl_manual_tip)
        m_row1.addWidget(self.slider_voltage)
        m_row1.addWidget(self.spin_voltage)
        manual_layout.addLayout(m_row1)

        m_row2 = QHBoxLayout()
        m_row2.setSpacing(4)
        for val_v, label in [(-10.0, "-10 V"), (-5.0, "-5 V"), (0.0, "0 V"), (5.0, "+5 V"), (10.0, "+10 V")]:
            btn = QPushButton(label)
            btn.setFixedHeight(24)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2C303E;
                    color: #FFD166;
                    border: 1px solid #44495C;
                    border-radius: 3px;
                    font-size: 11px;
                }
                QPushButton:hover {
                    background-color: #3D4357;
                    color: #FFFFFF;
                }
            """)
            btn.clicked.connect(lambda _, v=val_v: self.spin_voltage.setValue(v))
            m_row2.addWidget(btn)
        manual_layout.addLayout(m_row2)
        motion_layout.addWidget(self.widget_manual_ctrl)

        # 默认只显示自动控制子面板
        self.widget_manual_ctrl.setVisible(False)
        layout.addWidget(self.motion_box)

        # -------------------------------------------------------------
        # 3. 变桨机械锁
        # -------------------------------------------------------------
        self.lock_box = QGroupBox(" 变桨机械锁 ")
        lock_layout = QVBoxLayout(self.lock_box)
        lock_layout.setContentsMargins(8, 8, 8, 8)
        lock_layout.setSpacing(6)

        lock_btn_row = QHBoxLayout()
        lock_btn_row.setSpacing(8)
        self.btn_lock_release = QPushButton("🔓 拔销解锁")
        self.btn_lock_release.setFixedHeight(30)
        self.btn_lock_release.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_release.clicked.connect(self._cmd_unlock)

        self.btn_lock_engage = QPushButton("🔒 插销锁定")
        self.btn_lock_engage.setFixedHeight(30)
        self.btn_lock_engage.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_engage.clicked.connect(self._cmd_lock)

        lock_btn_row.addWidget(self.btn_lock_release)
        lock_btn_row.addWidget(self.btn_lock_engage)
        lock_layout.addLayout(lock_btn_row)

        self.lbl_lock_detail = QLabel("状态: 🟢 已解锁 | 90°对齐: 否")
        self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        lock_layout.addWidget(self.lbl_lock_detail)

        layout.addWidget(self.lock_box)

        # -------------------------------------------------------------
        # 4. 应急顺桨
        # -------------------------------------------------------------
        self.safe_box = QGroupBox(" 应急顺桨 ")
        safe_layout = QVBoxLayout(self.safe_box)
        safe_layout.setContentsMargins(8, 8, 8, 8)
        safe_layout.setSpacing(6)

        self.btn_feather = QPushButton("🛑 模拟紧急顺桨")
        self.btn_feather.setFixedHeight(32)
        self.btn_feather.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_feather.clicked.connect(self._on_toggle_feather)
        safe_layout.addWidget(self.btn_feather)

        self.lbl_safe_status = QLabel("状态: 正常运行")
        self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        safe_layout.addWidget(self.lbl_safe_status)
        layout.addWidget(self.safe_box)

        # -------------------------------------------------------------
        # 5. 实时遥测
        # -------------------------------------------------------------
        tele_box = QGroupBox(" 实时遥测 ")
        tele_layout = QGridLayout(tele_box)
        tele_layout.setContentsMargins(8, 8, 8, 8)
        tele_layout.setSpacing(5)

        self.tele_labels = {}
        fields = [
            ("disp", "缸位移:", "0.0 mm", "#4CC9F0"),
            ("vel",  "线速度:", "+0.0 mm/s", "#06D6A0"),
            ("u_v",  "阀电压:", "0.00 V", "#FFD166"),
            ("spool","阀开度:", "0.0 %", "#FFD166"),
            ("pA",   "A腔压力:", "15.0 bar", "#EF233C"),
            ("pB",   "B腔压力:", "15.0 bar", "#48CAE4"),
            ("pacc", "蓄能器压:", "260.0 bar", "#FFB703"),
            ("pPP",  "锁缸压力:", "260.0 bar", "#06D6A0"),
            ("v25",  "25#阀:",   "失电(卸荷)", "#A8DADC"),
            ("pP",   "泵口压力:", "2.5 bar", "#4CC9F0"),
        ]

        for i, (key, title, default_val, col) in enumerate(fields):
            row = i // 2
            col_offset = (i % 2) * 2

            t_lbl = QLabel(title)
            t_lbl.setStyleSheet("color: #8D99AE; font-size: 11px;")
            v_lbl = QLabel(default_val)
            v_lbl.setStyleSheet(f"color: {col}; font-family: Consolas; font-size: 12px; font-weight: bold;")

            tele_layout.addWidget(t_lbl, row, col_offset)
            tele_layout.addWidget(v_lbl, row, col_offset + 1)
            self.tele_labels[key] = v_lbl

        layout.addWidget(tele_box)
        layout.addStretch()

        self._update_face_buttons_style()
        self._update_mode_styles()
        self._update_feather_button_style()
        self._update_lock_buttons_style()

        return panel

    def _build_right_panel(self):
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(4, 0, 0, 0)
        layout.setSpacing(6)

        # 3 面独立可切换选项卡
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("""
            QTabWidget::pane {
                border: 1px solid #363A4A;
                border-radius: 6px;
                background-color: #16171D;
                top: -1px;
            }
            QTabBar::tab {
                background-color: #21232C;
                color: #8D99AE;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 22px;
                border: 1px solid #363A4A;
                border-bottom: none;
                border-top-left-radius: 6px;
                border-top-right-radius: 6px;
                margin-right: 4px;
            }
            QTabBar::tab:selected {
                background-color: #05668D;
                color: #FFFFFF;
                border: 1.5px solid #4CC9F0;
                border-bottom: 1.5px solid #05668D;
            }
            QTabBar::tab:hover:!selected {
                background-color: #2D313E;
                color: #EDF2F4;
            }
        """)

        self.schematic_widgets = {}
        self.plot_widgets = {}
        self.curves_target = {}
        self.curves_actual = {}
        self.curves_pA = {}
        self.curves_pB = {}
        self.curves_pacc = {}
        self.curves_pSupply = {}

        tabs_info = [
            ('A', '叶片 A'),
            ('B', '叶片 B'),
            ('C', '叶片 C')
        ]

        for k, face_title in tabs_info:
            tab_page = self._build_face_tab_page(k)
            self.tab_widget.addTab(tab_page, face_title)

        # 4. 6MW 轮毂递进式集中润滑系统选项卡
        self.lub_tab_page = self._build_lubrication_tab_page()
        self.tab_widget.addTab(self.lub_tab_page, "润滑系统")

        # 5. 6MW 轮毂温度热力学仿真监控选项卡
        self.temp_tab_page = self._build_temperature_tab_page()
        self.tab_widget.addTab(self.temp_tab_page, "温度监控")

        self.tab_widget.currentChanged.connect(self._on_tab_changed)
        layout.addWidget(self.tab_widget)
        return panel

    def _build_face_tab_page(self, k):
        page = QWidget()
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(6, 6, 6, 6)
        page_layout.setSpacing(6)

        # 1. 动态拓扑图 (带 A/B/C 面标识)
        schematic = HydraulicSchematicWidget(face_name=k)
        self.schematic_widgets[k] = schematic
        page_layout.addWidget(schematic)

        # 2. pyqtgraph 示波器
        scope_box = QGroupBox(f" 叶片 {k} 实时运行曲线 ")
        scope_layout = QVBoxLayout(scope_box)
        scope_layout.setContentsMargins(6, 8, 6, 6)

        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground("#121318")
        self.plot_widgets[k] = plot_widget

        # 通道 1: 位移
        p1 = plot_widget.addPlot(row=0, col=0)
        p1.showGrid(x=True, y=True, alpha=0.3)
        p1.setLabel('left', '位移 (mm)', color='#4CC9F0')
        p1.setYRange(0, 850, padding=0.05)
        p1.addLegend(offset=(10, 10), brush="#1E202B")

        self.curves_target[k] = p1.plot(
            pen=pg.mkPen(color='#E76F51', width=2, style=Qt.PenStyle.DashLine),
            name="目标位移"
        )
        self.curves_actual[k] = p1.plot(
            pen=pg.mkPen(color='#4CC9F0', width=2.5),
            name="实际行程"
        )

        # 通道 2: 压力
        p2 = plot_widget.addPlot(row=1, col=0)
        p2.showGrid(x=True, y=True, alpha=0.3)
        p2.setLabel('left', '压力 (bar)', color='#FFD166')
        p2.setLabel('bottom', '相对时间 (s)', color='#A8DADC')
        p2.setYRange(0, 300, padding=0.05)
        p2.addLegend(offset=(10, 10), brush="#1E202B")

        p2.addLine(y=235, pen=pg.mkPen(color='#EF233C', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=265, pen=pg.mkPen(color='#06D6A0', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=280, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DotLine))

        self.curves_pA[k] = p2.plot(pen=pg.mkPen(color='#EF233C', width=2), name="p_A (无杆腔)")
        self.curves_pB[k] = p2.plot(pen=pg.mkPen(color='#48CAE4', width=2), name="p_B (有杆腔)")
        self.curves_pacc[k] = p2.plot(
            pen=pg.mkPen(color='#FFB703', width=2, style=Qt.PenStyle.DashLine),
            name="p_acc (蓄能器)"
        )
        self.curves_pSupply[k] = p2.plot(
            pen=pg.mkPen(color='#4CC9F0', width=1.5, style=Qt.PenStyle.DotLine),
            name="p_P (供油口)"
        )

        scope_layout.addWidget(plot_widget)
        page_layout.addWidget(scope_box, stretch=1)

        return page

    def _build_lubrication_tab_page(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setStyleSheet("background-color: transparent;")

        page = QWidget()
        page.setStyleSheet("background-color: transparent;")
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(10, 8, 10, 8)
        page_layout.setSpacing(10)

        # 1. 顶部状态横幅
        banner = QFrame()
        banner.setObjectName("lubBanner")
        banner.setStyleSheet("""
            #lubBanner {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """)
        banner_layout = QHBoxLayout(banner)
        banner_layout.setContentsMargins(14, 8, 14, 8)
        banner_layout.setSpacing(12)

        title_vbox = QVBoxLayout()
        title_vbox.setSpacing(2)
        b_title = QLabel("6MW 轮毂递进式集中润滑系统动力学模拟")
        b_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #EDF2F4;")
        b_sub = QLabel("双泵独立回路 · 递进分配器活塞脉冲反馈 · 油箱液位动态消耗与故障仿真")
        b_sub.setStyleSheet("font-size: 11px; color: #8D99AE;")
        title_vbox.addWidget(b_title)
        title_vbox.addWidget(b_sub)
        banner_layout.addLayout(title_vbox)

        banner_layout.addStretch()

        self.lub_banner_pill1 = QLabel("1# 主回路: ⚪ 待机")
        self.lub_banner_pill1.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px;")
        banner_layout.addWidget(self.lub_banner_pill1)

        self.lub_banner_pill2 = QLabel("2# 备用回路: ⚪ 待机")
        self.lub_banner_pill2.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px;")
        banner_layout.addWidget(self.lub_banner_pill2)

        page_layout.addWidget(banner)

        # 2. 中部双回路卡片 (左右对称并列)
        circuits_layout = QHBoxLayout()
        circuits_layout.setSpacing(10)

        group_style = """
            QGroupBox {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
                margin-top: 10px;
                font-size: 12px;
                font-weight: bold;
                color: #EDF2F4;
                padding-top: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                background-color: #1E202B;
            }
        """
        bar_style = """
            QProgressBar {
                border: 1px solid #363A4A;
                border-radius: 4px;
                background-color: #16171D;
                text-align: center;
                color: #EDF2F4;
                font-family: Consolas;
                font-size: 11px;
                font-weight: bold;
                height: 18px;
            }
            QProgressBar::chunk {
                background-color: #06D6A0;
                border-radius: 3px;
            }
        """

        # -----------------------------
        # 1# 主回路卡片
        # -----------------------------
        box1 = QGroupBox(" 1# 主润滑回路 (pac.io.dOut.hubLubPump1On) ")
        box1.setStyleSheet(group_style)
        box1_layout = QVBoxLayout(box1)
        box1_layout.setContentsMargins(12, 10, 12, 10)
        box1_layout.setSpacing(8)

        # 泵电机状态 & 压力
        p1_row = QHBoxLayout()
        p1_row.addWidget(QLabel("泵状态:"))
        self.lbl_lub1_pump_status = QLabel("⚪ 停机")
        self.lbl_lub1_pump_status.setStyleSheet("color: #8D99AE; font-weight: bold; background-color: #21232C; padding: 2px 8px; border-radius: 4px; border: 1px solid #363A4A;")
        p1_row.addWidget(self.lbl_lub1_pump_status)
        p1_row.addStretch()
        p1_row.addWidget(QLabel("泵口压力:"))
        self.lbl_lub1_pressure = QLabel("  0.0 bar")
        self.lbl_lub1_pressure.setStyleSheet("font-family: Consolas; font-size: 13px; font-weight: bold; color: #4CC9F0;")
        p1_row.addWidget(self.lbl_lub1_pressure)
        box1_layout.addLayout(p1_row)

        self.bar_lub1_pressure = QProgressBar()
        self.bar_lub1_pressure.setStyleSheet(bar_style)
        self.bar_lub1_pressure.setRange(0, 250)
        self.bar_lub1_pressure.setValue(0)
        box1_layout.addWidget(self.bar_lub1_pressure)

        # 油箱液位
        tank1_row = QHBoxLayout()
        tank1_row.addWidget(QLabel("油脂余量:"))
        self.lbl_lub1_tank_level = QLabel(" 85.0 %")
        self.lbl_lub1_tank_level.setStyleSheet("font-family: Consolas; font-size: 12px; font-weight: bold; color: #FFD166;")
        tank1_row.addWidget(self.lbl_lub1_tank_level)
        tank1_row.addStretch()
        tank1_row.addWidget(QLabel("液位开关:"))
        self.lbl_lub1_tank_ok = QLabel("🟢 正常")
        self.lbl_lub1_tank_ok.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        tank1_row.addWidget(self.lbl_lub1_tank_ok)
        box1_layout.addLayout(tank1_row)

        self.bar_lub1_tank_level = QProgressBar()
        self.bar_lub1_tank_level.setStyleSheet(bar_style)
        self.bar_lub1_tank_level.setRange(0, 100)
        self.bar_lub1_tank_level.setValue(85)
        box1_layout.addWidget(self.bar_lub1_tank_level)

        # 分配器活塞循环反馈
        dist1_frame = QFrame()
        dist1_frame.setObjectName("dist1Frame")
        dist1_frame.setStyleSheet("""
            #dist1Frame {
                background-color: #16171D;
                border: 1px solid #363A4A;
                border-radius: 5px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """)
        dist1_vbox = QVBoxLayout(dist1_frame)
        dist1_vbox.setContentsMargins(6, 6, 6, 6)
        dist1_vbox.setSpacing(6)

        dist1_head = QHBoxLayout()
        d1_title = QLabel("递进分配器循环 (PistonMoving):")
        d1_title.setStyleSheet("font-size: 11px; font-weight: bold; color: #EDF2F4;")
        dist1_head.addWidget(d1_title)
        dist1_head.addStretch()
        self.lbl_lub1_cycles = QLabel("累计: 0 循环")
        self.lbl_lub1_cycles.setStyleSheet("font-family: Consolas; font-size: 11px; color: #A8DADC;")
        dist1_head.addWidget(self.lbl_lub1_cycles)
        dist1_vbox.addLayout(dist1_head)

        pistons1_row = QHBoxLayout()
        pistons1_row.setSpacing(6)
        self.lbl_lub1_piston_a = QLabel("⚪ 叶片A:待机")
        self.lbl_lub1_piston_b = QLabel("⚪ 叶片B:待机")
        self.lbl_lub1_piston_c = QLabel("⚪ 叶片C:待机")
        piston_lbl_style = "color: #8D99AE; font-size: 11px; font-weight: bold; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px; padding: 4px; qproperty-alignment: AlignCenter;"
        for lbl in [self.lbl_lub1_piston_a, self.lbl_lub1_piston_b, self.lbl_lub1_piston_c]:
            lbl.setStyleSheet(piston_lbl_style)
            pistons1_row.addWidget(lbl)
        dist1_vbox.addLayout(pistons1_row)
        box1_layout.addWidget(dist1_frame)

        # 测试与故障控制
        ctrl1_frame = QFrame()
        ctrl1_frame.setStyleSheet("background-color: transparent;")
        ctrl1_vbox = QVBoxLayout(ctrl1_frame)
        ctrl1_vbox.setContentsMargins(0, 4, 0, 0)
        ctrl1_vbox.setSpacing(6)

        self.btn_lub1_test_toggle = QPushButton("▶ 手动启动 1# 泵 (本地测试)")
        self.btn_lub1_test_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lub1_test_toggle.setStyleSheet("background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;")
        self.btn_lub1_test_toggle.clicked.connect(self._on_toggle_lub1_pump)
        ctrl1_vbox.addWidget(self.btn_lub1_test_toggle)

        faults1_row = QHBoxLayout()
        self.chk_lub1_low_level = QCheckBox("低油位故障")
        self.chk_lub1_low_level.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub1_low_level.toggled.connect(self._on_lub_faults_changed)

        self.chk_lub1_block = QCheckBox("分配器堵塞")
        self.chk_lub1_block.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub1_block.toggled.connect(self._on_lub_faults_changed)

        faults1_row.addWidget(self.chk_lub1_low_level)
        faults1_row.addWidget(self.chk_lub1_block)
        ctrl1_vbox.addLayout(faults1_row)
        box1_layout.addWidget(ctrl1_frame)

        circuits_layout.addWidget(box1)

        # -----------------------------
        # 2# 冗余回路卡片
        # -----------------------------
        box2 = QGroupBox(" 2# 冗余润滑回路 (pac.io.dOut.hubLubPump2On) ")
        box2.setStyleSheet(group_style)
        box2_layout = QVBoxLayout(box2)
        box2_layout.setContentsMargins(12, 10, 12, 10)
        box2_layout.setSpacing(8)

        # 泵电机状态 & 压力
        p2_row = QHBoxLayout()
        p2_row.addWidget(QLabel("泵状态:"))
        self.lbl_lub2_pump_status = QLabel("⚪ 停机")
        self.lbl_lub2_pump_status.setStyleSheet("color: #8D99AE; font-weight: bold; background-color: #21232C; padding: 2px 8px; border-radius: 4px; border: 1px solid #363A4A;")
        p2_row.addWidget(self.lbl_lub2_pump_status)
        p2_row.addStretch()
        p2_row.addWidget(QLabel("泵口压力:"))
        self.lbl_lub2_pressure = QLabel("  0.0 bar")
        self.lbl_lub2_pressure.setStyleSheet("font-family: Consolas; font-size: 13px; font-weight: bold; color: #FFD166;")
        p2_row.addWidget(self.lbl_lub2_pressure)
        box2_layout.addLayout(p2_row)

        self.bar_lub2_pressure = QProgressBar()
        self.bar_lub2_pressure.setStyleSheet(bar_style)
        self.bar_lub2_pressure.setRange(0, 250)
        self.bar_lub2_pressure.setValue(0)
        box2_layout.addWidget(self.bar_lub2_pressure)

        # 油箱液位
        tank2_row = QHBoxLayout()
        tank2_row.addWidget(QLabel("油脂余量:"))
        self.lbl_lub2_tank_level = QLabel(" 90.0 %")
        self.lbl_lub2_tank_level.setStyleSheet("font-family: Consolas; font-size: 12px; font-weight: bold; color: #FFD166;")
        tank2_row.addWidget(self.lbl_lub2_tank_level)
        tank2_row.addStretch()
        tank2_row.addWidget(QLabel("液位开关:"))
        self.lbl_lub2_tank_ok = QLabel("🟢 正常")
        self.lbl_lub2_tank_ok.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        tank2_row.addWidget(self.lbl_lub2_tank_ok)
        box2_layout.addLayout(tank2_row)

        self.bar_lub2_tank_level = QProgressBar()
        self.bar_lub2_tank_level.setStyleSheet(bar_style)
        self.bar_lub2_tank_level.setRange(0, 100)
        self.bar_lub2_tank_level.setValue(90)
        box2_layout.addWidget(self.bar_lub2_tank_level)

        # 分配器活塞循环反馈
        dist2_frame = QFrame()
        dist2_frame.setObjectName("dist2Frame")
        dist2_frame.setStyleSheet("""
            #dist2Frame {
                background-color: #16171D;
                border: 1px solid #363A4A;
                border-radius: 5px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """)
        dist2_vbox = QVBoxLayout(dist2_frame)
        dist2_vbox.setContentsMargins(6, 6, 6, 6)
        dist2_vbox.setSpacing(6)

        dist2_head = QHBoxLayout()
        d2_title = QLabel("递进分配器循环 (PistonMoving):")
        d2_title.setStyleSheet("font-size: 11px; font-weight: bold; color: #EDF2F4;")
        dist2_head.addWidget(d2_title)
        dist2_head.addStretch()
        self.lbl_lub2_cycles = QLabel("累计: 0 循环")
        self.lbl_lub2_cycles.setStyleSheet("font-family: Consolas; font-size: 11px; color: #A8DADC;")
        dist2_head.addWidget(self.lbl_lub2_cycles)
        dist2_vbox.addLayout(dist2_head)

        pistons2_row = QHBoxLayout()
        pistons2_row.setSpacing(6)
        self.lbl_lub2_piston_a = QLabel("⚪ 叶片A:待机")
        self.lbl_lub2_piston_b = QLabel("⚪ 叶片B:待机")
        self.lbl_lub2_piston_c = QLabel("⚪ 叶片C:待机")
        for lbl in [self.lbl_lub2_piston_a, self.lbl_lub2_piston_b, self.lbl_lub2_piston_c]:
            lbl.setStyleSheet(piston_lbl_style)
            pistons2_row.addWidget(lbl)
        dist2_vbox.addLayout(pistons2_row)
        box2_layout.addWidget(dist2_frame)

        # 测试与故障控制
        ctrl2_frame = QFrame()
        ctrl2_frame.setStyleSheet("background-color: transparent;")
        ctrl2_vbox = QVBoxLayout(ctrl2_frame)
        ctrl2_vbox.setContentsMargins(0, 4, 0, 0)
        ctrl2_vbox.setSpacing(6)

        self.btn_lub2_test_toggle = QPushButton("▶ 手动启动 2# 泵 (本地测试)")
        self.btn_lub2_test_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lub2_test_toggle.setStyleSheet("background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;")
        self.btn_lub2_test_toggle.clicked.connect(self._on_toggle_lub2_pump)
        ctrl2_vbox.addWidget(self.btn_lub2_test_toggle)

        faults2_row = QHBoxLayout()
        self.chk_lub2_low_level = QCheckBox("低油位故障")
        self.chk_lub2_low_level.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub2_low_level.toggled.connect(self._on_lub_faults_changed)

        self.chk_lub2_block = QCheckBox("分配器堵塞")
        self.chk_lub2_block.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub2_block.toggled.connect(self._on_lub_faults_changed)

        faults2_row.addWidget(self.chk_lub2_low_level)
        faults2_row.addWidget(self.chk_lub2_block)
        ctrl2_vbox.addLayout(faults2_row)
        box2_layout.addWidget(ctrl2_frame)

        circuits_layout.addWidget(box2)
        page_layout.addLayout(circuits_layout)

        # 3. 底部动态曲线示波器 (示波器展示压力和递进活塞时序)
        scope_box = QGroupBox(" 轮毂润滑系统实时运行曲线 ")
        scope_box.setStyleSheet(group_style)
        scope_layout = QVBoxLayout(scope_box)
        scope_layout.setContentsMargins(6, 8, 6, 6)

        lub_plot_widget = pg.GraphicsLayoutWidget()
        lub_plot_widget.setBackground("#121318")
        lub_plot_widget.setFixedHeight(220)

        # 通道 1: 泵供油压力
        lp1 = lub_plot_widget.addPlot(row=0, col=0)
        lp1.showGrid(x=True, y=True, alpha=0.3)
        lp1.setLabel('left', '压力 (bar)', color='#4CC9F0')
        lp1.setYRange(0, 280, padding=0.05)
        lp1.addLegend(offset=(10, 10), brush="#1E202B")
        lp1.addLine(y=185, pen=pg.mkPen(color='#06D6A0', width=1.0, style=Qt.PenStyle.DashLine))
        lp1.addLine(y=250, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DotLine))

        self.curve_lub_p1 = lp1.plot(pen=pg.mkPen(color='#4CC9F0', width=2), name="1# 泵压力")
        self.curve_lub_p2 = lp1.plot(pen=pg.mkPen(color='#FFD166', width=2, style=Qt.PenStyle.DashLine), name="2# 泵压力")

        # 通道 2: 1# 回路递进活塞时序方波
        lp2 = lub_plot_widget.addPlot(row=1, col=0)
        lp2.showGrid(x=True, y=True, alpha=0.3)
        lp2.setLabel('left', '分配脉冲', color='#06D6A0')
        lp2.setLabel('bottom', '相对时间 (s)', color='#A8DADC')
        lp2.setYRange(-0.2, 1.2, padding=0.05)
        lp2.addLegend(offset=(10, 10), brush="#1E202B", colCount=3)

        self.curve_lub_pa = lp2.plot(pen=pg.mkPen(color='#EF233C', width=2), name="叶片 A 活塞")
        self.curve_lub_pb = lp2.plot(pen=pg.mkPen(color='#06D6A0', width=2), name="叶片 B 活塞")
        self.curve_lub_pc = lp2.plot(pen=pg.mkPen(color='#4CC9F0', width=2), name="叶片 C 活塞")

        scope_layout.addWidget(lub_plot_widget)
        page_layout.addWidget(scope_box)

        scroll.setWidget(page)
        return scroll

    def _on_toggle_lub1_pump(self):
        new_state = not self.engine._lub_cmd_pump1_local
        self.engine.set_local_lub_commands(pump1_on=new_state)
        self.btn_lub1_test_toggle.setText("⏹ 停止 1# 泵 (本地测试)" if new_state else "▶ 手动启动 1# 泵 (本地测试)")
        self.btn_lub1_test_toggle.setStyleSheet(
            "background-color: #D90429; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;"
            if new_state else
            "background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;"
        )

    def _on_toggle_lub2_pump(self):
        new_state = not self.engine._lub_cmd_pump2_local
        self.engine.set_local_lub_commands(pump2_on=new_state)
        self.btn_lub2_test_toggle.setText("⏹ 停止 2# 泵 (本地测试)" if new_state else "▶ 手动启动 2# 泵 (本地测试)")
        self.btn_lub2_test_toggle.setStyleSheet(
            "background-color: #D90429; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;"
            if new_state else
            "background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;"
        )

    def _on_lub_faults_changed(self):
        self.engine.set_lub_faults(
            tank1_empty=self.chk_lub1_low_level.isChecked(),
            tank2_empty=self.chk_lub2_low_level.isChecked(),
            block_1=self.chk_lub1_block.isChecked(),
            block_2=self.chk_lub2_block.isChecked()
        )

    def _build_temperature_tab_page(self):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setStyleSheet("background-color: transparent;")

        page = QWidget()
        page.setStyleSheet("background-color: transparent;")
        page_layout = QVBoxLayout(page)
        page_layout.setContentsMargins(10, 8, 10, 8)
        page_layout.setSpacing(10)

        # 1. 顶部状态横幅
        banner = QFrame()
        banner.setObjectName("tempBanner")
        banner.setStyleSheet("""
            #tempBanner {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """)
        banner_layout = QHBoxLayout(banner)
        banner_layout.setContentsMargins(14, 8, 14, 8)
        banner_layout.setSpacing(12)

        title_vbox = QVBoxLayout()
        title_vbox.setSpacing(2)
        b_title = QLabel("6MW 轮毂与液压系统热力学/温度监控仿真")
        b_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #EDF2F4;")
        b_sub = QLabel("液压油温 · 电气柜温 · 轮毂环温 · 变桨轴承温 · 发电机气隙温 · 控制器CPU核心温")
        b_sub.setStyleSheet("font-size: 11px; color: #8D99AE;")
        title_vbox.addWidget(b_title)
        title_vbox.addWidget(b_sub)
        banner_layout.addLayout(title_vbox)

        banner_layout.addStretch()

        self.lbl_temp_banner_oil = QLabel("油温: 42.5°C (正常)")
        self.lbl_temp_banner_oil.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_oil)

        self.lbl_temp_banner_fan = QLabel("风扇: 🟢 运行中")
        self.lbl_temp_banner_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_fan)

        self.lbl_temp_banner_ctrl = QLabel("辅助温控: ⚪ 待机")
        self.lbl_temp_banner_ctrl.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_ctrl)

        page_layout.addWidget(banner)

        # 2. 中部 6 块温度监控卡片 (2x3 网格布局)
        grid = QGridLayout()
        grid.setSpacing(10)

        card_style = """
            QFrame {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """
        bar_style = """
            QProgressBar {
                border: 1px solid #363A4A;
                border-radius: 4px;
                background-color: #16171D;
                text-align: center;
                color: #EDF2F4;
                font-family: Consolas;
                font-size: 11px;
                font-weight: bold;
                height: 18px;
            }
            QProgressBar::chunk {
                background-color: #06D6A0;
                border-radius: 3px;
            }
        """

        def _make_temp_card(title: str, tag: str):
            f = QFrame()
            f.setStyleSheet(card_style)
            ly = QVBoxLayout(f)
            ly.setContentsMargins(12, 10, 12, 10)
            ly.setSpacing(6)

            h = QHBoxLayout()
            t_lbl = QLabel(title)
            t_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #EDF2F4;")
            h.addWidget(t_lbl)
            h.addStretch()
            tag_lbl = QLabel(tag)
            tag_lbl.setStyleSheet("font-size: 10px; color: #8D99AE; font-family: Consolas;")
            h.addWidget(tag_lbl)
            ly.addLayout(h)

            val_row = QHBoxLayout()
            val_lbl = QLabel("--.- °C")
            val_lbl.setStyleSheet("font-family: Consolas; font-size: 18px; font-weight: bold; color: #4CC9F0;")
            val_row.addWidget(val_lbl)
            val_row.addStretch()
            stat_lbl = QLabel("🟢 正常")
            stat_lbl.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
            val_row.addWidget(stat_lbl)
            ly.addLayout(val_row)

            pbar = QProgressBar()
            pbar.setStyleSheet(bar_style)
            pbar.setRange(-30, 100)
            pbar.setValue(25)
            ly.addWidget(pbar)

            sub_lbl = QLabel("阈值: 正常区间")
            sub_lbl.setStyleSheet("font-size: 10px; color: #8D99AE;")
            ly.addWidget(sub_lbl)

            return f, val_lbl, stat_lbl, pbar, sub_lbl

        # 1. 油箱油温
        (c1, self.lbl_temp_oil, self.lbl_stat_oil,
         self.bar_temp_oil, self.lbl_sub_oil) = _make_temp_card("1. 液压站油箱油温", "pac.io.ai.tempOil")
        self.lbl_sub_oil.setText("门限: -20°C 极寒下限 | 58°C 冷却 | 70°C 高温")
        grid.addWidget(c1, 0, 0)

        # 2. 控制电气柜温
        (c2, self.lbl_temp_cab, self.lbl_stat_cab,
         self.bar_temp_cab, self.lbl_sub_cab) = _make_temp_card("2. 变桨控制电气柜温", "pac.io.ai.tempCabinet")
        self.lbl_sub_cab.setText("门限: -10°C 柜内低温 | 60°C 柜内高温 (风扇连锁)")
        grid.addWidget(c2, 0, 1)

        # 3. 轮毂内部环境温
        (c3, self.lbl_temp_hub, self.lbl_stat_hub,
         self.bar_temp_hub, self.lbl_sub_hub) = _make_temp_card("3. 轮毂内部环境温", "pac.io.ai.tempHub")
        self.lbl_sub_hub.setText("门限: -25°C 轮毂低温 | 55°C 轮毂高温")
        grid.addWidget(c3, 0, 2)

        # 4. CPU核心温度
        (c4, self.lbl_temp_cpu, self.lbl_stat_cpu,
         self.bar_temp_cpu, self.lbl_sub_cpu) = _make_temp_card("4. 控制器 CPU 核心温", "pac.io.ai.tempCPU")
        self.lbl_sub_cpu.setText("门限: 80°C CPU 高温告警 (芯片保护)")
        grid.addWidget(c4, 1, 0)

        # 5. 变桨/主轴承温
        (c5, self.lbl_temp_bearing, self.lbl_stat_bearing,
         self.bar_temp_bearing, self.lbl_sub_bearing) = _make_temp_card("5. 变桨/主轴承温度", "pac.io.ai.tempBearing")
        self.lbl_sub_bearing.setText("工况: 双泵注脂润滑散热耦合")
        grid.addWidget(c5, 1, 1)

        # 6. 发电机气隙温
        (c6, self.lbl_temp_airgap, self.lbl_stat_airgap,
         self.bar_temp_airgap, self.lbl_sub_airgap) = _make_temp_card("6. 发电机气隙温度", "pac.io.ai.tempAirgab")
        self.lbl_sub_airgap.setText("工况: 机舱气隙热耗散平衡")
        grid.addWidget(c6, 1, 2)

        page_layout.addLayout(grid)

        # 3. 环境调节与故障注入卡片
        ctrl_box = QGroupBox(" 环境设定与故障模拟 (Local Simulation) ")
        ctrl_box.setStyleSheet("""
            QGroupBox {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
                margin-top: 10px;
                font-size: 12px;
                font-weight: bold;
                color: #EDF2F4;
                padding-top: 14px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 6px;
                background-color: #1E202B;
            }
        """)
        ctrl_layout = QVBoxLayout(ctrl_box)
        ctrl_layout.setContentsMargins(12, 10, 12, 10)
        ctrl_layout.setSpacing(8)

        # 环境温度滑块
        amb_row = QHBoxLayout()
        amb_row.addWidget(QLabel("外部自然环境温度:"))
        self.slider_ambient = QSlider(Qt.Orientation.Horizontal)
        self.slider_ambient.setRange(-40, 50)
        self.slider_ambient.setValue(25)
        self.slider_ambient.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px;
                background: #2C303E;
                border-radius: 3px;
            }
            QSlider::sub-page:horizontal {
                background: #4CC9F0;
                border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #EDF2F4;
                width: 16px;
                margin-top: -5px;
                margin-bottom: -5px;
                border-radius: 8px;
            }
        """)
        self.slider_ambient.valueChanged.connect(self._on_ambient_slider_changed)
        amb_row.addWidget(self.slider_ambient)

        self.lbl_ambient_val = QLabel("25.0 °C")
        self.lbl_ambient_val.setStyleSheet("font-family: Consolas; font-size: 13px; font-weight: bold; color: #4CC9F0; min-width: 60px;")
        amb_row.addWidget(self.lbl_ambient_val)
        ctrl_layout.addLayout(amb_row)

        # 故障复选框
        fault_row = QHBoxLayout()
        fault_row.setSpacing(14)
        self.chk_fault_oil_overheat = QCheckBox("⚠️ 油温过高故障 (>70°C)")
        self.chk_fault_extreme_cold = QCheckBox("❄️ 极寒冷态环境 (-28°C)")
        self.chk_fault_fan_fail = QCheckBox("❌ 柜顶风扇停机")
        self.chk_fault_pt100_open = QCheckBox("⚡ PT100断线开路")

        for chk in [self.chk_fault_oil_overheat, self.chk_fault_extreme_cold,
                    self.chk_fault_fan_fail, self.chk_fault_pt100_open]:
            chk.setStyleSheet("color: #8D99AE; font-size: 11px;")
            chk.toggled.connect(self._on_temp_faults_changed)
            fault_row.addWidget(chk)
        ctrl_layout.addLayout(fault_row)

        page_layout.addWidget(ctrl_box)

        # 4. 底部实时示波器
        scope_box = QGroupBox(" 轮毂与液压系统温度实时运行曲线 ")
        scope_box.setStyleSheet(ctrl_box.styleSheet())
        scope_layout = QVBoxLayout(scope_box)
        scope_layout.setContentsMargins(6, 8, 6, 6)

        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground("#121318")
        plot_widget.setFixedHeight(220)

        tp = plot_widget.addPlot(row=0, col=0)
        tp.showGrid(x=True, y=True, alpha=0.3)
        tp.setLabel('left', '温度 (°C)', color='#FFD166')
        tp.setLabel('bottom', '相对时间 (s)', color='#A8DADC')
        tp.setYRange(-35, 90, padding=0.05)
        tp.addLegend(offset=(10, 10), brush="#1E202B", colCount=5)

        tp.addLine(y=70, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DashLine))
        tp.addLine(y=60, pen=pg.mkPen(color='#F72585', width=1.0, style=Qt.PenStyle.DotLine))
        tp.addLine(y=-20, pen=pg.mkPen(color='#4CC9F0', width=1.0, style=Qt.PenStyle.DashDotLine))

        self.curve_temp_oil = tp.plot(pen=pg.mkPen(color='#FFD166', width=2), name="油箱油温")
        self.curve_temp_cab = tp.plot(pen=pg.mkPen(color='#4CC9F0', width=2), name="电气柜温")
        self.curve_temp_hub = tp.plot(pen=pg.mkPen(color='#06D6A0', width=2), name="轮毂环温")
        self.curve_temp_bearing = tp.plot(pen=pg.mkPen(color='#B5179E', width=2), name="变桨轴承")
        self.curve_temp_cpu = tp.plot(pen=pg.mkPen(color='#EF233C', width=2, style=Qt.PenStyle.DashLine), name="CPU核心")

        scope_layout.addWidget(plot_widget)
        page_layout.addWidget(scope_box)

        scroll.setWidget(page)
        return scroll

    def _on_ambient_slider_changed(self, val):
        t = float(val)
        self.lbl_ambient_val.setText(f"{t:+4.1f} °C")
        self.engine.set_ambient_temperature(t)

    def _on_temp_faults_changed(self):
        self.engine.set_temp_faults(
            oil_overheat=self.chk_fault_oil_overheat.isChecked(),
            extreme_cold=self.chk_fault_extreme_cold.isChecked(),
            fan_fail=self.chk_fault_fan_fail.isChecked(),
            pt100_open=self.chk_fault_pt100_open.isChecked()
        )

    # =========================================================================
    # 状态与样式控制
    # =========================================================================
    def _select_face(self, key):
        self.active_blade = key
        idx = self.blades.index(key)
        if self.tab_widget.currentIndex() != idx:
            self.tab_widget.setCurrentIndex(idx)
        self._update_face_buttons_style()
        self._sync_left_controls_to_active_blade()

    def _on_tab_changed(self, idx):
        if 0 <= idx < len(self.blades):
            key = self.blades[idx]
            self.active_blade = key
            self._update_face_buttons_style()
            self._sync_left_controls_to_active_blade()
        else:
            for b in self.btn_faces.values():
                b.setChecked(False)
                b.setStyleSheet("background-color: #21232C; color: #8D99AE; border: 1px solid #363A4A; border-radius: 4px;")

    def _on_sync_changed(self, state):
        self.is_sync_all = (state == Qt.CheckState.Checked.value or state == 2)
        if self.is_sync_all:
            cur_t = self.target_pos_mm[self.active_blade]
            cur_v = self.manual_voltage[self.active_blade]
            for k in self.blades:
                self.target_pos_mm[k] = cur_t
                self.manual_voltage[k] = cur_v

    def _update_face_buttons_style(self):
        for k, b in self.btn_faces.items():
            is_active = (k == self.active_blade)
            b.setChecked(is_active)
            if is_active:
                b.setStyleSheet("background-color: #05668D; color: #FFFFFF; font-weight: bold; border: 2px solid #4CC9F0; border-radius: 5px; font-size: 11px;")
            else:
                b.setStyleSheet("background-color: #262936; color: #8D99AE; border: 1px solid #424658; border-radius: 5px; font-size: 11px;")

    def _sync_left_controls_to_active_blade(self):
        cur_t = self.target_pos_mm[self.active_blade]
        if self.slider_target.value() != int(cur_t):
            self.slider_target.setValue(int(cur_t))
        self.spin_target.setValue(cur_t)

        cur_v = self.manual_voltage[self.active_blade]
        int_v = int(cur_v * 10)
        if self.slider_voltage.value() != int_v:
            self.slider_voltage.setValue(int_v)
        self.spin_voltage.setValue(cur_v)

        self._update_feather_button_style()
        self._update_lock_buttons_style()

    def _set_mode(self, is_auto):
        self.is_auto_mode = is_auto
        self.btn_mode_auto.setChecked(is_auto)
        self.btn_mode_manual.setChecked(not is_auto)

        self.widget_auto_ctrl.setVisible(is_auto)
        self.widget_manual_ctrl.setVisible(not is_auto)

        self._update_mode_styles()

        if not is_auto:
            target_keys = self.blades if self.is_sync_all else [self.active_blade]
            for k in target_keys:
                self.engine.set_inputs(blade=k, u_prop=self.manual_voltage[k])

    def _update_mode_styles(self):
        if self.is_auto_mode:
            self.btn_mode_auto.setStyleSheet("""
                QPushButton {
                    background-color: #05668D;
                    color: #FFFFFF;
                    font-weight: bold;
                    border: 2px solid #4CC9F0;
                    border-radius: 6px;
                    font-size: 12px;
                }
            """)
            self.btn_mode_manual.setStyleSheet("""
                QPushButton {
                    background-color: #262936;
                    color: #8D99AE;
                    border: 1px solid #424658;
                    border-radius: 6px;
                    font-size: 12px;
                }
                QPushButton:hover {
                    background-color: #35394A;
                    color: #EDF2F4;
                }
            """)
        else:
            self.btn_mode_auto.setStyleSheet("""
                QPushButton {
                    background-color: #262936;
                    color: #8D99AE;
                    border: 1px solid #424658;
                    border-radius: 6px;
                    font-size: 12px;
                }
                QPushButton:hover {
                    background-color: #35394A;
                    color: #EDF2F4;
                }
            """)
            self.btn_mode_manual.setStyleSheet("""
                QPushButton {
                    background-color: #7B2CBF;
                    color: #FFFFFF;
                    font-weight: bold;
                    border: 2px solid #C77DFF;
                    border-radius: 6px;
                    font-size: 12px;
                }
            """)

    def _update_feather_button_style(self):
        is_feath = self.is_feathering[self.active_blade]
        if is_feath:
            self.btn_feather.setText("✔ 恢复正常闭环运行")
            self.btn_feather.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #06D6A0, stop:1 #058C6B);
                    color: #FFFFFF;
                    font-size: 13px;
                    font-weight: bold;
                    border: 2px solid #52FFD1;
                    border-radius: 6px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #12E6AE, stop:1 #07A37C);
                }
            """)
            self.lbl_safe_status.setText(f"状态: ⚠️ 【{self.active_blade}面】应急顺桨中")
            self.lbl_safe_status.setStyleSheet("color: #EF233C; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        else:
            self.btn_feather.setText("🛑 模拟紧急顺桨")
            self.btn_feather.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #D90429, stop:1 #9D0208);
                    color: #FFFFFF;
                    font-size: 13px;
                    font-weight: bold;
                    border: 2px solid #FF4D6D;
                    border-radius: 6px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #EF233C, stop:1 #B30006);
                }
            """)
            self.lbl_safe_status.setText("状态: 正常运行")
            self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

    def _update_lock_buttons_style(self):
        u130 = self.u_130_cmd[self.active_blade]
        u132 = self.u_132_cmd[self.active_blade]
        unlocked = (u130 > 0.5 and u132 > 0.5)
        if unlocked:
            self.btn_lock_release.setStyleSheet("""
                QPushButton {
                    background-color: #05668D;
                    color: #FFFFFF;
                    font-weight: bold;
                    border: 2px solid #4CC9F0;
                    border-radius: 6px;
                }
            """)
            self.btn_lock_engage.setStyleSheet("""
                QPushButton {
                    background-color: #2C303E;
                    color: #8D99AE;
                    border: 1px solid #44495C;
                    border-radius: 6px;
                }
                QPushButton:hover {
                    background-color: #3D4357;
                    color: #FFFFFF;
                }
            """)
        else:
            self.btn_lock_release.setStyleSheet("""
                QPushButton {
                    background-color: #2C303E;
                    color: #8D99AE;
                    border: 1px solid #44495C;
                    border-radius: 6px;
                }
                QPushButton:hover {
                    background-color: #3D4357;
                    color: #FFFFFF;
                }
            """)
            self.btn_lock_engage.setStyleSheet("""
                QPushButton {
                    background-color: #B5179E;
                    color: #FFFFFF;
                    font-weight: bold;
                    border: 2px solid #F72585;
                    border-radius: 6px;
                }
            """)

    def _cmd_unlock(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.u_130_cmd[k] = 1.0
            self.u_132_cmd[k] = 1.0
            self.engine.set_inputs(blade=k, u_valve_130=1.0, u_valve_132=1.0)
        self._update_lock_buttons_style()

    def _cmd_lock(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        for k in target_keys:
            self.u_130_cmd[k] = 0.0
            self.u_132_cmd[k] = 0.0
            self.engine.set_inputs(blade=k, u_valve_130=0.0, u_valve_132=0.0)
        self._update_lock_buttons_style()

    def _on_target_changed(self, val):
        v = float(val)
        if self.is_sync_all:
            for k in self.blades:
                self.target_pos_mm[k] = v
        else:
            self.target_pos_mm[self.active_blade] = v
        if self.slider_target.value() != int(v):
            self.slider_target.setValue(int(v))

    def _on_voltage_changed(self, val):
        v = float(val)
        if self.is_sync_all:
            for k in self.blades:
                self.manual_voltage[k] = v
                if not self.is_auto_mode and not self.is_feathering[k]:
                    self.engine.set_inputs(blade=k, u_prop=v)
        else:
            self.manual_voltage[self.active_blade] = v
            if not self.is_auto_mode and not self.is_feathering[self.active_blade]:
                self.engine.set_inputs(blade=self.active_blade, u_prop=v)
        int_v = int(v * 10)
        if self.slider_voltage.value() != int_v:
            self.slider_voltage.setValue(int_v)

    def _on_toggle_feather(self):
        target_keys = self.blades if self.is_sync_all else [self.active_blade]
        new_feather_state = not self.is_feathering[self.active_blade]
        for k in target_keys:
            self.is_feathering[k] = new_feather_state
            if new_feather_state:
                self.engine.set_inputs(blade=k, u_safe_feather=1.0, u_prop=0.0, u_enable=0.0)
            else:
                self.engine.set_inputs(blade=k, u_safe_feather=0.0, u_enable=1.0)
        self._update_feather_button_style()

    def _open_conn_settings(self):
        """弹出 OPC UA 通讯参数与连接诊断设置对话框"""
        dlg = ConnectionSettingsDialog(parent=self, current_config=self.conn_config, on_apply=self._apply_conn_config)
        dlg.exec()

    def _apply_conn_config(self, new_config: dict):
        """应用并热重载 OPC UA 连接参数"""
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

            # 重新实例化带新配置的 Bridge
            self.opcua_bridge = PitchOpcUaBridge(
                endpoint=self.opcua_endpoint,
                engine=self.engine,
                cycle_time_s=self.opcua_cycle_time_s
            )

            if was_running and self.control_mode == 'REMOTE':
                self.opcua_bridge.start()

    def _set_control_authority(self, mode: str):
        is_local = (mode == "LOCAL")
        self.control_mode = mode
        self.engine.set_control_mode(mode)
        self.btn_local_ctrl.setChecked(is_local)
        self.btn_remote_ctrl.setChecked(not is_local)

        if is_local:
            # 本地控制：停止 OPC UA 远程覆盖
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
                self.opcua_bridge.stop()

            self.btn_local_ctrl.setStyleSheet("background-color: #06D6A0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.btn_remote_ctrl.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.lbl_auth_status.setText("● 本地就绪")
            self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px;")

            self.motion_box.setEnabled(True)
            self.lock_box.setEnabled(True)
            self.safe_box.setEnabled(True)
        else:
            # 远程控制：自动开启 OPC UA 通讯
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
                self.opcua_bridge.start()

            self.btn_local_ctrl.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.btn_remote_ctrl.setStyleSheet("background-color: #4CC9F0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px; padding: 0 12px;")
            self.lbl_auth_status.setText("🔄 连接中 (4840)...")
            self.lbl_auth_status.setStyleSheet("color: #F77F00; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #2B2317; border-radius: 4px;")

            self.motion_box.setEnabled(False)
            self.lock_box.setEnabled(False)
            self.safe_box.setEnabled(False)

    # =========================================================================
    # 50 FPS 循环更新与机械互锁监测 (同时推进 A/B/C 三面)
    # =========================================================================
    def _on_ui_tick(self):
        telems = self.engine.get_all_telemetry()

        # 1. 对 A、B、C 三个面分别执行自动闭环控制 (PID) 或手动开环控制
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
                        self.engine.set_inputs(blade=k, u_prop=u_calc)
                    u_outputs[k] = u_calc
                else:
                    u_calc = self.manual_voltage[k] if (not self.is_feathering[k] and not t_k.is_locked) else 0.0
                    if not self.is_auto_mode:
                        self.engine.set_inputs(blade=k, u_prop=u_calc)
                    u_outputs[k] = u_calc
            else:
                # 远程模式: 比例阀实际动作由远程指令驱动，本地算法不覆盖输入
                u_outputs[k] = t_k.valve_position_pct / 10.0

            # 更新该面的拓扑示意图
            self.schematic_widgets[k].update_telemetry(
                stroke_mm=x_k,
                p_A=t_k.p_A_bar,
                p_B=t_k.p_B_bar,
                p_acc=t_k.p_acc_bar,
                valve_pos_pct=t_k.valve_position_pct,
                is_feathering=self.is_feathering[k],
                lock_pin_mm=t_k.lock_pin_mm,
                is_locked=t_k.is_locked,
                switch_129=t_k.switch_129,
                switch_S_unlocked=t_k.switch_S_unlocked,
                p_PP_bar=t_k.p_PP_bar,
                valve_25_state=t_k.valve_25_state,
                p_supply_bar=t_k.p_supply_bar,
                q_hpu_lpm=t_k.q_hpu_lpm
            )

            # 更新该面的示波器历史缓存与曲线
            self.target_hist[k] = np.roll(self.target_hist[k], -1)
            self.target_hist[k][-1] = self.target_pos_mm[k]

            self.actual_hist[k] = np.roll(self.actual_hist[k], -1)
            self.actual_hist[k][-1] = x_k

            self.pA_hist[k] = np.roll(self.pA_hist[k], -1)
            self.pA_hist[k][-1] = t_k.p_A_bar

            self.pB_hist[k] = np.roll(self.pB_hist[k], -1)
            self.pB_hist[k][-1] = t_k.p_B_bar

            self.pacc_hist[k] = np.roll(self.pacc_hist[k], -1)
            self.pacc_hist[k][-1] = t_k.p_acc_bar

            self.pSupply_hist[k] = np.roll(self.pSupply_hist[k], -1)
            self.pSupply_hist[k][-1] = t_k.p_supply_bar

            self.curves_target[k].setData(self.time_hist, self.target_hist[k])
            self.curves_actual[k].setData(self.time_hist, self.actual_hist[k])
            self.curves_pA[k].setData(self.time_hist, self.pA_hist[k])
            self.curves_pB[k].setData(self.time_hist, self.pB_hist[k])
            self.curves_pacc[k].setData(self.time_hist, self.pacc_hist[k])
            self.curves_pSupply[k].setData(self.time_hist, self.pSupply_hist[k])

        # 2. 状态条与活动面数据更新
        active_t = telems[self.active_blade]
        self.lbl_clock_stat.setText(
            f"时钟: {active_t.timestamp:5.1f}s | 周期: {active_t.cycle_time_ms:4.1f}ms | 抖动: {active_t.jitter_ms:4.1f}ms"
        )

        # 3. 变桨锁交互状态展示 (针对当前活动面)
        align_str = "90°对齐: 是" if active_t.switch_129 else "90°对齐: 否"
        if active_t.is_locked:
            self.lbl_lock_detail.setText(f"状态: 🔒 已锁定 ({active_t.lock_pin_mm:4.1f}mm) | {align_str}")
            self.lbl_lock_detail.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #2B2317; border: 1px solid #5C4A22; border-radius: 4px;")
        else:
            lock_str = "🟢 已解锁" if active_t.switch_S_unlocked else f"⏳ 动作中 ({active_t.lock_pin_mm:4.1f}mm)"
            self.lbl_lock_detail.setText(f"状态: {lock_str} | {align_str}")
            self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

        # 4. 遥测读数 (当前活动面)
        self.tele_labels["disp"].setText(f"{active_t.displacement_mm:5.1f} mm")
        self.tele_labels["vel"].setText(f"{active_t.velocity_mps*1000.0:+5.1f} mm/s")
        self.tele_labels["u_v"].setText(f"{u_outputs[self.active_blade]:+5.2f} V")
        self.tele_labels["spool"].setText(f"{active_t.valve_position_pct:+5.1f} %")
        self.tele_labels["pA"].setText(f"{active_t.p_A_bar:5.1f} bar")
        self.tele_labels["pB"].setText(f"{active_t.p_B_bar:5.1f} bar")
        self.tele_labels["pacc"].setText(f"{active_t.p_acc_bar:5.1f} bar")
        self.tele_labels["pPP"].setText(f"{active_t.p_PP_bar:5.1f} bar")
        self.tele_labels["v25"].setText("打压" if active_t.valve_25_state else "卸荷")
        self.tele_labels["v25"].setStyleSheet("color: #06D6A0; font-family: Consolas; font-size: 12px; font-weight: bold;" if active_t.valve_25_state else "color: #8D99AE; font-family: Consolas; font-size: 12px; font-weight: bold;")
        self.tele_labels["pP"].setText(f"{active_t.p_supply_bar:5.1f} bar")

        # 5. 远程控制模式下实时更新 OPC UA 通讯状态与双向心跳
        if self.control_mode == 'REMOTE':
            if hasattr(self, 'opcua_bridge') and self.opcua_bridge and self.opcua_bridge.is_connected:
                hb_plc = self.opcua_bridge.heartbeat_plc
                hb_sim = self.opcua_bridge.heartbeat_sim
                self.lbl_auth_status.setText(f"🟢 远程在线 (PLC: {hb_plc} | Sim: {hb_sim})")
                self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
            else:
                self.lbl_auth_status.setText("🔄 连接中 (4840)...")
                self.lbl_auth_status.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold; padding: 2px 8px; background-color: #2B2317; border-radius: 4px; border: 1px solid #5A4015;")

        # 6. 6MW 轮毂润滑系统遥测更新
        lub = self.engine.get_lub_telemetry()

        # 1# 泵回路更新
        if lub.pump1_on:
            self.lbl_lub1_pump_status.setText("🟢 运行中")
            self.lbl_lub1_pump_status.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_lub1_pump_status.setText("⚪ 停机")
            self.lbl_lub1_pump_status.setStyleSheet("color: #8D99AE; font-weight: bold; background-color: #21232C; padding: 2px 8px; border-radius: 4px; border: 1px solid #363A4A;")

        self.lbl_lub1_pressure.setText(f"{lub.pump1_pressure_bar:5.1f} bar")
        self.bar_lub1_pressure.setValue(int(min(250, max(0, lub.pump1_pressure_bar))))

        self.lbl_lub1_tank_level.setText(f"{lub.tank1_level_pct:5.1f} %")
        self.bar_lub1_tank_level.setValue(int(min(100, max(0, lub.tank1_level_pct))))

        if lub.tank1_level_ok:
            self.lbl_lub1_tank_ok.setText("🟢 正常")
            self.lbl_lub1_tank_ok.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_lub1_tank_ok.setText("🔴 低油位报警")
            self.lbl_lub1_tank_ok.setStyleSheet("color: #EF233C; font-weight: bold; background-color: #381519; padding: 2px 8px; border-radius: 4px; border: 1px solid #5C2229;")

        def _update_piston_lbl(lbl, active, name):
            if active:
                lbl.setText(f"🟢 {name}:动作")
                lbl.setStyleSheet("color: #52FFD1; font-size: 11px; font-weight: bold; background-color: #1B3D32; border: 1px solid #06D6A0; border-radius: 4px; padding: 4px; qproperty-alignment: AlignCenter;")
            else:
                lbl.setText(f"⚪ {name}:待机")
                lbl.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px; padding: 4px; qproperty-alignment: AlignCenter;")

        _update_piston_lbl(self.lbl_lub1_piston_a, lub.piston1_a, "叶片A")
        _update_piston_lbl(self.lbl_lub1_piston_b, lub.piston1_b, "叶片B")
        _update_piston_lbl(self.lbl_lub1_piston_c, lub.piston1_c, "叶片C")
        self.lbl_lub1_cycles.setText(f"累计: {lub.cycle_count_1} 循环")

        # 2# 泵回路更新
        if lub.pump2_on:
            self.lbl_lub2_pump_status.setText("🟢 运行中")
            self.lbl_lub2_pump_status.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_lub2_pump_status.setText("⚪ 停机")
            self.lbl_lub2_pump_status.setStyleSheet("color: #8D99AE; font-weight: bold; background-color: #21232C; padding: 2px 8px; border-radius: 4px; border: 1px solid #363A4A;")

        self.lbl_lub2_pressure.setText(f"{lub.pump2_pressure_bar:5.1f} bar")
        self.bar_lub2_pressure.setValue(int(min(250, max(0, lub.pump2_pressure_bar))))

        self.lbl_lub2_tank_level.setText(f"{lub.tank2_level_pct:5.1f} %")
        self.bar_lub2_tank_level.setValue(int(min(100, max(0, lub.tank2_level_pct))))

        if lub.tank2_level_ok:
            self.lbl_lub2_tank_ok.setText("🟢 正常")
            self.lbl_lub2_tank_ok.setStyleSheet("color: #06D6A0; font-weight: bold; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_lub2_tank_ok.setText("🔴 低油位报警")
            self.lbl_lub2_tank_ok.setStyleSheet("color: #EF233C; font-weight: bold; background-color: #381519; padding: 2px 8px; border-radius: 4px; border: 1px solid #5C2229;")

        _update_piston_lbl(self.lbl_lub2_piston_a, lub.piston2_a, "叶片A")
        _update_piston_lbl(self.lbl_lub2_piston_b, lub.piston2_b, "叶片B")
        _update_piston_lbl(self.lbl_lub2_piston_c, lub.piston2_c, "叶片C")
        self.lbl_lub2_cycles.setText(f"累计: {lub.cycle_count_2} 循环")

        # 顶部横幅药丸状态
        if lub.fault_tank1_empty or lub.fault_block_1:
            self.lub_banner_pill1.setText("1# 主回路: ⚠️ 故障告警")
            self.lub_banner_pill1.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        elif lub.pump1_on:
            self.lub_banner_pill1.setText("1# 主回路: 🟢 运行注脂中")
            self.lub_banner_pill1.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        else:
            self.lub_banner_pill1.setText("1# 主回路: ⚪ 待机")
            self.lub_banner_pill1.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px;")

        if lub.fault_tank2_empty or lub.fault_block_2:
            self.lub_banner_pill2.setText("2# 备用回路: ⚠️ 故障告警")
            self.lub_banner_pill2.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        elif lub.pump2_on:
            self.lub_banner_pill2.setText("2# 备用回路: 🟢 运行注脂中")
            self.lub_banner_pill2.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        else:
            self.lub_banner_pill2.setText("2# 备用回路: ⚪ 待机")
            self.lub_banner_pill2.setStyleSheet("color: #8D99AE; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #21232C; border: 1px solid #363A4A; border-radius: 4px;")

        # 更新示波器数据
        self.lub_p1_hist = np.roll(self.lub_p1_hist, -1)
        self.lub_p1_hist[-1] = lub.pump1_pressure_bar
        self.lub_p2_hist = np.roll(self.lub_p2_hist, -1)
        self.lub_p2_hist[-1] = lub.pump2_pressure_bar

        self.lub_piston1_a_hist = np.roll(self.lub_piston1_a_hist, -1)
        self.lub_piston1_a_hist[-1] = 1.0 if lub.piston1_a else 0.0
        self.lub_piston1_b_hist = np.roll(self.lub_piston1_b_hist, -1)
        self.lub_piston1_b_hist[-1] = 1.0 if lub.piston1_b else 0.0
        self.lub_piston1_c_hist = np.roll(self.lub_piston1_c_hist, -1)
        self.lub_piston1_c_hist[-1] = 1.0 if lub.piston1_c else 0.0

        self.curve_lub_p1.setData(self.time_hist, self.lub_p1_hist)
        self.curve_lub_p2.setData(self.time_hist, self.lub_p2_hist)
        self.curve_lub_pa.setData(self.time_hist, self.lub_piston1_a_hist)
        self.curve_lub_pb.setData(self.time_hist, self.lub_piston1_b_hist)
        self.curve_lub_pc.setData(self.time_hist, self.lub_piston1_c_hist)

        # 7. 6MW 轮毂温度热力学系统遥测更新
        tmp = self.engine.get_temp_telemetry()

        # 更新卡片数值与样式
        def _render_val(lbl, stat_lbl, pbar, val, low_lim, high_lim):
            if val <= -900.0:
                lbl.setText("断线故障")
                lbl.setStyleSheet("font-family: Consolas; font-size: 16px; font-weight: bold; color: #EF233C;")
                stat_lbl.setText("⚡ 断线")
                stat_lbl.setStyleSheet("color: #EF233C; font-weight: bold; font-size: 11px; background-color: #381519; padding: 2px 8px; border-radius: 4px; border: 1px solid #5C2229;")
                pbar.setValue(0)
            else:
                lbl.setText(f"{val:4.1f} °C")
                pbar.setValue(int(val))
                if val >= high_lim:
                    lbl.setStyleSheet("font-family: Consolas; font-size: 18px; font-weight: bold; color: #EF233C;")
                    stat_lbl.setText("🔴 高温报警")
                    stat_lbl.setStyleSheet("color: #EF233C; font-weight: bold; font-size: 11px; background-color: #381519; padding: 2px 8px; border-radius: 4px; border: 1px solid #5C2229;")
                elif val <= low_lim:
                    lbl.setStyleSheet("font-family: Consolas; font-size: 18px; font-weight: bold; color: #4CC9F0;")
                    stat_lbl.setText("❄️ 极寒报警")
                    stat_lbl.setStyleSheet("color: #4CC9F0; font-weight: bold; font-size: 11px; background-color: #172B38; padding: 2px 8px; border-radius: 4px; border: 1px solid #22455C;")
                elif val >= high_lim - 10.0:
                    lbl.setStyleSheet("font-family: Consolas; font-size: 18px; font-weight: bold; color: #FFB703;")
                    stat_lbl.setText("⚠️ 偏高预警")
                    stat_lbl.setStyleSheet("color: #FFB703; font-weight: bold; font-size: 11px; background-color: #2B2317; padding: 2px 8px; border-radius: 4px; border: 1px solid #5C4A22;")
                else:
                    lbl.setStyleSheet("font-family: Consolas; font-size: 18px; font-weight: bold; color: #06D6A0;")
                    stat_lbl.setText("🟢 正常")
                    stat_lbl.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; background-color: #172B23; padding: 2px 8px; border-radius: 4px; border: 1px solid #23533E;")

        _render_val(self.lbl_temp_oil, self.lbl_stat_oil, self.bar_temp_oil, tmp.temp_oil_deg, -20.0, 70.0)
        _render_val(self.lbl_temp_cab, self.lbl_stat_cab, self.bar_temp_cab, tmp.temp_cabinet_deg, -10.0, 60.0)
        _render_val(self.lbl_temp_hub, self.lbl_stat_hub, self.bar_temp_hub, tmp.temp_hub_deg, -25.0, 55.0)
        _render_val(self.lbl_temp_cpu, self.lbl_stat_cpu, self.bar_temp_cpu, tmp.temp_cpu_deg, -10.0, 80.0)
        _render_val(self.lbl_temp_bearing, self.lbl_stat_bearing, self.bar_temp_bearing, tmp.temp_bearing_deg, -20.0, 65.0)
        _render_val(self.lbl_temp_airgap, self.lbl_stat_airgap, self.bar_temp_airgap, tmp.temp_airgap_deg, -20.0, 65.0)

        # 顶部横幅药丸状态
        if tmp.alarm_oil_high:
            self.lbl_temp_banner_oil.setText("油温: 🔴 超上限报警")
            self.lbl_temp_banner_oil.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        elif tmp.alarm_oil_low:
            self.lbl_temp_banner_oil.setText("油温: ❄️ 超下限报警")
            self.lbl_temp_banner_oil.setStyleSheet("color: #4CC9F0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B38; border: 1px solid #22455C; border-radius: 4px;")
        else:
            self.lbl_temp_banner_oil.setText(f"油温: 🟢 正常 ({tmp.temp_oil_deg:4.1f}°C)")
            self.lbl_temp_banner_oil.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

        if tmp.cabinet_fan:
            self.lbl_temp_banner_fan.setText("风扇: 🟢 运行中")
            self.lbl_temp_banner_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        else:
            self.lbl_temp_banner_fan.setText("风扇: 🔴 停机")
            self.lbl_temp_banner_fan.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")

        heat_str = "加热: ON" if tmp.oil_heater else "加热: OFF"
        cool_str = "冷却: ON" if tmp.oil_cooler else "冷却: OFF"
        self.lbl_temp_banner_ctrl.setText(f"{heat_str} | {cool_str}")

        # 示波器曲线更新
        safe_oil = tmp.temp_oil_deg if tmp.temp_oil_deg > -900.0 else 0.0
        safe_cab = tmp.temp_cabinet_deg if tmp.temp_cabinet_deg > -900.0 else 0.0
        safe_hub = tmp.temp_hub_deg if tmp.temp_hub_deg > -900.0 else 0.0
        safe_bearing = tmp.temp_bearing_deg if tmp.temp_bearing_deg > -900.0 else 0.0

        self.temp_oil_hist = np.roll(self.temp_oil_hist, -1)
        self.temp_oil_hist[-1] = safe_oil
        self.temp_cab_hist = np.roll(self.temp_cab_hist, -1)
        self.temp_cab_hist[-1] = safe_cab
        self.temp_hub_hist = np.roll(self.temp_hub_hist, -1)
        self.temp_hub_hist[-1] = safe_hub
        self.temp_bearing_hist = np.roll(self.temp_bearing_hist, -1)
        self.temp_bearing_hist[-1] = safe_bearing
        self.temp_cpu_hist = np.roll(self.temp_cpu_hist, -1)
        self.temp_cpu_hist[-1] = tmp.temp_cpu_deg

        self.curve_temp_oil.setData(self.time_hist, self.temp_oil_hist)
        self.curve_temp_cab.setData(self.time_hist, self.temp_cab_hist)
        self.curve_temp_hub.setData(self.time_hist, self.temp_hub_hist)
        self.curve_temp_bearing.setData(self.time_hist, self.temp_bearing_hist)
        self.curve_temp_cpu.setData(self.time_hist, self.temp_cpu_hist)

    def closeEvent(self, event):
        self.timer.stop()
        if hasattr(self, 'opcua_bridge') and self.opcua_bridge:
            self.opcua_bridge.stop()
        self.engine.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = PitchMainWindow()
    if "--remote" in sys.argv:
        win._set_control_authority("REMOTE")
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
