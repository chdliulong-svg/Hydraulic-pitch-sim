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
    QTabWidget, QCheckBox
)
from PyQt6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QLinearGradient
)
from PyQt6.QtCore import Qt, QTimer

import pyqtgraph as pg
from pitch_soft_realtime import SoftRealtimePitchEngine
from pitch_opcua_bridge import PitchOpcUaBridge


# =============================================================================
# 1. 自定义高精度动态机械/液压拓扑渲染画布 (含 40/20X60 变桨机械锁)
# =============================================================================
class HydraulicSchematicWidget(QWidget):
    def __init__(self, face_name="A", parent=None):
        super().__init__(parent)
        self.face_name = face_name
        self.setMinimumHeight(320)
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
        tag = f"【{self.face_name}面机构】"
        if self.is_locked:
            painter.setPen(QColor("#FFB703"))
            painter.drawText(
                22, 23,
                f"{tag} 🔒 变桨锁已刚性锁死：40/20X60销轴卡入轮毂锁孔，主油缸机械闭锁禁止动作！"
            )
        elif self.is_feathering:
            painter.setPen(QColor("#EF233C"))
            painter.drawText(
                22, 23,
                f"{tag} ★ 应急顺桨保护触发：阀120/103导通，蓄能器高压直充A腔，B腔经快放孔高速泄压 ★"
            )
        else:
            painter.setPen(QColor("#06D6A0"))
            painter.drawText(
                22, 23,
                f"{tag} 🟢 正常变桨运行：机械锁已拔出解锁，比例伺服阀 102A 连续微调，双缸并联驱动"
            )

        # 2. 坐标布局 (start_x 向右留足 118px 边距，c1_y 下移至 90px，避免文字与缸体重叠或被顶部切边)
        avail_w = w - 190
        start_x = 118
        cyl_w = max(250, min(320, avail_w - 250))
        cyl_h = 40
        c1_y = 90
        c2_y = 155

        ratio = max(0.0, min(1.0, self.stroke_mm / self.stroke_max))
        piston_x = start_x + 12 + ratio * (cyl_w - 55)

        col_A = self._p_to_color(self.p_A_bar)
        col_B = self._p_to_color(self.p_B_bar)

        # 绘制主缸 124 与缸 125
        for cy, name in [(c1_y, f"{self.face_name}面缸124 (200/125)"), (c2_y, f"{self.face_name}面缸125 (200/125)")]:
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
            rod_len = (cyl_w - 45)
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

            # 标签 (置于缸体左侧空白区 start_x-110 处，杜绝与缸体重叠遮挡)
            painter.setPen(QColor("#A8DADC"))
            painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
            painter.drawText(start_x - 110, cy + 25, name)

        # 双杆机械同步连杆 (带有机械锁孔)
        link_x = piston_x + 16 + (cyl_w - 45)
        painter.setPen(QPen(QColor("#FFD166"), 6))
        painter.drawLine(int(link_x), int(c1_y + 20), int(link_x), int(c2_y + 20))

        # 铰支轴承
        painter.setPen(QPen(QColor("#EDF2F4"), 2))
        painter.setBrush(QBrush(QColor("#06D6A0")))
        mid_cy = int((c1_y + c2_y) / 2 + 20)
        painter.drawEllipse(int(link_x - 7), mid_cy - 7, 14, 14)
        painter.setPen(QColor("#06D6A0"))
        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        painter.drawText(int(link_x + 16), mid_cy + 5, "▶ 驱动叶片")

        # -------------------------------------------------------------
        # 3. 动态绘制 40/20X60 变桨机械锁紧装置与锁销动作
        # -------------------------------------------------------------
        lock_stand_x = start_x + 12 + (cyl_w - 55) + 16 + (cyl_w - 45) # 90°顺桨时连杆所在固定位置
        lock_box_x = int(lock_stand_x - 14)
        lock_box_y = c1_y - 44
        lock_box_w = 28
        lock_box_h = 36

        # 销轴伸缩 (0~60mm 映射为 0~24px)
        pin_travel_px = int((self.lock_pin_mm / 60.0) * 24)
        pin_tip_y = lock_box_y + lock_box_h + pin_travel_px

        # 锁油缸外壳 (40/20X60)
        painter.setPen(QPen(QColor("#FFD166"), 1.8))
        painter.setBrush(QBrush(QColor("#242734")))
        painter.drawRoundedRect(lock_box_x, lock_box_y, lock_box_w, lock_box_h, 3, 3)

        # 内置弹簧示意 (绿色受压, 橙色释能)
        painter.setPen(QPen(QColor("#06D6A0" if self.lock_pin_mm < 10 else "#FFB703"), 1.5))
        for sp_y in range(lock_box_y + 6, lock_box_y + 24, 4):
            painter.drawLine(lock_box_x + 4, sp_y, lock_box_x + lock_box_w - 4, sp_y + 2)

        # 高强度钢锁紧销 (向下伸出插孔)
        painter.setPen(QPen(QColor("#EDF2F4"), 1))
        painter.setBrush(QBrush(QColor("#E0E1DD" if not self.is_locked else "#EF233C")))
        painter.drawRect(lock_box_x + 8, lock_box_y + 12, 12, lock_box_h - 12 + pin_travel_px)

        # 锁孔套筒 (位于机械架上)
        painter.setPen(QPen(QColor("#8D99AE"), 2))
        painter.drawRect(lock_box_x + 5, c1_y - 2, 18, 14)

        # 锁状态标志 (纵向坐标 lock_box_y - 6 = 40px，上方留足空间，彻底解决文字切边与遮挡问题)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        if self.is_locked:
            painter.setPen(QColor("#EF233C"))
            painter.drawText(lock_box_x - 30, lock_box_y - 6, f"🔒 {self.face_name}面已锁死")
        elif self.switch_129:
            painter.setPen(QColor("#06D6A0"))
            painter.drawText(lock_box_x - 35, lock_box_y - 6, f"🟢 {self.face_name}面对准可锁")
        else:
            painter.setPen(QColor("#A8DADC"))
            painter.drawText(lock_box_x - 30, lock_box_y - 6, f"🔓 {self.face_name}面已解锁")

        # 标注 40/20X60
        painter.setPen(QColor("#EDF2F4"))
        painter.setFont(QFont("Consolas", 7))
        painter.drawText(lock_box_x + 32, lock_box_y + 16, "40/20X60\n锁紧缸")

        # 油口标注
        painter.setFont(QFont("Segoe UI", 8))
        painter.drawText(start_x + 10, c1_y - 8, "A口 (无杆腔并联)")
        painter.drawText(start_x + cyl_w - 95, c1_y - 8, "B口 (有杆腔并联)")

        # -------------------------------------------------------------
        # 4. 8x25L 蓄能器组 (106)
        # -------------------------------------------------------------
        acc_x = w - 145
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
        painter.drawText(acc_x + 9, acc_y + 26, f"{self.face_name}面\n8x25L")
        painter.setPen(QColor("#FFD166"))
        painter.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        painter.drawText(acc_x + 2, acc_y + 112, f"{self.p_acc_bar:.1f} bar")

        # -------------------------------------------------------------
        # 5. 底部阀组拓扑卡片
        # -------------------------------------------------------------
        valve_box_y = 220
        # 比例阀 102A
        v_col = QColor("#06D6A0") if not self.is_feathering and not self.is_locked else QColor("#8D99AE")
        painter.setPen(QPen(v_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(start_x, valve_box_y, 135, 52, 4, 4)
        painter.setPen(v_col)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 14, valve_box_y + 22, "比例伺服阀 102A")
        painter.setFont(QFont("Consolas", 9))
        painter.drawText(start_x + 25, valve_box_y + 41, f"开度: {self.valve_pos_pct:+.1f} %")

        # 顺桨节流元件 121 (充液) / 122 (快放)
        f_col = QColor("#EF233C") if self.is_feathering else QColor("#4A4E69")
        painter.setPen(QPen(f_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(start_x + 145, valve_box_y, 150, 52, 4, 4)
        painter.setPen(QColor("#EDF2F4"))
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 152, valve_box_y + 22, f"顺桨充液 121 (Φ{self.d_121_mm:.1f}mm)")
        painter.drawText(start_x + 152, valve_box_y + 41, f"快放调速 122 (Φ{self.d_122_mm:.1f}mm)")

        # 变桨锁电磁阀 130 / 132 卡片
        lock_valv_col = QColor("#06D6A0") if self.switch_S_unlocked else QColor("#FFB703")
        painter.setPen(QPen(lock_valv_col, 1.8))
        painter.drawRoundedRect(start_x + 305, valve_box_y, 140, 52, 4, 4)
        painter.setPen(lock_valv_col)
        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.drawText(start_x + 315, valve_box_y + 22, "变桨锁阀 130/132")
        painter.setFont(QFont("Consolas", 9))
        painter.setPen(QColor("#EDF2F4"))
        painter.drawText(start_x + 315, valve_box_y + 41, f"PP压: {self.p_PP_bar:4.1f} bar")

        # P/T 供油与液压站 (HPU) 动力单元拓扑
        hpu_box_x = start_x + 455
        hpu_box_w = max(195, w - hpu_box_x - 15)
        hpu_border_col = QColor("#06D6A0") if self.valve_25_state else QColor("#4A4E69")
        painter.setPen(QPen(hpu_border_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(hpu_box_x, valve_box_y, hpu_box_w, 52, 4, 4)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        hpu_mode_txt = "🟢 25#得电打压" if self.valve_25_state else "⚪ 25#失电卸荷"
        painter.setPen(hpu_border_col)
        painter.drawText(hpu_box_x + 8, valve_box_y + 17, f"HPU站: {hpu_mode_txt}")

        painter.setFont(QFont("Consolas", 8))
        painter.setPen(QColor("#4CC9F0"))
        painter.drawText(hpu_box_x + 8, valve_box_y + 32, f"P口: {self.p_supply_bar:4.1f} bar | {self.q_hpu_lpm:4.1f} L/m")
        painter.setPen(QColor("#8D99AE"))
        painter.drawText(hpu_box_x + 8, valve_box_y + 46, f"溢流: 280 bar | 3泵 1450rpm")


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

        # 初始化 CODESYS OPC UA 桥接网关 (200Hz / 5ms 纳秒级对齐，选择远程控制时自动开启通讯)
        self.opcua_bridge = PitchOpcUaBridge(
            endpoint="opc.tcp://127.0.0.1:4840",
            engine=self.engine,
            cycle_time_s=0.005
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

        splitter.setStretchFactor(0, 4)
        splitter.setStretchFactor(1, 6)
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
        self.chk_sync_all.setStyleSheet("color: #FFD166; font-size: 11px; font-weight: bold;")
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
        self.spin_target.setFixedWidth(85)

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
        self.spin_voltage.setFixedWidth(85)

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

        self.lbl_lock_detail = QLabel("状态: 🟢 已解锁 (0.0 mm) | 90°对准: 否")
        self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold;")
        lock_layout.addWidget(self.lbl_lock_detail)

        layout.addWidget(self.lock_box)

        # -------------------------------------------------------------
        # 4. 应急顺桨
        # -------------------------------------------------------------
        self.safe_box = QGroupBox(" 应急顺桨 ")
        safe_layout = QVBoxLayout(self.safe_box)
        safe_layout.setContentsMargins(8, 8, 8, 8)
        safe_layout.setSpacing(6)

        self.btn_feather = QPushButton("🛑 紧急顺桨")
        self.btn_feather.setFixedHeight(32)
        self.btn_feather.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_feather.clicked.connect(self._on_toggle_feather)
        safe_layout.addWidget(self.btn_feather)

        self.lbl_safe_status = QLabel("状态: 正常运行")
        self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px;")
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
            ('A', '【A 面】变桨机构 (Blade A / 轴 1)'),
            ('B', '【B 面】变桨机构 (Blade B / 轴 2)'),
            ('C', '【C 面】变桨机构 (Blade C / 轴 3)')
        ]

        for k, face_title in tabs_info:
            tab_page = self._build_face_tab_page(k)
            self.tab_widget.addTab(tab_page, face_title)

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
        page_layout.addWidget(schematic, stretch=4)

        # 2. pyqtgraph 示波器
        scope_box = QGroupBox(f" {k} 面软实时双通道示波器 (Oscilloscope Strip Charts) ")
        scope_layout = QVBoxLayout(scope_box)
        scope_layout.setContentsMargins(6, 8, 6, 6)

        plot_widget = pg.GraphicsLayoutWidget()
        plot_widget.setBackground("#121318")
        self.plot_widgets[k] = plot_widget

        # 通道 1: 位移
        p1 = plot_widget.addPlot(row=0, col=0)
        p1.showGrid(x=True, y=True, alpha=0.3)
        p1.setLabel('left', f'{k}面 位移 (mm)', color='#4CC9F0')
        p1.setYRange(0, 850, padding=0.05)
        p1.addLegend(offset=(10, 10), brush="#1E202B")

        self.curves_target[k] = p1.plot(
            pen=pg.mkPen(color='#E76F51', width=2, style=Qt.PenStyle.DashLine),
            name="目标位移指令"
        )
        self.curves_actual[k] = p1.plot(
            pen=pg.mkPen(color='#4CC9F0', width=2.5),
            name=f"{k}面双缸实际行程"
        )

        # 通道 2: 压力
        p2 = plot_widget.addPlot(row=1, col=0)
        p2.showGrid(x=True, y=True, alpha=0.3)
        p2.setLabel('left', f'{k}面 压力 (bar)', color='#FFD166')
        p2.setLabel('bottom', '相对时间 (秒)', color='#A8DADC')
        p2.setYRange(0, 300, padding=0.05)
        p2.addLegend(offset=(10, 10), brush="#1E202B")

        p2.addLine(y=235, pen=pg.mkPen(color='#EF233C', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=265, pen=pg.mkPen(color='#06D6A0', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=280, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DotLine))

        self.curves_pA[k] = p2.plot(pen=pg.mkPen(color='#EF233C', width=2), name=f"{k}面 p_A (无杆腔)")
        self.curves_pB[k] = p2.plot(pen=pg.mkPen(color='#48CAE4', width=2), name=f"{k}面 p_B (有杆腔)")
        self.curves_pacc[k] = p2.plot(
            pen=pg.mkPen(color='#FFB703', width=2, style=Qt.PenStyle.DashLine),
            name=f"{k}面 p_107 (母管蓄能器)"
        )
        self.curves_pSupply[k] = p2.plot(
            pen=pg.mkPen(color='#4CC9F0', width=1.5, style=Qt.PenStyle.DotLine),
            name="p_P (供油口)"
        )

        scope_layout.addWidget(plot_widget)
        page_layout.addWidget(scope_box, stretch=6)

        return page

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
            self.lbl_safe_status.setStyleSheet("color: #EF233C; font-weight: bold; font-size: 11px;")
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
            self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px;")

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
            self.lbl_lock_detail.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold;")
        else:
            lock_str = "🟢 已解锁" if active_t.switch_S_unlocked else f"⏳ 动作中 ({active_t.lock_pin_mm:4.1f}mm)"
            self.lbl_lock_detail.setText(f"状态: {lock_str} | {align_str}")
            self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold;")

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
                self.lbl_auth_status.setStyleSheet("color: #06D6A0; font-size: 12px; font-weight: bold;")
            else:
                self.lbl_auth_status.setText("🔄 连接中 (4840)...")
                self.lbl_auth_status.setStyleSheet("color: #FFB703; font-size: 12px; font-weight: bold;")

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
