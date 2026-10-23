"""
兆瓦级风机液压变桨仿真系统 - 单叶片监控选项卡
包含动态机械液压拓扑图与双通道示波器 (位移/压力实时曲线)
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QGroupBox
from PyQt6.QtCore import Qt
import pyqtgraph as pg

from gui.schematic_widget import HydraulicSchematicWidget


class BladeFaceTabWidget(QWidget):
    """单叶片 (A/B/C) 动态拓扑与实时运行曲线选项卡"""

    def __init__(self, face_name="A", parent=None):
        super().__init__(parent)
        self.face_name = face_name

        page_layout = QVBoxLayout(self)
        page_layout.setContentsMargins(6, 6, 6, 6)
        page_layout.setSpacing(6)

        # 1. 动态拓扑图
        self.schematic = HydraulicSchematicWidget(face_name=face_name)
        page_layout.addWidget(self.schematic)

        # 2. pyqtgraph 示波器
        scope_box = QGroupBox(f" 叶片 {face_name} 实时运行曲线 ")
        scope_layout = QVBoxLayout(scope_box)
        scope_layout.setContentsMargins(6, 8, 6, 6)

        self.plot_widget = pg.GraphicsLayoutWidget()
        self.plot_widget.setBackground("#121318")

        # 通道 1: 位移
        p1 = self.plot_widget.addPlot(row=0, col=0)
        p1.showGrid(x=True, y=True, alpha=0.3)
        p1.setLabel('left', '位移 (mm)', color='#4CC9F0')
        p1.setYRange(0, 850, padding=0.05)
        p1.addLegend(offset=(10, 10), brush="#1E202B")

        self.curve_target = p1.plot(
            pen=pg.mkPen(color='#E76F51', width=2, style=Qt.PenStyle.DashLine),
            name="目标位移"
        )
        self.curve_actual = p1.plot(
            pen=pg.mkPen(color='#4CC9F0', width=2.5),
            name="实际行程"
        )

        # 通道 2: 压力
        p2 = self.plot_widget.addPlot(row=1, col=0)
        p2.showGrid(x=True, y=True, alpha=0.3)
        p2.setLabel('left', '压力 (bar)', color='#FFD166')
        p2.setLabel('bottom', '相对时间 (s)', color='#A8DADC')
        p2.setYRange(0, 300, padding=0.05)
        p2.addLegend(offset=(10, 10), brush="#1E202B")

        p2.addLine(y=235, pen=pg.mkPen(color='#EF233C', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=265, pen=pg.mkPen(color='#06D6A0', width=1.0, style=Qt.PenStyle.DashDotLine))
        p2.addLine(y=280, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DotLine))

        self.curve_pA = p2.plot(pen=pg.mkPen(color='#EF233C', width=2), name="p_A (无杆腔)")
        self.curve_pB = p2.plot(pen=pg.mkPen(color='#48CAE4', width=2), name="p_B (有杆腔)")
        self.curve_pacc = p2.plot(
            pen=pg.mkPen(color='#FFB703', width=2, style=Qt.PenStyle.DashLine),
            name="p_acc (蓄能器)"
        )
        self.curve_pSupply = p2.plot(
            pen=pg.mkPen(color='#4CC9F0', width=1.5, style=Qt.PenStyle.DotLine),
            name="p_P (供油口)"
        )

        scope_layout.addWidget(self.plot_widget)
        page_layout.addWidget(scope_box, stretch=1)

    def update_telemetry(self, t, is_feathering, time_hist, target_hist, stroke_hist,
                         pA_hist, pB_hist, pacc_hist, pSupply_hist):
        """同步刷新动态拓扑画布与实时示波器曲线"""
        self.schematic.update_telemetry(
            stroke_mm=t.displacement_mm,
            p_A=t.p_A_bar,
            p_B=t.p_B_bar,
            p_acc=t.p_acc_bar,
            valve_pos_pct=t.valve_position_pct,
            is_feathering=is_feathering,
            lock_pin_mm=t.lock_pin_mm,
            is_locked=t.is_locked,
            switch_129=t.switch_129,
            switch_S_unlocked=t.switch_S_unlocked,
            p_PP_bar=t.p_PP_bar,
            valve_25_state=t.valve_25_state,
            p_supply_bar=t.p_supply_bar,
            q_hpu_lpm=t.q_hpu_lpm,
            motor_running=getattr(t, 'motor_running', True),
            motor_rpm=getattr(t, 'motor_rpm', 1450.0)
        )

        # 刷新示波器曲线
        self.curve_target.setData(time_hist, target_hist)
        self.curve_actual.setData(time_hist, stroke_hist)
        self.curve_pA.setData(time_hist, pA_hist)
        self.curve_pB.setData(time_hist, pB_hist)
        self.curve_pacc.setData(time_hist, pacc_hist)
        self.curve_pSupply.setData(time_hist, pSupply_hist)
