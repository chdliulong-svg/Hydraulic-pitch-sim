"""
兆瓦级风机液压变桨仿真系统 - 6MW 集中润滑系统监控选项卡
包含双回路独立状态卡片、递进分配器活塞动作反馈、故障注入开关及双通道示波器
"""

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QProgressBar,
    QGroupBox, QCheckBox, QPushButton, QFrame, QScrollArea
)
from PyQt6.QtCore import Qt, pyqtSignal
import pyqtgraph as pg

from pitch_lubrication_sim import LubricationTelemetry


class LubricationTabWidget(QWidget):
    """6MW 轮毂递进式集中润滑系统监控选项卡"""

    togglePump1 = pyqtSignal()
    togglePump2 = pyqtSignal()
    faultsChanged = pyqtSignal(bool, bool, bool, bool)  # tank1_empty, tank2_empty, block1, block2

    def __init__(self, buf_size=300, parent=None):
        super().__init__(parent)
        self.buf_size = buf_size

        # 示波器历史缓存
        self.lub_p1_hist = np.zeros(self.buf_size)
        self.lub_p2_hist = np.zeros(self.buf_size)
        self.lub_piston1_a_hist = np.zeros(self.buf_size)
        self.lub_piston1_b_hist = np.zeros(self.buf_size)
        self.lub_piston1_c_hist = np.zeros(self.buf_size)

        self._init_ui()

    def _init_ui(self):
        root_layout = QVBoxLayout(self)
        root_layout.setContentsMargins(0, 0, 0, 0)

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

        # --- 1# 主回路卡片 ---
        box1 = QGroupBox(" 1# 主润滑回路 (pac.io.dOut.hubLubPump1On) ")
        box1.setStyleSheet(group_style)
        box1_layout = QVBoxLayout(box1)
        box1_layout.setContentsMargins(12, 10, 12, 10)
        box1_layout.setSpacing(8)

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

        ctrl1_frame = QFrame()
        ctrl1_frame.setStyleSheet("background-color: transparent;")
        ctrl1_vbox = QVBoxLayout(ctrl1_frame)
        ctrl1_vbox.setContentsMargins(0, 4, 0, 0)
        ctrl1_vbox.setSpacing(6)

        self.btn_lub1_test_toggle = QPushButton("▶ 手动启动 1# 泵 (本地测试)")
        self.btn_lub1_test_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lub1_test_toggle.setStyleSheet("background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;")
        self.btn_lub1_test_toggle.clicked.connect(self.togglePump1.emit)
        ctrl1_vbox.addWidget(self.btn_lub1_test_toggle)

        faults1_row = QHBoxLayout()
        self.chk_lub1_low_level = QCheckBox("低油位故障")
        self.chk_lub1_low_level.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub1_low_level.toggled.connect(self._emit_faults)

        self.chk_lub1_block = QCheckBox("分配器堵塞")
        self.chk_lub1_block.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub1_block.toggled.connect(self._emit_faults)

        faults1_row.addWidget(self.chk_lub1_low_level)
        faults1_row.addWidget(self.chk_lub1_block)
        ctrl1_vbox.addLayout(faults1_row)
        box1_layout.addWidget(ctrl1_frame)
        circuits_layout.addWidget(box1)

        # --- 2# 冗余回路卡片 ---
        box2 = QGroupBox(" 2# 冗余润滑回路 (pac.io.dOut.hubLubPump2On) ")
        box2.setStyleSheet(group_style)
        box2_layout = QVBoxLayout(box2)
        box2_layout.setContentsMargins(12, 10, 12, 10)
        box2_layout.setSpacing(8)

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

        ctrl2_frame = QFrame()
        ctrl2_frame.setStyleSheet("background-color: transparent;")
        ctrl2_vbox = QVBoxLayout(ctrl2_frame)
        ctrl2_vbox.setContentsMargins(0, 4, 0, 0)
        ctrl2_vbox.setSpacing(6)

        self.btn_lub2_test_toggle = QPushButton("▶ 手动启动 2# 泵 (本地测试)")
        self.btn_lub2_test_toggle.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lub2_test_toggle.setStyleSheet("background-color: #05668D; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;")
        self.btn_lub2_test_toggle.clicked.connect(self.togglePump2.emit)
        ctrl2_vbox.addWidget(self.btn_lub2_test_toggle)

        faults2_row = QHBoxLayout()
        self.chk_lub2_low_level = QCheckBox("低油位故障")
        self.chk_lub2_low_level.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub2_low_level.toggled.connect(self._emit_faults)

        self.chk_lub2_block = QCheckBox("分配器堵塞")
        self.chk_lub2_block.setStyleSheet("color: #8D99AE; font-size: 11px;")
        self.chk_lub2_block.toggled.connect(self._emit_faults)

        faults2_row.addWidget(self.chk_lub2_low_level)
        faults2_row.addWidget(self.chk_lub2_block)
        ctrl2_vbox.addLayout(faults2_row)
        box2_layout.addWidget(ctrl2_frame)
        circuits_layout.addWidget(box2)

        page_layout.addLayout(circuits_layout)

        # 3. 底部动态曲线示波器
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
        root_layout.addWidget(scroll)

    def _emit_faults(self):
        self.faultsChanged.emit(
            self.chk_lub1_low_level.isChecked(),
            self.chk_lub2_low_level.isChecked(),
            self.chk_lub1_block.isChecked(),
            self.chk_lub2_block.isChecked()
        )

    def update_pump_toggle_btn(self, pump_idx: int, is_running: bool):
        btn = self.btn_lub1_test_toggle if pump_idx == 1 else self.btn_lub2_test_toggle
        txt = f"⏹ 停止 {pump_idx}# 泵 (本地测试)" if is_running else f"▶ 手动启动 {pump_idx}# 泵 (本地测试)"
        bg = "#D90429" if is_running else "#05668D"
        btn.setText(txt)
        btn.setStyleSheet(f"background-color: {bg}; color: #FFFFFF; font-weight: bold; border-radius: 4px; padding: 6px 12px; font-size: 11px;")

    def update_telemetry(self, lub: LubricationTelemetry, time_hist):
        """实时刷新卡片遥测与示波器曲线"""
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

        self.curve_lub_p1.setData(time_hist, self.lub_p1_hist)
        self.curve_lub_p2.setData(time_hist, self.lub_p2_hist)
        self.curve_lub_pa.setData(time_hist, self.lub_piston1_a_hist)
        self.curve_lub_pb.setData(time_hist, self.lub_piston1_b_hist)
        self.curve_lub_pc.setData(time_hist, self.lub_piston1_c_hist)
