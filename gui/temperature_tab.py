"""
兆瓦级风机液压变桨仿真系统 - 6MW 温度监控系统选项卡
包含油温/电气柜温/环境温/CPU/轴承/气隙 6 大测点卡片、环境调节与故障注入开关及多曲线示波器
"""

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QProgressBar, QGroupBox, QCheckBox, QSlider, QFrame, QScrollArea
)
from PyQt6.QtCore import Qt, pyqtSignal
import pyqtgraph as pg

from pitch_temperature_sim import TemperatureTelemetry


class TemperatureTabWidget(QWidget):
    """6MW 轮毂与液压系统热力学/温度监控仿真选项卡"""

    ambientChanged = pyqtSignal(float)
    faultsChanged = pyqtSignal(bool, bool, bool, bool)  # oil_overheat, extreme_cold, fan_fail, pt100_open

    def __init__(self, buf_size=300, parent=None):
        super().__init__(parent)
        self.buf_size = buf_size

        # 示波器历史缓存
        self.temp_oil_hist = np.full(self.buf_size, 42.5)
        self.temp_cab_hist = np.full(self.buf_size, 32.0)
        self.temp_hub_hist = np.full(self.buf_size, 25.0)
        self.temp_bearing_hist = np.full(self.buf_size, 35.0)
        self.temp_cpu_hist = np.full(self.buf_size, 45.0)

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

        (c1, self.lbl_temp_oil, self.lbl_stat_oil, self.bar_temp_oil, self.lbl_sub_oil) = _make_temp_card("1. 液压站油箱油温", "pac.io.ai.tempOil")
        self.lbl_sub_oil.setText("门限: -20°C 极寒下限 | 58°C 冷却 | 70°C 高温")
        grid.addWidget(c1, 0, 0)

        (c2, self.lbl_temp_cab, self.lbl_stat_cab, self.bar_temp_cab, self.lbl_sub_cab) = _make_temp_card("2. 变桨控制电气柜温", "pac.io.ai.tempCabinet")
        self.lbl_sub_cab.setText("门限: -10°C 柜内低温 | 60°C 柜内高温 (风扇连锁)")
        grid.addWidget(c2, 0, 1)

        (c3, self.lbl_temp_hub, self.lbl_stat_hub, self.bar_temp_hub, self.lbl_sub_hub) = _make_temp_card("3. 轮毂内部环境温", "pac.io.ai.tempHub")
        self.lbl_sub_hub.setText("门限: -25°C 轮毂低温 | 55°C 轮毂高温")
        grid.addWidget(c3, 0, 2)

        (c4, self.lbl_temp_cpu, self.lbl_stat_cpu, self.bar_temp_cpu, self.lbl_sub_cpu) = _make_temp_card("4. 控制器 CPU 核心温", "pac.io.ai.tempCPU")
        self.lbl_sub_cpu.setText("门限: 80°C CPU 高温告警 (芯片保护)")
        grid.addWidget(c4, 1, 0)

        (c5, self.lbl_temp_bearing, self.lbl_stat_bearing, self.bar_temp_bearing, self.lbl_sub_bearing) = _make_temp_card("5. 变桨/主轴承温度", "pac.io.ai.tempBearing")
        self.lbl_sub_bearing.setText("工况: 双泵注脂润滑散热耦合")
        grid.addWidget(c5, 1, 1)

        (c6, self.lbl_temp_airgap, self.lbl_stat_airgap, self.bar_temp_airgap, self.lbl_sub_airgap) = _make_temp_card("6. 发电机气隙温度", "pac.io.ai.tempAirgab")
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
        self.slider_ambient.valueChanged.connect(self._on_slider_changed)
        amb_row.addWidget(self.slider_ambient)

        self.lbl_ambient_val = QLabel("25.0 °C")
        self.lbl_ambient_val.setStyleSheet("font-family: Consolas; font-size: 13px; font-weight: bold; color: #4CC9F0; min-width: 60px;")
        amb_row.addWidget(self.lbl_ambient_val)
        ctrl_layout.addLayout(amb_row)

        fault_row = QHBoxLayout()
        fault_row.setSpacing(14)
        self.chk_fault_oil_overheat = QCheckBox("⚠️ 油温过高故障 (>70°C)")
        self.chk_fault_extreme_cold = QCheckBox("❄️ 极寒冷态环境 (-28°C)")
        self.chk_fault_fan_fail = QCheckBox("❌ 柜顶风扇停机")
        self.chk_fault_pt100_open = QCheckBox("⚡ PT100断线开路")

        for chk in [self.chk_fault_oil_overheat, self.chk_fault_extreme_cold,
                    self.chk_fault_fan_fail, self.chk_fault_pt100_open]:
            chk.setStyleSheet("color: #8D99AE; font-size: 11px;")
            chk.toggled.connect(self._emit_faults)
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
        root_layout.addWidget(scroll)

    def _on_slider_changed(self, val):
        t = float(val)
        self.lbl_ambient_val.setText(f"{t:+4.1f} °C")
        self.ambientChanged.emit(t)

    def _emit_faults(self):
        self.faultsChanged.emit(
            self.chk_fault_oil_overheat.isChecked(),
            self.chk_fault_extreme_cold.isChecked(),
            self.chk_fault_fan_fail.isChecked(),
            self.chk_fault_pt100_open.isChecked()
        )

    def update_telemetry(self, tmp: TemperatureTelemetry, time_hist):
        """实时刷新卡片数值、状态药丸与示波器曲线"""
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

        self.curve_temp_oil.setData(time_hist, self.temp_oil_hist)
        self.curve_temp_cab.setData(time_hist, self.temp_cab_hist)
        self.curve_temp_hub.setData(time_hist, self.temp_hub_hist)
        self.curve_temp_bearing.setData(time_hist, self.temp_bearing_hist)
        self.curve_temp_cpu.setData(time_hist, self.temp_cpu_hist)
