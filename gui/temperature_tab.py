"""
兆瓦级风机液压变桨仿真系统 - 6MW 温度监控系统选项卡
包含电气柜温/轮毂环温/CPU/轴承/气隙 5 大测点卡片与温控状态概览、环境调节与故障注入开关及多曲线示波器
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
    """6MW 轮毂与变桨系统热力学/温度监控仿真选项卡"""

    ambientChanged = pyqtSignal(float)
    faultsChanged = pyqtSignal(bool, bool, bool)  # extreme_cold, fan_fail, pt100_open

    def __init__(self, buf_size=300, parent=None):
        super().__init__(parent)
        self.buf_size = buf_size

        # 示波器历史缓存 (电气柜温、轮毂环温、轴承)
        self.temp_cab_hist = np.full(self.buf_size, 32.0)
        self.temp_hub_hist = np.full(self.buf_size, 25.0)
        self.temp_bearing_hist = np.full(self.buf_size, 35.0)

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
        b_title = QLabel("6MW 轮毂电气与轴承热力学/温度监控仿真")
        b_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #EDF2F4;")
        b_sub = QLabel("电气柜温 · 轮毂环温 · 变桨轴承温 · 发电机气隙温")
        b_sub.setStyleSheet("font-size: 11px; color: #8D99AE;")
        title_vbox.addWidget(b_title)
        title_vbox.addWidget(b_sub)
        banner_layout.addLayout(title_vbox)

        banner_layout.addStretch()

        self.lbl_temp_banner_cab = QLabel("柜温: 32.0°C (正常)")
        self.lbl_temp_banner_cab.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_cab)

        self.lbl_temp_banner_hub = QLabel("轮毂: 25.0°C (正常)")
        self.lbl_temp_banner_hub.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_hub)

        self.lbl_temp_banner_fan = QLabel("风扇: 🟢 运行中")
        self.lbl_temp_banner_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        banner_layout.addWidget(self.lbl_temp_banner_fan)

        page_layout.addWidget(banner)

        # 2. 中部温度监控卡片 (2x3 网格布局)
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

        # 4 大核心测点卡片
        (c1, self.lbl_temp_cab, self.lbl_stat_cab, self.bar_temp_cab, self.lbl_sub_cab) = _make_temp_card("1. 变桨控制电气柜温", "pac.io.ai.tempCabinet")
        self.lbl_sub_cab.setText("门限: -10°C 柜内低温 | 60°C 柜内高温 (风扇连锁)")
        grid.addWidget(c1, 0, 0)

        (c2, self.lbl_temp_hub, self.lbl_stat_hub, self.bar_temp_hub, self.lbl_sub_hub) = _make_temp_card("2. 轮毂内部环境温", "pac.io.ai.tempHub")
        self.lbl_sub_hub.setText("门限: -25°C 轮毂低温 | 55°C 轮毂高温")
        grid.addWidget(c2, 0, 1)

        (c4, self.lbl_temp_bearing, self.lbl_stat_bearing, self.bar_temp_bearing, self.lbl_sub_bearing) = _make_temp_card("3. 变桨/主轴承温度", "pac.io.ai.tempBearing")
        self.lbl_sub_bearing.setText("工况: 双泵注脂润滑散热耦合")
        grid.addWidget(c4, 1, 0)

        (c5, self.lbl_temp_airgap, self.lbl_stat_airgap, self.bar_temp_airgap, self.lbl_sub_airgap) = _make_temp_card("4. 发电机气隙温度", "pac.io.ai.tempAirgab")
        self.lbl_sub_airgap.setText("工况: 机舱气隙热耗散平衡")
        grid.addWidget(c5, 1, 1)

        # 第 5 块卡片: 温控与传感器回路健康概览
        c6 = QFrame()
        c6.setStyleSheet(card_style)
        c6_layout = QVBoxLayout(c6)
        c6_layout.setContentsMargins(12, 10, 12, 10)
        c6_layout.setSpacing(6)

        c6_top = QHBoxLayout()
        c6_title = QLabel("5. 温控与传感器回路概况")
        c6_title.setStyleSheet("font-size: 12px; font-weight: bold; color: #EDF2F4;")
        c6_tag = QLabel("pac.io.status")
        c6_tag.setStyleSheet("font-size: 10px; color: #8D99AE; font-family: Consolas;")
        c6_top.addWidget(c6_title)
        c6_top.addStretch()
        c6_top.addWidget(c6_tag)
        c6_layout.addLayout(c6_top)

        self.lbl_ov_fan = QLabel("• 散热风扇: 🟢 运行中 (温控自启)")
        self.lbl_ov_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")
        self.lbl_ov_pt100 = QLabel("• PT100回路: 🟢 4通道测量正常")
        self.lbl_ov_pt100.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")
        self.lbl_ov_alarm = QLabel("• 温度告警: 🟢 无任何温度越限")
        self.lbl_ov_alarm.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")

        c6_layout.addWidget(self.lbl_ov_fan)
        c6_layout.addWidget(self.lbl_ov_pt100)
        c6_layout.addWidget(self.lbl_ov_alarm)
        c6_layout.addStretch()
        grid.addWidget(c6, 0, 2, 2, 1)

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
        self.chk_fault_extreme_cold = QCheckBox("❄️ 极寒冷态环境 (-28°C)")
        self.chk_fault_fan_fail = QCheckBox("❌ 柜顶风扇停机故障")
        self.chk_fault_pt100_open = QCheckBox("⚡ PT100断线开路故障")

        for chk in [self.chk_fault_extreme_cold, self.chk_fault_fan_fail, self.chk_fault_pt100_open]:
            chk.setStyleSheet("color: #8D99AE; font-size: 11px;")
            chk.toggled.connect(self._emit_faults)
            fault_row.addWidget(chk)
        fault_row.addStretch()
        ctrl_layout.addLayout(fault_row)
        page_layout.addWidget(ctrl_box)

        # 4. 底部实时示波器
        scope_box = QGroupBox(" 轮毂电气与轴承温度实时运行曲线 ")
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
        tp.addLegend(offset=(10, 10), brush="#1E202B", colCount=3)

        tp.addLine(y=80, pen=pg.mkPen(color='#D90429', width=1.0, style=Qt.PenStyle.DashLine))
        tp.addLine(y=60, pen=pg.mkPen(color='#F72585', width=1.0, style=Qt.PenStyle.DotLine))
        tp.addLine(y=-20, pen=pg.mkPen(color='#4CC9F0', width=1.0, style=Qt.PenStyle.DashDotLine))

        self.curve_temp_cab = tp.plot(pen=pg.mkPen(color='#4CC9F0', width=2), name="电气柜温")
        self.curve_temp_hub = tp.plot(pen=pg.mkPen(color='#06D6A0', width=2), name="轮毂环温")
        self.curve_temp_bearing = tp.plot(pen=pg.mkPen(color='#B5179E', width=2), name="变桨轴承")

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

        _render_val(self.lbl_temp_cab, self.lbl_stat_cab, self.bar_temp_cab, tmp.temp_cabinet_deg, -10.0, 60.0)
        _render_val(self.lbl_temp_hub, self.lbl_stat_hub, self.bar_temp_hub, tmp.temp_hub_deg, -25.0, 55.0)
        _render_val(self.lbl_temp_bearing, self.lbl_stat_bearing, self.bar_temp_bearing, tmp.temp_bearing_deg, -20.0, 65.0)
        _render_val(self.lbl_temp_airgap, self.lbl_stat_airgap, self.bar_temp_airgap, tmp.temp_airgap_deg, -20.0, 65.0)

        # 顶部横幅药丸状态
        if tmp.alarm_cab_high:
            self.lbl_temp_banner_cab.setText("柜温: 🔴 超上限报警")
            self.lbl_temp_banner_cab.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        elif tmp.alarm_cab_low:
            self.lbl_temp_banner_cab.setText("柜温: ❄️ 超下限报警")
            self.lbl_temp_banner_cab.setStyleSheet("color: #4CC9F0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B38; border: 1px solid #22455C; border-radius: 4px;")
        else:
            self.lbl_temp_banner_cab.setText(f"柜温: 🟢 正常 ({tmp.temp_cabinet_deg:4.1f}°C)")
            self.lbl_temp_banner_cab.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

        if tmp.alarm_hub_high:
            self.lbl_temp_banner_hub.setText("轮毂: 🔴 超上限报警")
            self.lbl_temp_banner_hub.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        elif tmp.alarm_hub_low:
            self.lbl_temp_banner_hub.setText("轮毂: ❄️ 超下限报警")
            self.lbl_temp_banner_hub.setStyleSheet("color: #4CC9F0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B38; border: 1px solid #22455C; border-radius: 4px;")
        else:
            self.lbl_temp_banner_hub.setText(f"轮毂: 🟢 正常 ({tmp.temp_hub_deg:4.1f}°C)")
            self.lbl_temp_banner_hub.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

        if tmp.cabinet_fan:
            self.lbl_temp_banner_fan.setText("风扇: 🟢 运行中")
            self.lbl_temp_banner_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
            self.lbl_ov_fan.setText("• 散热风扇: 🟢 运行中 (温控排热)")
            self.lbl_ov_fan.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")
        else:
            self.lbl_temp_banner_fan.setText("风扇: 🔴 停机")
            self.lbl_temp_banner_fan.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px 10px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
            self.lbl_ov_fan.setText("• 散热风扇: 🔴 停机故障 (排热受阻)")
            self.lbl_ov_fan.setStyleSheet("color: #EF233C; font-size: 11px; font-family: Consolas;")

        if tmp.sensor_fault_open:
            self.lbl_ov_pt100.setText("• PT100回路: ⚡ 存在断线开路故障")
            self.lbl_ov_pt100.setStyleSheet("color: #EF233C; font-size: 11px; font-family: Consolas;")
        else:
            self.lbl_ov_pt100.setText("• PT100回路: 🟢 4通道测量正常")
            self.lbl_ov_pt100.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")

        has_alarm = tmp.alarm_cab_high or tmp.alarm_cab_low or tmp.alarm_hub_high or tmp.alarm_hub_low
        if has_alarm:
            self.lbl_ov_alarm.setText("• 温度告警: 🔴 存在温度越限超标")
            self.lbl_ov_alarm.setStyleSheet("color: #EF233C; font-size: 11px; font-family: Consolas;")
        else:
            self.lbl_ov_alarm.setText("• 温度告警: 🟢 无任何温度越限")
            self.lbl_ov_alarm.setStyleSheet("color: #06D6A0; font-size: 11px; font-family: Consolas;")

        # 示波器曲线更新
        safe_cab = tmp.temp_cabinet_deg if tmp.temp_cabinet_deg > -900.0 else 0.0
        safe_hub = tmp.temp_hub_deg if tmp.temp_hub_deg > -900.0 else 0.0
        safe_bearing = tmp.temp_bearing_deg if tmp.temp_bearing_deg > -900.0 else 0.0

        self.temp_cab_hist = np.roll(self.temp_cab_hist, -1)
        self.temp_cab_hist[-1] = safe_cab
        self.temp_hub_hist = np.roll(self.temp_hub_hist, -1)
        self.temp_hub_hist[-1] = safe_hub
        self.temp_bearing_hist = np.roll(self.temp_bearing_hist, -1)
        self.temp_bearing_hist[-1] = safe_bearing

        self.curve_temp_cab.setData(time_hist, self.temp_cab_hist)
        self.curve_temp_hub.setData(time_hist, self.temp_hub_hist)
        self.curve_temp_bearing.setData(time_hist, self.temp_bearing_hist)
