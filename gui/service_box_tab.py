"""
=============================================================================
兆瓦级风电机组液压变桨控制系统
机旁就地手动操作终端 (Local Service Box Pendant) 仿真选项卡
=============================================================================
功能:
1. 拟物化模拟轮毂内部机旁手操盒 (remoteBox)，支持暗黑工业级人机交互。
2. 包含航插物理接入开关、3 档叶片选择旋钮 (Blade A / B / C)、速度档位切换。
3. 真实工控点动按键 (Fine 开桨 / Feather 顺桨 / 变桨锁拔销与插销)，支持按下使能与松开复位。
4. 0° 零位物理基准标定按键与故障复位 (Clear CMD) 按键。
5. 三轴安全状态矩阵监控 (bladeSafe[0..2] 与 nSafeBlades >= 2 联锁安全评估)。
6. 当前受控叶片实时数字液晶仪表与点动动态示波器。
=============================================================================
"""

import numpy as np
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QGroupBox, QRadioButton, QButtonGroup, QFrame,
    QProgressBar, QScrollArea, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal
import pyqtgraph as pg

from pitch_soft_realtime import ServiceBoxTelemetry, PitchTelemetryData


class ServiceBoxTabWidget(QWidget):
    """机旁就地手持操作终端仿真选项卡组件"""

    # 信号定义
    pendantConnectedChanged = pyqtSignal(bool)
    bladeChosenChanged = pyqtSignal(str)          # 'A', 'B', 'C'
    fineJogChanged = pyqtSignal(bool)             # True=按下点动, False=松开
    featherJogChanged = pyqtSignal(bool)          # True=按下点动, False=松开
    speedModeChanged = pyqtSignal(bool)           # True=快速, False=慢速
    lockEngageTriggered = pyqtSignal(bool)        # True=按下
    lockDisengageTriggered = pyqtSignal(bool)     # True=按下
    pos0CalibrateTriggered = pyqtSignal()
    clearAlarmTriggered = pyqtSignal()

    def __init__(self, buf_size=300, parent=None):
        super().__init__(parent)
        self.buf_size = buf_size

        # 示波器历史缓存 (当前受控叶片的位移与比例阀开度)
        self.disp_hist = np.full(self.buf_size, 50.0)
        self.spool_hist = np.zeros(self.buf_size)

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
        banner = self._create_top_banner()
        page_layout.addWidget(banner)

        # 2. 主体区 (左右分栏: 左侧手操盒拟物面板，右侧状态监测与液晶仪表)
        main_hbox = QHBoxLayout()
        main_hbox.setSpacing(12)

        # 左侧: 手操盒手持终端拟物面板 (固定宽度 380px)
        pendant_box = self._create_pendant_panel()
        main_hbox.addWidget(pendant_box, stretch=0)

        # 右侧: 状态监测、三轴安全矩阵与动态示波器
        telemetry_box = self._create_telemetry_panel()
        main_hbox.addWidget(telemetry_box, stretch=1)

        page_layout.addLayout(main_hbox)
        scroll.setWidget(page)
        root_layout.addWidget(scroll)

    # =========================================================================
    # 顶部状态横幅
    # =========================================================================
    def _create_top_banner(self) -> QFrame:
        banner = QFrame()
        banner.setObjectName("serviceBanner")
        banner.setStyleSheet("""
            #serviceBanner {
                background-color: #1E202B;
                border: 1px solid #363A4A;
                border-radius: 6px;
            }
            QLabel {
                border: none;
                background-color: transparent;
            }
        """)
        b_layout = QHBoxLayout(banner)
        b_layout.setContentsMargins(14, 8, 14, 8)
        b_layout.setSpacing(12)

        title_vbox = QVBoxLayout()
        title_vbox.setSpacing(2)
        b_title = QLabel("🕹️ 轮毂机旁就地手动操作终端 (Local Service Box Pendant)")
        b_title.setStyleSheet("font-size: 13px; font-weight: bold; color: #EDF2F4;")
        b_sub = QLabel("用于轮毂就地调试、变桨轴承维护、单叶片微调与机械锁插拔操作 (带安全联锁保护)")
        b_sub.setStyleSheet("font-size: 11px; color: #8D99AE;")
        title_vbox.addWidget(b_title)
        title_vbox.addWidget(b_sub)
        b_layout.addLayout(title_vbox)

        b_layout.addStretch()

        # 航插接入状态
        self.lbl_banner_plug = QLabel("🔌 航插: 未接入")
        self.lbl_banner_plug.setStyleSheet("font-size: 11px; font-weight: bold; color: #8D99AE; padding: 4px 10px; background-color: #2B2E3B; border-radius: 4px;")
        b_layout.addWidget(self.lbl_banner_plug)

        # 控制权模式
        self.lbl_banner_auth = QLabel("🌐 权限: 远程系统主控")
        self.lbl_banner_auth.setStyleSheet("font-size: 11px; font-weight: bold; color: #8D99AE; padding: 4px 10px; background-color: #2B2E3B; border-radius: 4px;")
        b_layout.addWidget(self.lbl_banner_auth)

        # 安全叶片数量
        self.lbl_banner_safe = QLabel("🛡️ 安全叶片: 3/3")
        self.lbl_banner_safe.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        b_layout.addWidget(self.lbl_banner_safe)

        # 综合报警状态
        self.lbl_banner_alarm = QLabel("🟢 状态: 正常")
        self.lbl_banner_alarm.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        b_layout.addWidget(self.lbl_banner_alarm)

        return banner

    # =========================================================================
    # 左侧: 手操盒拟物控制面板
    # =========================================================================
    def _create_pendant_panel(self) -> QGroupBox:
        grp = QGroupBox("就地手操盒拟物控制终端")
        grp.setFixedWidth(380)
        grp.setStyleSheet("""
            QGroupBox {
                background-color: #161820;
                border: 2px solid #2F3545;
                border-radius: 10px;
                padding-top: 16px;
            }
        """)
        layout = QVBoxLayout(grp)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(12)

        # 1. 物理航插与连接控制
        conn_frame = QFrame()
        conn_frame.setStyleSheet("background-color: #1F232E; border: 1px solid #363D50; border-radius: 6px; padding: 4px;")
        conn_layout = QHBoxLayout(conn_frame)
        conn_layout.setContentsMargins(8, 6, 8, 6)

        self.btn_plug = QPushButton("🔌 插接机旁手操盒航插")
        self.btn_plug.setCheckable(True)
        self.btn_plug.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_plug.setStyleSheet("""
            QPushButton {
                background-color: #2B2E3B;
                color: #EDF2F4;
                font-weight: bold;
                font-size: 12px;
                padding: 8px 12px;
                border-radius: 4px;
                border: 1px solid #43485C;
            }
            QPushButton:hover {
                background-color: #383D4F;
            }
            QPushButton:checked {
                background-color: #1B4D3E;
                color: #06D6A0;
                border: 1px solid #06D6A0;
            }
        """)
        self.btn_plug.toggled.connect(self._on_plug_toggled)
        conn_layout.addWidget(self.btn_plug)

        self.led_plug = QLabel("●")
        self.led_plug.setStyleSheet("color: #555A6E; font-size: 18px;")
        self.led_plug.setToolTip("航插连接电气回路指示")
        conn_layout.addWidget(self.led_plug)
        layout.addWidget(conn_frame)

        # 2. 叶片选择 3 档旋转开关 (Blade A / B / C)
        blade_grp = QGroupBox("受控叶片选择 (3 档旋钮开关)")
        blade_grp.setStyleSheet("QGroupBox { background-color: #1A1D26; border: 1px solid #2B303E; font-size: 11px; }")
        blade_layout = QHBoxLayout(blade_grp)
        blade_layout.setContentsMargins(10, 10, 10, 10)
        blade_layout.setSpacing(8)

        self.btn_bld_a = QPushButton("叶片 A")
        self.btn_bld_b = QPushButton("叶片 B")
        self.btn_bld_c = QPushButton("叶片 C")
        for b in [self.btn_bld_a, self.btn_bld_b, self.btn_bld_c]:
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.setStyleSheet("""
                QPushButton {
                    background-color: #242834;
                    color: #8D99AE;
                    font-weight: bold;
                    font-size: 12px;
                    padding: 8px 4px;
                    border-radius: 4px;
                    border: 1px solid #363C4E;
                }
                QPushButton:checked {
                    background-color: #2A3C58;
                    color: #4CC9F0;
                    border: 1px solid #4CC9F0;
                }
            """)
        self.btn_bld_b.setChecked(True)  # 默认居中为 B

        self.blade_btn_group = QButtonGroup(self)
        self.blade_btn_group.addButton(self.btn_bld_a, 0)
        self.blade_btn_group.addButton(self.btn_bld_b, 1)
        self.blade_btn_group.addButton(self.btn_bld_c, 2)
        self.blade_btn_group.idClicked.connect(self._on_blade_selected)

        blade_layout.addWidget(self.btn_bld_a)
        blade_layout.addWidget(self.btn_bld_b)
        blade_layout.addWidget(self.btn_bld_c)
        layout.addWidget(blade_grp)

        # 3. 速度档位开关
        spd_grp = QGroupBox("调桨速度档位")
        spd_grp.setStyleSheet("QGroupBox { background-color: #1A1D26; border: 1px solid #2B303E; font-size: 11px; }")
        spd_layout = QHBoxLayout(spd_grp)
        spd_layout.setContentsMargins(10, 10, 10, 10)
        spd_layout.setSpacing(8)

        self.btn_spd_slow = QPushButton("🐢 慢速微调 (0.1°)")
        self.btn_spd_fast = QPushButton("🐇 快速调桨 (1.0°)")
        for b in [self.btn_spd_slow, self.btn_spd_fast]:
            b.setCheckable(True)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.setStyleSheet("""
                QPushButton {
                    background-color: #242834;
                    color: #8D99AE;
                    font-weight: bold;
                    font-size: 11px;
                    padding: 6px 4px;
                    border-radius: 4px;
                    border: 1px solid #363C4E;
                }
                QPushButton:checked {
                    background-color: #353A2A;
                    color: #FFB703;
                    border: 1px solid #FFB703;
                }
            """)
        self.btn_spd_slow.setChecked(True)

        self.spd_btn_group = QButtonGroup(self)
        self.spd_btn_group.addButton(self.btn_spd_slow, 0)
        self.spd_btn_group.addButton(self.btn_spd_fast, 1)
        self.spd_btn_group.idClicked.connect(self._on_speed_changed)

        spd_layout.addWidget(self.btn_spd_slow)
        spd_layout.addWidget(self.btn_spd_fast)
        layout.addWidget(spd_grp)

        # 4. 点动变桨操作按键 (Fine 开桨 / Feather 顺桨)
        jog_grp = QGroupBox("变桨点动控制 (按住点动 · 松开停止)")
        jog_grp.setStyleSheet("QGroupBox { background-color: #1A1D26; border: 1px solid #2B303E; font-size: 11px; }")
        jog_layout = QVBoxLayout(jog_grp)
        jog_layout.setContentsMargins(10, 12, 10, 12)
        jog_layout.setSpacing(10)

        # 顺桨按键 Feather (向上/增大角度向 90°)
        self.btn_feather = QPushButton("🔼 顺桨点动 [Feather] (增大桨距角)")
        self.btn_feather.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_feather.setStyleSheet("""
            QPushButton {
                background-color: #4A3E1B;
                color: #FFE6A7;
                font-size: 13px;
                font-weight: bold;
                padding: 12px;
                border-radius: 6px;
                border: 2px solid #997B2A;
            }
            QPushButton:hover {
                background-color: #5E4E20;
            }
            QPushButton:pressed {
                background-color: #DDA15E;
                color: #1B1811;
                border: 2px solid #FFD166;
            }
        """)
        self.btn_feather.pressed.connect(lambda: self.featherJogChanged.emit(True))
        self.btn_feather.released.connect(lambda: self.featherJogChanged.emit(False))
        jog_layout.addWidget(self.btn_feather)

        # 开桨按键 Fine (向下/减小角度向 0°)
        self.btn_fine = QPushButton("🔽 开桨点动 [Fine] (迎风功率运行)")
        self.btn_fine.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_fine.setStyleSheet("""
            QPushButton {
                background-color: #1B4332;
                color: #D8F3DC;
                font-size: 13px;
                font-weight: bold;
                padding: 12px;
                border-radius: 6px;
                border: 2px solid #2D6A4F;
            }
            QPushButton:hover {
                background-color: #235841;
            }
            QPushButton:pressed {
                background-color: #52B788;
                color: #081C15;
                border: 2px solid #74C69D;
            }
        """)
        self.btn_fine.pressed.connect(lambda: self.fineJogChanged.emit(True))
        self.btn_fine.released.connect(lambda: self.fineJogChanged.emit(False))
        jog_layout.addWidget(self.btn_fine)

        layout.addWidget(jog_grp)

        # 5. 变桨机械锁插拔控制
        lock_grp = QGroupBox("变桨机械锁控制")
        lock_grp.setStyleSheet("QGroupBox { background-color: #1A1D26; border: 1px solid #2B303E; font-size: 11px; }")
        lock_layout = QHBoxLayout(lock_grp)
        lock_layout.setContentsMargins(10, 10, 10, 10)
        lock_layout.setSpacing(8)

        self.btn_lock_disengage = QPushButton("🔓 拔销解锁\n(Disengage)")
        self.btn_lock_disengage.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_disengage.setStyleSheet("""
            QPushButton {
                background-color: #242834;
                color: #EDF2F4;
                font-weight: bold;
                font-size: 11px;
                padding: 8px 4px;
                border-radius: 4px;
                border: 1px solid #363C4E;
            }
            QPushButton:pressed {
                background-color: #06D6A0;
                color: #000;
            }
        """)
        self.btn_lock_disengage.pressed.connect(lambda: self.lockDisengageTriggered.emit(True))
        self.btn_lock_disengage.released.connect(lambda: self.lockDisengageTriggered.emit(False))
        lock_layout.addWidget(self.btn_lock_disengage)

        self.btn_lock_engage = QPushButton("🔒 插销锁紧\n(Engage)")
        self.btn_lock_engage.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_engage.setStyleSheet("""
            QPushButton {
                background-color: #242834;
                color: #EDF2F4;
                font-weight: bold;
                font-size: 11px;
                padding: 8px 4px;
                border-radius: 4px;
                border: 1px solid #363C4E;
            }
            QPushButton:pressed {
                background-color: #EF476F;
                color: #FFF;
            }
        """)
        self.btn_lock_engage.pressed.connect(lambda: self.lockEngageTriggered.emit(True))
        self.btn_lock_engage.released.connect(lambda: self.lockEngageTriggered.emit(False))
        lock_layout.addWidget(self.btn_lock_engage)

        layout.addWidget(lock_grp)

        # 6. 标定与复位操作
        ops_layout = QHBoxLayout()
        ops_layout.setSpacing(8)

        self.btn_calib = QPushButton("🎯 0° 基准标定")
        self.btn_calib.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_calib.setStyleSheet("""
            QPushButton {
                background-color: #232B3E;
                color: #64DFDF;
                font-weight: bold;
                font-size: 11px;
                padding: 8px 6px;
                border-radius: 4px;
                border: 1px solid #3A4E70;
            }
            QPushButton:hover {
                background-color: #2E3852;
            }
        """)
        self.btn_calib.clicked.connect(self._on_calib_clicked)
        ops_layout.addWidget(self.btn_calib)

        self.btn_clear = QPushButton("🔴 故障清除 (Clear)")
        self.btn_clear.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_clear.setStyleSheet("""
            QPushButton {
                background-color: #3E1E24;
                color: #FF85A1;
                font-weight: bold;
                font-size: 11px;
                padding: 8px 6px;
                border-radius: 4px;
                border: 1px solid #703A43;
            }
            QPushButton:hover {
                background-color: #52272F;
            }
        """)
        self.btn_clear.clicked.connect(lambda: self.clearAlarmTriggered.emit())
        ops_layout.addWidget(self.btn_clear)

        layout.addLayout(ops_layout)
        layout.addStretch()

        return grp

    # =========================================================================
    # 右侧: 状态监测、三轴安全矩阵与动态示波器
    # =========================================================================
    def _create_telemetry_panel(self) -> QWidget:
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(10)

        # 1. 三轴安全互锁状态矩阵 (Safety Matrix)
        safe_grp = QGroupBox("三轴安全互锁矩阵监控 (手操模式安全门限: 安全叶片数 >= 2)")
        safe_layout = QGridLayout(safe_grp)
        safe_layout.setContentsMargins(12, 12, 12, 12)
        safe_layout.setSpacing(10)

        # 卡片: 叶片 A / B / C
        self.card_safe_a = self._create_blade_safe_card("叶片 A (Blade A)")
        self.card_safe_b = self._create_blade_safe_card("叶片 B (Blade B)")
        self.card_safe_c = self._create_blade_safe_card("叶片 C (Blade C)")

        safe_layout.addWidget(self.card_safe_a['frame'], 0, 0)
        safe_layout.addWidget(self.card_safe_b['frame'], 0, 1)
        safe_layout.addWidget(self.card_safe_c['frame'], 0, 2)
        layout.addWidget(safe_grp)

        # 2. 当前受控叶片实时数字液晶仪表盘 (LCD Gauge Console)
        lcd_grp = QGroupBox("当前活动叶片实时遥测仪表盘")
        lcd_layout = QGridLayout(lcd_grp)
        lcd_layout.setContentsMargins(12, 12, 12, 12)
        lcd_layout.setSpacing(10)

        self.lcd_bld_title = self._create_stat_card("当前受控叶片", "Blade B", "#4CC9F0")
        self.lcd_disp = self._create_stat_card("双缸实时行程", "50.0 mm", "#06D6A0")
        self.lcd_deg = self._create_stat_card("换算桨距角", "5.5 °", "#FFD166")
        self.lcd_spool = self._create_stat_card("比例伺服阀开度", "0.0 % (0.0V)", "#EDF2F4")
        self.lcd_p107 = self._create_stat_card("107 母管蓄能器压力", "260.0 bar", "#06D6A0")
        self.lcd_lock_pin = self._create_stat_card("变桨锁销位移", "0.0 mm (解锁)", "#4CC9F0")

        lcd_layout.addWidget(self.lcd_bld_title, 0, 0)
        lcd_layout.addWidget(self.lcd_disp, 0, 1)
        lcd_layout.addWidget(self.lcd_deg, 0, 2)
        lcd_layout.addWidget(self.lcd_spool, 1, 0)
        lcd_layout.addWidget(self.lcd_p107, 1, 1)
        lcd_layout.addWidget(self.lcd_lock_pin, 1, 2)
        layout.addWidget(lcd_grp)

        # 3. 点动动态示波器
        scope_grp = QGroupBox("机旁手操点动动态响应波形 (行程位移与比例阀开度)")
        scope_layout = QVBoxLayout(scope_grp)
        scope_layout.setContentsMargins(10, 10, 10, 10)

        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('#13141C')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self.plot_widget.setLabel('bottom', '时间 / 采样点', color='#8D99AE')
        self.plot_widget.setLabel('left', '位移行程 (mm)', color='#06D6A0')
        self.plot_widget.setMinimumHeight(180)

        # 位移曲线 (绿色)
        self.curve_disp = self.plot_widget.plot(pen=pg.mkPen(color='#06D6A0', width=2), name="行程 (mm)")

        scope_layout.addWidget(self.plot_widget)
        layout.addWidget(scope_grp)

        return panel

    def _create_blade_safe_card(self, title: str) -> dict:
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #1B1E28;
                border: 1px solid #2E3344;
                border-radius: 6px;
                padding: 6px;
            }
        """)
        f_layout = QVBoxLayout(frame)
        f_layout.setContentsMargins(8, 6, 8, 6)
        f_layout.setSpacing(4)

        t_lbl = QLabel(title)
        t_lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #EDF2F4;")
        f_layout.addWidget(t_lbl)

        status_lbl = QLabel("● 安全位置")
        status_lbl.setStyleSheet("font-size: 12px; font-weight: bold; color: #06D6A0;")
        f_layout.addWidget(status_lbl)

        detail_lbl = QLabel("行程: 50.0mm | 锁: 解锁")
        detail_lbl.setStyleSheet("font-size: 10px; color: #8D99AE;")
        f_layout.addWidget(detail_lbl)

        return {'frame': frame, 'status_lbl': status_lbl, 'detail_lbl': detail_lbl}

    def _create_stat_card(self, title: str, init_val: str, color_hex: str) -> QFrame:
        frame = QFrame()
        frame.setStyleSheet("""
            QFrame {
                background-color: #1B1E28;
                border: 1px solid #2B3040;
                border-radius: 6px;
            }
        """)
        f_layout = QVBoxLayout(frame)
        f_layout.setContentsMargins(10, 6, 10, 6)
        f_layout.setSpacing(2)

        t_lbl = QLabel(title)
        t_lbl.setStyleSheet("font-size: 10px; color: #8D99AE;")
        f_layout.addWidget(t_lbl)

        v_lbl = QLabel(init_val)
        v_lbl.setObjectName("val")
        v_lbl.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {color_hex}; font-family: 'Consolas', 'Segoe UI', monospace;")
        f_layout.addWidget(v_lbl)

        return frame

    # =========================================================================
    # 事件处理与信号发射
    # =========================================================================
    def _on_plug_toggled(self, checked: bool):
        if checked:
            self.btn_plug.setText("🔌 拔出机旁手操盒航插")
            self.led_plug.setStyleSheet("color: #06D6A0; font-size: 18px;")
            self.lbl_banner_plug.setText("🔌 航插: 已插接就绪")
            self.lbl_banner_plug.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.btn_plug.setText("🔌 插接机旁手操盒航插")
            self.led_plug.setStyleSheet("color: #555A6E; font-size: 18px;")
            self.lbl_banner_plug.setText("🔌 航插: 未接入")
            self.lbl_banner_plug.setStyleSheet("font-size: 11px; font-weight: bold; color: #8D99AE; padding: 4px 10px; background-color: #2B2E3B; border-radius: 4px;")
        self.pendantConnectedChanged.emit(checked)

    def _on_blade_selected(self, id: int):
        b_name = ['A', 'B', 'C'][id]
        self.bladeChosenChanged.emit(b_name)

    def _on_speed_changed(self, id: int):
        # 0: Slow, 1: Fast
        self.speedModeChanged.emit(bool(id == 1))

    def _on_calib_clicked(self):
        ret = QMessageBox.question(
            self,
            "0° 基准零位标定确认",
            "是否确认将当前选中叶片的当前物理机械位置标定为 0° 零位物理基准？\n\n注意: 要求叶片当前处于小角度 (<10°) 且处于就地手操安全控制状态下。",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No
        )
        if ret == QMessageBox.StandardButton.Yes:
            self.pos0CalibrateTriggered.emit()

    # =========================================================================
    # 周期遥测刷新 (由主界面 50 FPS tick 调用)
    # =========================================================================
    def update_telemetry(self, sbt: ServiceBoxTelemetry, telems: dict, time_hist: np.ndarray):
        """
        :param sbt: 最新手操盒系统状态 (ServiceBoxTelemetry)
        :param telems: 各叶片物理遥测数据字典 {'A': t_a, 'B': t_b, 'C': t_c}
        :param time_hist: 全局时间缓存
        """
        # 1. 顶部横幅更新
        if sbt.in_control:
            self.lbl_banner_auth.setText("🕹️ 权限: 手操就地排他主控")
            self.lbl_banner_auth.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_banner_auth.setText("🌐 权限: 远程系统主控")
            self.lbl_banner_auth.setStyleSheet("font-size: 11px; font-weight: bold; color: #8D99AE; padding: 4px 10px; background-color: #2B2E3B; border-radius: 4px;")

        # 安全叶片指示
        n_safe = sbt.n_safe_blades
        self.lbl_banner_safe.setText(f"🛡️ 安全叶片: {n_safe}/3")
        if n_safe >= 2:
            self.lbl_banner_safe.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_banner_safe.setStyleSheet("font-size: 11px; font-weight: bold; color: #EF476F; padding: 4px 10px; background-color: #381A22; border-radius: 4px; border: 1px solid #6E2234;")

        # 综合报警
        if sbt.alarm_status:
            self.lbl_banner_alarm.setText("🔴 状态: 联锁保护告警")
            self.lbl_banner_alarm.setStyleSheet("font-size: 11px; font-weight: bold; color: #EF476F; padding: 4px 10px; background-color: #381A22; border-radius: 4px; border: 1px solid #6E2234;")
        else:
            self.lbl_banner_alarm.setText("🟢 状态: 正常")
            self.lbl_banner_alarm.setStyleSheet("font-size: 11px; font-weight: bold; color: #06D6A0; padding: 4px 10px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")

        # 2. 三轴安全矩阵卡片更新
        cards = {'A': self.card_safe_a, 'B': self.card_safe_b, 'C': self.card_safe_c}
        for b_id, card in cards.items():
            is_safe = sbt.blade_safe.get(b_id, False)
            t_b = telems.get(b_id)
            if t_b:
                lock_str = "锁紧" if t_b.is_locked else "解锁"
                card['detail_lbl'].setText(f"行程: {t_b.displacement_mm:5.1f}mm | 锁: {lock_str}")
            if is_safe:
                card['status_lbl'].setText("● 处于安全位置")
                card['status_lbl'].setStyleSheet("font-size: 12px; font-weight: bold; color: #06D6A0;")
            else:
                card['status_lbl'].setText("● 未处于安全位")
                card['status_lbl'].setStyleSheet("font-size: 12px; font-weight: bold; color: #FFB703;")

        # 3. 当前受控叶片仪表盘更新
        act_b = sbt.active_blade
        t_act = telems.get(act_b)
        if t_act:
            self.lcd_bld_title.findChild(QLabel, "val").setText(f"叶片 {act_b}")
            self.lcd_disp.findChild(QLabel, "val").setText(f"{t_act.displacement_mm:5.1f} mm")
            
            # 计算近似几何桨距角 (50mm 对应 ~0°, 820mm 对应 ~90°)
            ang_deg = (t_act.displacement_mm - 50.0) / (820.0 - 50.0) * 90.0
            self.lcd_deg.findChild(QLabel, "val").setText(f"{ang_deg:5.1f} °")

            v_pct = t_act.valve_position_pct
            self.lcd_spool.findChild(QLabel, "val").setText(f"{v_pct:+5.1f} % ({v_pct/10.0:+4.1f}V)")
            self.lcd_p107.findChild(QLabel, "val").setText(f"{t_act.p_acc_bar:5.1f} bar")

            lock_state_str = "已插销闭锁" if t_act.is_locked else "已拔销解锁"
            self.lcd_lock_pin.findChild(QLabel, "val").setText(f"{t_act.lock_pin_mm:4.1f} mm ({lock_state_str})")

            # 4. 滚动示波器更新
            self.disp_hist = np.roll(self.disp_hist, -1)
            self.disp_hist[-1] = t_act.displacement_mm
            self.curve_disp.setData(self.disp_hist)
