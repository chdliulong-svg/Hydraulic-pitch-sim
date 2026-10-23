"""
兆瓦级风机液压变桨仿真系统 - 左侧核心控制与实时遥测面板
包含叶片选择、运动控制 (PID/开环)、机械锁控制、应急顺桨与遥测数值网格
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QGroupBox, QPushButton, QSlider, QDoubleSpinBox,
    QLabel, QCheckBox
)
from PyQt6.QtCore import Qt, pyqtSignal


class LeftControlPanel(QWidget):
    """左侧控制与遥测面板"""

    # 外部交互信号
    bladeSelected = pyqtSignal(str)
    faceSelected = bladeSelected  # 兼容旧版信号别名
    syncChanged = pyqtSignal(bool)
    autoModeChanged = pyqtSignal(bool)
    targetChanged = pyqtSignal(float)
    voltageChanged = pyqtSignal(float)
    unlockTriggered = pyqtSignal()
    lockTriggered = pyqtSignal()
    featherToggled = pyqtSignal()
    motorToggled = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("background-color: transparent;")

        self.active_blade = 'A'
        self.active_face = self.active_blade  # 兼容旧属性
        self.is_sync = False
        self.is_auto = True

        self._init_ui()

    def _init_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 0, 8, 8)
        layout.setSpacing(8)

        # 1. 叶片选择
        blade_box = QGroupBox(" 叶片选择 ")
        blade_layout = QHBoxLayout(blade_box)
        blade_layout.setContentsMargins(8, 6, 8, 6)
        blade_layout.setSpacing(6)

        self.btn_blades = {}
        for k, text in [('A', '叶片 A'), ('B', '叶片 B'), ('C', '叶片 C')]:
            b = QPushButton(text)
            b.setCheckable(True)
            b.setFixedHeight(28)
            b.setCursor(Qt.CursorShape.PointingHandCursor)
            b.clicked.connect(lambda _, key=k: self.bladeSelected.emit(key))
            self.btn_blades[k] = b
            blade_layout.addWidget(b)

        self.btn_faces = self.btn_blades  # 兼容旧属性别名

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
        self.chk_sync_all.stateChanged.connect(lambda state: self.syncChanged.emit(bool(state)))
        blade_layout.addWidget(self.chk_sync_all)
        layout.addWidget(blade_box)

        # 2. 运动控制
        self.motion_box = QGroupBox(" 运动控制 ")
        motion_layout = QVBoxLayout(self.motion_box)
        motion_layout.setContentsMargins(8, 8, 8, 8)
        motion_layout.setSpacing(6)

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

        self.btn_mode_auto.clicked.connect(lambda: self.autoModeChanged.emit(True))
        self.btn_mode_manual.clicked.connect(lambda: self.autoModeChanged.emit(False))
        mode_btn_row.addWidget(self.btn_mode_auto)
        mode_btn_row.addWidget(self.btn_mode_manual)
        motion_layout.addLayout(mode_btn_row)

        # 自动模式子面板
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
        self.spin_target.valueChanged.connect(lambda v: self.targetChanged.emit(float(v)))

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

        # 手动模式子面板
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
        self.spin_voltage.valueChanged.connect(lambda v: self.voltageChanged.emit(float(v)))

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

        self.widget_manual_ctrl.setVisible(False)
        layout.addWidget(self.motion_box)

        # 3. 变桨机械锁
        self.lock_box = QGroupBox(" 变桨机械锁 ")
        lock_layout = QVBoxLayout(self.lock_box)
        lock_layout.setContentsMargins(8, 8, 8, 8)
        lock_layout.setSpacing(6)

        lock_btn_row = QHBoxLayout()
        lock_btn_row.setSpacing(8)
        self.btn_lock_release = QPushButton("🔓 拔销解锁")
        self.btn_lock_release.setFixedHeight(30)
        self.btn_lock_release.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_release.clicked.connect(self.unlockTriggered.emit)

        self.btn_lock_engage = QPushButton("🔒 插销锁定")
        self.btn_lock_engage.setFixedHeight(30)
        self.btn_lock_engage.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_lock_engage.clicked.connect(self.lockTriggered.emit)

        lock_btn_row.addWidget(self.btn_lock_release)
        lock_btn_row.addWidget(self.btn_lock_engage)
        lock_layout.addLayout(lock_btn_row)

        self.lbl_lock_detail = QLabel("状态: 🟢 已解锁 | 90°对齐: 否")
        self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        lock_layout.addWidget(self.lbl_lock_detail)
        layout.addWidget(self.lock_box)

        # 4. 应急顺桨
        self.safe_box = QGroupBox(" 应急顺桨 ")
        safe_layout = QVBoxLayout(self.safe_box)
        safe_layout.setContentsMargins(8, 8, 8, 8)
        safe_layout.setSpacing(6)

        self.btn_feather = QPushButton("🛑 模拟紧急顺桨")
        self.btn_feather.setFixedHeight(32)
        self.btn_feather.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_feather.clicked.connect(self.featherToggled.emit)
        safe_layout.addWidget(self.btn_feather)

        self.lbl_safe_status = QLabel("状态: 正常运行")
        self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        safe_layout.addWidget(self.lbl_safe_status)
        layout.addWidget(self.safe_box)

        # 5. 液压站动力源 (HPU)
        self.hpu_box = QGroupBox(" 液压站动力源 (HPU) ")
        hpu_layout = QVBoxLayout(self.hpu_box)
        hpu_layout.setContentsMargins(8, 8, 8, 8)
        hpu_layout.setSpacing(6)

        self.btn_motor = QPushButton("🟢 油泵电机: 运行中 (点击停机)")
        self.btn_motor.setFixedHeight(32)
        self.btn_motor.setCursor(Qt.CursorShape.PointingHandCursor)
        self.btn_motor.clicked.connect(self.motorToggled.emit)
        hpu_layout.addWidget(self.btn_motor)

        self.lbl_motor_status = QLabel("电机转速: 1450 rpm | 3泵额定: 64.5 L/min")
        self.lbl_motor_status.setStyleSheet("color: #06D6A0; font-family: Consolas; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        hpu_layout.addWidget(self.lbl_motor_status)
        layout.addWidget(self.hpu_box)

        # 6. 实时遥测
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
            ("rpm",  "电机转速:", "1450 rpm", "#06D6A0"),
            ("q_hpu","泵站流量:", "0.0 L/min", "#4CC9F0"),
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

        self.update_blade_buttons_style('A', False)
        self.update_mode_styles(True)
        self.update_feather_button_style(False)
        self.update_lock_buttons_style(False)
        self.update_motor_button_style(True, 1450.0)

    def update_blade_buttons_style(self, active_blade: str, is_sync: bool):
        self.active_blade = active_blade
        self.active_face = active_blade  # 兼容旧属性
        self.is_sync = is_sync
        self.chk_sync_all.setChecked(is_sync)
        for k, btn in self.btn_blades.items():
            if is_sync:
                btn.setChecked(True)
                btn.setStyleSheet("background-color: #3A405A; color: #FFD166; font-weight: bold; border: 1px solid #FFD166; border-radius: 4px; font-size: 11px;")
            else:
                is_cur = (k == active_blade)
                btn.setChecked(is_cur)
                if is_cur:
                    btn.setStyleSheet("background-color: #4CC9F0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 11px;")
                else:
                    btn.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 11px;")

    # 兼容旧方法名
    def update_face_buttons_style(self, active_face: str, is_sync: bool):
        self.update_blade_buttons_style(active_face, is_sync)

    def update_mode_styles(self, is_auto: bool):
        self.is_auto = is_auto
        self.btn_mode_auto.setChecked(is_auto)
        self.btn_mode_manual.setChecked(not is_auto)
        if is_auto:
            self.btn_mode_auto.setStyleSheet("background-color: #4CC9F0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 12px;")
            self.btn_mode_manual.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 12px;")
            self.widget_auto_ctrl.setVisible(True)
            self.widget_manual_ctrl.setVisible(False)
        else:
            self.btn_mode_auto.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 12px;")
            self.btn_mode_manual.setStyleSheet("background-color: #FFD166; color: #121318; font-weight: bold; border-radius: 4px; font-size: 12px;")
            self.widget_auto_ctrl.setVisible(False)
            self.widget_manual_ctrl.setVisible(True)

    def update_feather_button_style(self, is_feathering: bool):
        if is_feathering:
            self.btn_feather.setText("🛑 正在顺桨 (点击恢复正常)")
            self.btn_feather.setStyleSheet("background-color: #D90429; color: #FFFFFF; font-weight: bold; font-size: 13px; border: 2px solid #EF233C; border-radius: 6px;")
            self.lbl_safe_status.setText("状态: ⚠️ 紧急顺桨激活中")
            self.lbl_safe_status.setStyleSheet("color: #EF233C; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")
        else:
            self.btn_feather.setText("🔴 模拟紧急顺桨")
            self.btn_feather.setStyleSheet("background-color: #5C151B; color: #EDF2F4; font-weight: bold; font-size: 13px; border: 1px solid #781F27; border-radius: 6px;")
            self.lbl_safe_status.setText("状态: 正常运行")
            self.lbl_safe_status.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

    def update_lock_buttons_style(self, is_locked: bool):
        if is_locked:
            self.btn_lock_release.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 12px;")
            self.btn_lock_engage.setStyleSheet("background-color: #FFB703; color: #121318; font-weight: bold; border-radius: 4px; font-size: 12px;")
        else:
            self.btn_lock_release.setStyleSheet("background-color: #4CC9F0; color: #121318; font-weight: bold; border-radius: 4px; font-size: 12px;")
            self.btn_lock_engage.setStyleSheet("background-color: #2C303E; color: #8D99AE; border: 1px solid #44495C; border-radius: 4px; font-size: 12px;")

    def update_motor_button_style(self, is_running: bool, rpm: float = 1450.0):
        """刷新油泵电机启停按钮与状态标签外观"""
        if is_running:
            self.btn_motor.setText("🟢 油泵电机: 运行中 (点击停机)")
            self.btn_motor.setStyleSheet("background-color: #162E25; color: #06D6A0; font-weight: bold; border: 1px solid #23533E; border-radius: 4px; font-size: 12px;")
            self.lbl_motor_status.setText(f"电机转速: {rpm:4.0f} rpm | 3泵额定: 64.5 L/min")
            self.lbl_motor_status.setStyleSheet("color: #06D6A0; font-family: Consolas; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")
        else:
            self.btn_motor.setText("🔴 油泵电机: 已停机 (点击启动)")
            self.btn_motor.setStyleSheet("background-color: #381519; color: #EF233C; font-weight: bold; border: 1px solid #5C2229; border-radius: 4px; font-size: 12px;")
            self.lbl_motor_status.setText("电机转速: 0 rpm | 停机无压力供油")
            self.lbl_motor_status.setStyleSheet("color: #EF233C; font-family: Consolas; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #381519; border: 1px solid #5C2229; border-radius: 4px;")

    def update_telemetry(self, active_t, is_locked: bool, u_cmd: float = 0.0):
        """刷新遥测数据标签与机械锁状态"""
        self.tele_labels["disp"].setText(f"{active_t.displacement_mm:5.1f} mm")
        self.tele_labels["vel"].setText(f"{active_t.velocity_mps * 1000.0:+5.1f} mm/s")
        self.tele_labels["u_v"].setText(f"{u_cmd:+5.2f} V")
        self.tele_labels["spool"].setText(f"{active_t.valve_position_pct:+5.1f} %")
        self.tele_labels["pA"].setText(f"{active_t.p_A_bar:5.1f} bar")
        self.tele_labels["pB"].setText(f"{active_t.p_B_bar:5.1f} bar")
        self.tele_labels["pacc"].setText(f"{active_t.p_acc_bar:5.1f} bar")
        self.tele_labels["pPP"].setText(f"{active_t.p_PP_bar:5.1f} bar")
        self.tele_labels["v25"].setText("打压" if active_t.valve_25_state else "卸荷")
        self.tele_labels["v25"].setStyleSheet("color: #06D6A0; font-family: Consolas; font-size: 12px; font-weight: bold;" if active_t.valve_25_state else "color: #8D99AE; font-family: Consolas; font-size: 12px; font-weight: bold;")
        self.tele_labels["pP"].setText(f"{active_t.p_supply_bar:5.1f} bar")
        self.tele_labels["rpm"].setText(f"{getattr(active_t, 'motor_rpm', 1450.0):4.0f} rpm")
        self.tele_labels["q_hpu"].setText(f"{getattr(active_t, 'q_hpu_lpm', 0.0):4.1f} L/min")

        # 同步电机按键显示
        self.update_motor_button_style(getattr(active_t, 'motor_running', True), getattr(active_t, 'motor_rpm', 1450.0))

        align_txt = "是" if active_t.switch_129 else "否"
        if is_locked:
            self.lbl_lock_detail.setText(f"状态: 🔒 已锁定 ({active_t.lock_pin_mm:0.1f}mm) | 90°对齐: {align_txt}")
            self.lbl_lock_detail.setStyleSheet("color: #FFB703; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #2B2317; border: 1px solid #5A4015; border-radius: 4px;")
        else:
            lock_str = "🟢 已拔销解锁" if active_t.switch_S_unlocked else f"⏳ 动作中 ({active_t.lock_pin_mm:4.1f}mm)"
            self.lbl_lock_detail.setText(f"状态: {lock_str} | 90°对齐: {align_txt}")
            self.lbl_lock_detail.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px 6px; background-color: #172B23; border: 1px solid #23533E; border-radius: 4px;")

    def set_remote_authority(self, is_remote: bool):
        """远程控制时禁用本地运动控制与机械锁手动操作，移交 PLC 接管"""
        enabled = not is_remote
        self.motion_box.setEnabled(enabled)
        self.lock_box.setEnabled(enabled)
        self.safe_box.setEnabled(enabled)
