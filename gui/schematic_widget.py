"""
兆瓦级风机液压变桨仿真系统 - 机械/液压拓扑动态渲染画布
包含双活塞主油缸、铰支机构、40/20X60变桨机械锁及底部阀组卡片
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QLinearGradient
)
from PyQt6.QtCore import Qt


class HydraulicSchematicWidget(QWidget):
    """自定义高精度动态机械/液压拓扑渲染画布"""

    def __init__(self, blade_id="A", parent=None, **kwargs):
        super().__init__(parent)
        self.blade_id = kwargs.get("face_name", blade_id)
        self.face_name = self.blade_id  # 兼容旧属性
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
        self.motor_running = True       # 电机运行状态 (默认开启)
        self.motor_rpm = 1450.0

    def update_telemetry(self, stroke_mm, p_A, p_B, p_acc, valve_pos_pct, is_feathering,
                         lock_pin_mm, is_locked, switch_129, switch_S_unlocked, p_PP_bar,
                         valve_25_state=False, p_supply_bar=2.5, q_hpu_lpm=0.0,
                         motor_running=True, motor_rpm=1450.0):
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
        self.motor_running = motor_running
        self.motor_rpm = motor_rpm
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

        # 1. 顶部状态横幅 (支持安全顺桨 & 机械锁紧综合报警，带半透明底色卡片)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(QColor("#1A1C24")))
        painter.drawRoundedRect(12, 6, max(600, w - 24), 26, 4, 4)

        painter.setFont(QFont("Segoe UI", 9, QFont.Weight.Bold))
        tag = f"【叶片 {self.blade_id}】"
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
        if not self.motor_running:
            hpu_border_col = QColor("#EF233C")
            hpu_mode_txt = "🔴 电机停机"
        elif self.valve_25_state:
            hpu_border_col = QColor("#06D6A0")
            hpu_mode_txt = "🟢 打压供油"
        else:
            hpu_border_col = QColor("#4A4E69")
            hpu_mode_txt = "⚪ 卸荷循环"

        painter.setPen(QPen(hpu_border_col, 1.8))
        painter.setBrush(QBrush(QColor("#20222C")))
        painter.drawRoundedRect(hpu_box_x, valve_box_y, hpu_box_w, 50, 4, 4)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        painter.setPen(hpu_border_col)
        painter.drawText(hpu_box_x + 10, valve_box_y + 20, f"动力源: {hpu_mode_txt}")

        painter.setFont(QFont("Consolas", 8))
        if not self.motor_running:
            painter.setPen(QColor("#EF233C"))
            painter.drawText(hpu_box_x + 10, valve_box_y + 38, f"电机: 0 rpm | 供: {self.p_supply_bar:4.1f} bar")
        else:
            painter.setPen(QColor("#4CC9F0"))
            painter.drawText(hpu_box_x + 10, valve_box_y + 38, f"供油: {self.p_supply_bar:4.1f} bar | {self.q_hpu_lpm:4.1f} L/min")
