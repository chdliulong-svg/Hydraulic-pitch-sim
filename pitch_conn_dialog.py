"""
上海电气风电集团 (SHANGHAI ELECTRIC) 兆瓦级液压变桨仿真系统
CODESYS OPC UA 通讯与连接设置对话框 (Connection Settings Dialog)
"""

import os
import sys
import json
import time
import asyncio
import threading
from typing import Optional, Callable, Dict, Any

from PyQt6.QtWidgets import (
    QDialog, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox,
    QFrame, QSizePolicy, QMessageBox
)
from PyQt6.QtCore import Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont, QColor

CONFIG_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "pitch_conn_config.json")

DEFAULT_CONFIG: Dict[str, Any] = {
    "endpoint": "opc.tcp://127.0.0.1:4840",
    "prefix": "ns=4;s=|var|CODESYS Control Win V3.PAC.PRG_pitch_sim.",
    "cycle_time_ms": 5.0,
    "timeout_ms": 2000,
    "retry_interval_s": 1.0,
    "auth_mode": "Anonymous",
    "username": "",
    "password": "",
    "raw_policy": "AutoFilter",
}


def load_connection_config() -> Dict[str, Any]:
    """读取已保存的连接配置文件，若不存在则返回默认值"""
    config = dict(DEFAULT_CONFIG)
    if os.path.exists(CONFIG_FILE_PATH):
        try:
            with open(CONFIG_FILE_PATH, "r", encoding="utf-8") as f:
                saved = json.load(f)
                config.update(saved)
        except Exception as e:
            print(f"[WARN] 加载连接配置失败: {e}，使用默认配置。")
    return config


def save_connection_config(config: Dict[str, Any]) -> bool:
    """持久化保存连接配置到本地 JSON"""
    try:
        with open(CONFIG_FILE_PATH, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2, ensure_ascii=False)
        return True
    except Exception as e:
        print(f"[ERROR] 保存连接配置失败: {e}")
        return False


class TestWorker(QObject):
    """异步 OPC UA 连通性测试工作器"""
    finished = pyqtSignal(bool, str, float, int)

    def __init__(self, endpoint: str, prefix: str):
        super().__init__()
        self.endpoint = endpoint
        self.prefix = prefix

    def run_test(self):
        t = threading.Thread(target=self._async_runner, daemon=True)
        t.start()

    def _async_runner(self):
        try:
            from asyncua import Client
        except ImportError:
            self.finished.emit(False, "缺少 asyncua 依赖库", 0.0, 0)
            return

        async def _test():
            t0 = time.perf_counter()
            try:
                async with Client(url=self.endpoint, timeout=2.0) as client:
                    # 尝试探测父节点
                    parent_node_id = self.prefix.rstrip(".")
                    parent_node = client.get_node(parent_node_id)
                    children = await parent_node.get_children()
                    dt_ms = (time.perf_counter() - t0) * 1000.0
                    return True, "连接成功", dt_ms, len(children)
            except Exception as e:
                dt_ms = (time.perf_counter() - t0) * 1000.0
                return False, str(e), dt_ms, 0

        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            ok, msg, dt, count = loop.run_until_complete(_test())
            self.finished.emit(ok, msg, dt, count)
        finally:
            loop.close()


class ConnectionSettingsDialog(QDialog):
    """
    OPC UA 通讯参数与连接诊断设置对话框
    采用与仿真器统一的暗黑工业风格 (Dark Slate / Teal Accent)
    """

    def __init__(self, parent=None, current_config: Optional[Dict[str, Any]] = None, on_apply: Optional[Callable[[Dict[str, Any]], None]] = None):
        super().__init__(parent)
        self.setWindowTitle("OPC UA 通讯与连接配置")
        self.resize(580, 540)
        self.setMinimumSize(540, 500)
        self.setStyleSheet("""
            QDialog {
                background-color: #151821;
                color: #EDF2F4;
                font-family: 'Segoe UI', 'Microsoft YaHei', sans-serif;
            }
            QGroupBox {
                background-color: #1B1E2B;
                border: 1px solid #2B3145;
                border-radius: 6px;
                margin-top: 10px;
                padding-top: 14px;
                font-size: 12px;
                font-weight: bold;
                color: #4CC9F0;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 6px;
                left: 12px;
                background-color: #151821;
            }
            QLabel {
                color: #C0CAD6;
                font-size: 11px;
            }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
                background-color: #242938;
                color: #EDF2F4;
                border: 1px solid #363E56;
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 11px;
                font-family: Consolas, monospace;
            }
            QLineEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus {
                border: 1px solid #4CC9F0;
                background-color: #2A3042;
            }
            QComboBox::drop-down {
                border: none;
                width: 20px;
            }
            QPushButton {
                background-color: #272C3E;
                color: #EDF2F4;
                border: 1px solid #3E4663;
                border-radius: 4px;
                padding: 5px 14px;
                font-size: 11px;
                font-weight: 500;
            }
            QPushButton:hover {
                background-color: #323950;
                border-color: #4CC9F0;
            }
            QPushButton:pressed {
                background-color: #1E2230;
            }
        """)

        self.config = dict(current_config if current_config else load_connection_config())
        self.on_apply = on_apply
        self._test_worker: Optional[TestWorker] = None

        self._init_ui()
        self._populate_data()

    def _init_ui(self):
        main_vbox = QVBoxLayout(self)
        main_vbox.setContentsMargins(18, 16, 18, 16)
        main_vbox.setSpacing(12)

        # 1. 顶部 Header
        header_layout = QHBoxLayout()
        header_icon = QLabel("⚙")
        header_icon.setStyleSheet("font-size: 22px; color: #4CC9F0;")
        header_layout.addWidget(header_icon)

        title_vbox = QVBoxLayout()
        title_lbl = QLabel("CODESYS OPC UA 远程连接配置")
        title_lbl.setStyleSheet("font-size: 14px; font-weight: bold; color: #EDF2F4;")
        subtitle_lbl = QLabel("配置数字孪生仿真机与 CODESYS 虚拟/物理控制器的在环通讯端点与时钟参数")
        subtitle_lbl.setStyleSheet("font-size: 10px; color: #7E8B9E;")
        title_vbox.addWidget(title_lbl)
        title_vbox.addWidget(subtitle_lbl)
        header_layout.addLayout(title_vbox)
        header_layout.addStretch()
        main_vbox.addLayout(header_layout)

        # 分割线
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        line.setStyleSheet("color: #272C3D;")
        main_vbox.addWidget(line)

        # 2. 通讯端点分组 (Endpoint Configuration)
        grp_endpoint = QWidget()
        grp_endpoint.setStyleSheet("background-color: #1B1E2B; border: 1px solid #2B3145; border-radius: 6px; padding: 10px;")
        vbox_ep = QVBoxLayout(grp_endpoint)
        vbox_ep.setContentsMargins(8, 8, 8, 8)
        vbox_ep.setSpacing(8)

        lbl_ep_title = QLabel("🌐 通讯服务器端点 (Server Endpoint)")
        lbl_ep_title.setStyleSheet("color: #4CC9F0; font-weight: bold; font-size: 12px;")
        vbox_ep.addWidget(lbl_ep_title)

        grid_ep = QGridLayout()
        grid_ep.setHorizontalSpacing(10)
        grid_ep.setVerticalSpacing(8)

        grid_ep.addWidget(QLabel("服务地址 (URL):"), 0, 0)
        self.edit_endpoint = QLineEdit()
        self.edit_endpoint.setPlaceholderText("opc.tcp://127.0.0.1:4840")
        grid_ep.addWidget(self.edit_endpoint, 0, 1)

        # 预设端点快捷胶囊按钮
        preset_layout = QHBoxLayout()
        preset_layout.setSpacing(8)
        btn_pre_local = QPushButton("本地 127.0.0.1")
        btn_pre_local.setStyleSheet("padding: 3px 8px; font-size: 11px; background-color: #212636; border: 1px solid #363E56; border-radius: 3px;")
        btn_pre_local.clicked.connect(lambda: self.edit_endpoint.setText("opc.tcp://127.0.0.1:4840"))

        btn_pre_lan = QPushButton("主控 192.168.1.100")
        btn_pre_lan.setStyleSheet("padding: 3px 8px; font-size: 11px; background-color: #212636; border: 1px solid #363E56; border-radius: 3px;")
        btn_pre_lan.clicked.connect(lambda: self.edit_endpoint.setText("opc.tcp://192.168.1.100:4840"))

        btn_pre_vm = QPushButton("工控机 192.168.137.10")
        btn_pre_vm.setStyleSheet("padding: 3px 8px; font-size: 11px; background-color: #212636; border: 1px solid #363E56; border-radius: 3px;")
        btn_pre_vm.clicked.connect(lambda: self.edit_endpoint.setText("opc.tcp://192.168.137.10:4840"))

        lbl_pre = QLabel("常用预设:")
        lbl_pre.setStyleSheet("color: #8D99AE; font-size: 11px;")
        preset_layout.addWidget(lbl_pre)
        preset_layout.addWidget(btn_pre_local)
        preset_layout.addWidget(btn_pre_lan)
        preset_layout.addWidget(btn_pre_vm)
        preset_layout.addStretch()
        grid_ep.addLayout(preset_layout, 1, 1)

        grid_ep.addWidget(QLabel("符号前缀 (Prefix):"), 2, 0)
        self.edit_prefix = QLineEdit()
        self.edit_prefix.setPlaceholderText("ns=4;s=|var|CODESYS Control Win V3.PAC.PRG_pitch_sim.")
        grid_ep.addWidget(self.edit_prefix, 2, 1)

        vbox_ep.addLayout(grid_ep)
        main_vbox.addWidget(grp_endpoint)

        # 3. 实时通讯时钟与策略 (Performance & Sync Timing)
        grp_timing = QWidget()
        grp_timing.setStyleSheet("background-color: #1B1E2B; border: 1px solid #2B3145; border-radius: 6px; padding: 10px;")
        vbox_tm = QVBoxLayout(grp_timing)
        vbox_tm.setContentsMargins(8, 8, 8, 8)
        vbox_tm.setSpacing(8)

        lbl_tm_title = QLabel("⏱ 实时同步时钟与通讯策略 (Clock & Policy)")
        lbl_tm_title.setStyleSheet("color: #4CC9F0; font-weight: bold; font-size: 12px;")
        vbox_tm.addWidget(lbl_tm_title)

        grid_tm = QGridLayout()
        grid_tm.setHorizontalSpacing(10)
        grid_tm.setVerticalSpacing(8)

        grid_tm.addWidget(QLabel("同步周期 (Cycle):"), 0, 0)
        self.combo_cycle = QComboBox()
        self.combo_cycle.addItem("2.0 ms (500Hz 极致微周期)", 2.0)
        self.combo_cycle.addItem("5.0 ms (200Hz 超高速工业对齐 - 推荐)", 5.0)
        self.combo_cycle.addItem("10.0 ms (100Hz 标准模式)", 10.0)
        self.combo_cycle.addItem("20.0 ms (50Hz 低负载模式)", 20.0)
        grid_tm.addWidget(self.combo_cycle, 0, 1)

        grid_tm.addWidget(QLabel("心跳看门狗超时:"), 1, 0)
        spin_layout = QHBoxLayout()
        self.spin_timeout = QSpinBox()
        self.spin_timeout.setRange(500, 10000)
        self.spin_timeout.setSingleStep(500)
        self.spin_timeout.setSuffix(" ms")
        self.spin_timeout.setValue(2000)

        self.spin_retry = QDoubleSpinBox()
        self.spin_retry.setRange(0.2, 10.0)
        self.spin_retry.setSingleStep(0.5)
        self.spin_retry.setSuffix(" s")
        self.spin_retry.setValue(1.0)

        spin_layout.addWidget(self.spin_timeout)
        spin_layout.addWidget(QLabel("断线重试间隔:"))
        spin_layout.addWidget(self.spin_retry)
        grid_tm.addLayout(spin_layout, 1, 1)

        grid_tm.addWidget(QLabel("通道写入策略:"), 2, 0)
        self.combo_raw_policy = QComboBox()
        self.combo_raw_policy.addItem("智能权限探测 (自动跳过只读通道，由 PLC 内部周期映射 - 推荐)", "AutoFilter")
        self.combo_raw_policy.addItem("纯符号表写入 (仅写入 PRG_pitch_sim 导出变量)", "PureSymbol")
        grid_tm.addWidget(self.combo_raw_policy, 2, 1)

        vbox_tm.addLayout(grid_tm)
        main_vbox.addWidget(grp_timing)

        # 4. 连通性测试与诊断区域 (Diagnostic & Ping Test)
        grp_diag = QWidget()
        grp_diag.setStyleSheet("background-color: #171A24; border: 1px solid #2B3145; border-radius: 6px; padding: 10px;")
        vbox_dg = QVBoxLayout(grp_diag)
        vbox_dg.setContentsMargins(8, 8, 8, 8)
        vbox_dg.setSpacing(6)

        diag_header = QHBoxLayout()
        lbl_dg_title = QLabel("🔍 连通性在线诊断 (Diagnostic Test)")
        lbl_dg_title.setStyleSheet("color: #06D6A0; font-weight: bold; font-size: 11px;")
        diag_header.addWidget(lbl_dg_title)
        diag_header.addStretch()

        self.btn_test_conn = QPushButton("⚡ 测试连接")
        self.btn_test_conn.setStyleSheet("""
            QPushButton {
                background-color: #1E3A34;
                color: #06D6A0;
                border: 1px solid #2B5E4E;
                font-weight: bold;
                padding: 4px 12px;
            }
            QPushButton:hover {
                background-color: #274D44;
                border-color: #06D6A0;
            }
        """)
        self.btn_test_conn.clicked.connect(self._on_click_test)
        diag_header.addWidget(self.btn_test_conn)
        vbox_dg.addLayout(diag_header)

        self.lbl_test_result = QLabel("点击【测试连接】可向目标服务器发起 OPC UA 握手与变量探测。")
        self.lbl_test_result.setStyleSheet("color: #8D99AE; font-size: 11px; padding: 4px; background-color: #1E212E; border-radius: 4px; border: 1px dashed #343C52;")
        self.lbl_test_result.setWordWrap(True)
        vbox_dg.addWidget(self.lbl_test_result)

        main_vbox.addWidget(grp_diag)

        # 5. 底部操作按钮栏
        main_vbox.addStretch()
        btn_bar = QHBoxLayout()
        btn_bar.setSpacing(10)

        btn_reset_default = QPushButton("恢复默认设置")
        btn_reset_default.setStyleSheet("color: #8D99AE; background-color: transparent; border: 1px solid #363D4E;")
        btn_reset_default.clicked.connect(self._on_reset_defaults)
        btn_bar.addWidget(btn_reset_default)

        btn_bar.addStretch()

        btn_cancel = QPushButton("取消")
        btn_cancel.clicked.connect(self.reject)
        btn_bar.addWidget(btn_cancel)

        btn_save = QPushButton("✔ 保存并应用")
        btn_save.setStyleSheet("""
            QPushButton {
                background-color: #06D6A0;
                color: #121318;
                font-weight: bold;
                border: 1px solid #06D6A0;
                padding: 6px 18px;
            }
            QPushButton:hover {
                background-color: #05C493;
            }
        """)
        btn_save.clicked.connect(self._on_save_and_apply)
        btn_bar.addWidget(btn_save)

        main_vbox.addLayout(btn_bar)

    def _populate_data(self):
        """填充当前配置数据到 UI 控件"""
        self.edit_endpoint.setText(str(self.config.get("endpoint", DEFAULT_CONFIG["endpoint"])))
        self.edit_prefix.setText(str(self.config.get("prefix", DEFAULT_CONFIG["prefix"])))

        target_cycle = float(self.config.get("cycle_time_ms", 5.0))
        idx = 1 # 默认 5.0ms
        for i in range(self.combo_cycle.count()):
            if abs(self.combo_cycle.itemData(i) - target_cycle) < 0.1:
                idx = i
                break
        self.combo_cycle.setCurrentIndex(idx)

        self.spin_timeout.setValue(int(self.config.get("timeout_ms", 2000)))
        self.spin_retry.setValue(float(self.config.get("retry_interval_s", 1.0)))

        policy = self.config.get("raw_policy", "AutoFilter")
        for i in range(self.combo_raw_policy.count()):
            if self.combo_raw_policy.itemData(i) == policy:
                self.combo_raw_policy.setCurrentIndex(i)
                break

    def _collect_data(self) -> Dict[str, Any]:
        """从 UI 控件收集最新配置字典"""
        return {
            "endpoint": self.edit_endpoint.text().strip(),
            "prefix": self.edit_prefix.text().strip(),
            "cycle_time_ms": float(self.combo_cycle.currentData()),
            "timeout_ms": int(self.spin_timeout.value()),
            "retry_interval_s": float(self.spin_retry.value()),
            "auth_mode": self.config.get("auth_mode", "Anonymous"),
            "username": self.config.get("username", ""),
            "password": self.config.get("password", ""),
            "raw_policy": str(self.combo_raw_policy.currentData()),
        }

    def _on_click_test(self):
        """执行异步 OPC UA 端点测试"""
        endpoint = self.edit_endpoint.text().strip()
        prefix = self.edit_prefix.text().strip()

        if not endpoint:
            self.lbl_test_result.setText("❌ 端点 URL 不能为空！")
            self.lbl_test_result.setStyleSheet("color: #EF233C; font-size: 11px; padding: 4px; background-color: #2E181D; border-radius: 4px; border: 1px solid #6E2129;")
            return

        self.btn_test_conn.setEnabled(False)
        self.lbl_test_result.setText(f"🔄 正在探测端点 {endpoint}，请稍候...")
        self.lbl_test_result.setStyleSheet("color: #FFB703; font-size: 11px; padding: 4px; background-color: #2B2317; border-radius: 4px; border: 1px solid #5A4015;")

        self._test_worker = TestWorker(endpoint, prefix)
        self._test_worker.finished.connect(self._on_test_finished)
        self._test_worker.run_test()

    def _on_test_finished(self, success: bool, msg: str, dt_ms: float, count: int):
        self.btn_test_conn.setEnabled(True)
        if success:
            self.lbl_test_result.setText(f"🟢 通讯测试成功！响应时延: {dt_ms:0.1f} ms | 成功解析根节点并发现 {count} 个有效符号。")
            self.lbl_test_result.setStyleSheet("color: #06D6A0; font-size: 11px; font-weight: bold; padding: 4px; background-color: #162E25; border-radius: 4px; border: 1px solid #23533E;")
        else:
            self.lbl_test_result.setText(f"🔴 连接失败: {msg} (耗时 {dt_ms:0.1f} ms)")
            self.lbl_test_result.setStyleSheet("color: #EF233C; font-size: 11px; font-weight: bold; padding: 4px; background-color: #2E181D; border-radius: 4px; border: 1px solid #6E2129;")

    def _on_reset_defaults(self):
        self.config = dict(DEFAULT_CONFIG)
        self._populate_data()
        self.lbl_test_result.setText("已恢复默认配置参数。")
        self.lbl_test_result.setStyleSheet("color: #8D99AE; font-size: 11px; padding: 4px; background-color: #1E212E; border-radius: 4px; border: 1px dashed #343C52;")

    def _on_save_and_apply(self):
        new_config = self._collect_data()
        save_connection_config(new_config)
        self.config = new_config

        if self.on_apply:
            try:
                self.on_apply(new_config)
            except Exception as e:
                print(f"[WARN] 应用连接配置回调异常: {e}")

        self.accept()


if __name__ == "__main__":
    from PyQt6.QtWidgets import QApplication
    app = QApplication(sys.argv)
    dlg = ConnectionSettingsDialog()
    dlg.show()
    sys.exit(app.exec())
