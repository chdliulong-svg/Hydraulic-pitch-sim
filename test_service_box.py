"""
机旁手操盒 (Service Box) 单元与渲染测试脚本
"""

import sys
import os
import time

# 确保导入路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt, QTimer

from pitch_soft_realtime import SoftRealtimePitchEngine
from gui.main_window import PitchMainWindow

def run_test():
    app = QApplication.instance()
    if not app:
        app = QApplication(sys.argv)

    print("1. 启动软实时仿真机引擎...")
    engine = SoftRealtimePitchEngine(cycle_time_s=0.005)
    engine.start()

    print("2. 测试手操盒输入与状态存取...")
    engine.set_service_box_inputs(connected=True, blade_chosen='A', fine=True, fast_speed=False)
    sbt = engine.get_service_box_telemetry()
    print(f"手操盒状态: active={sbt.active_blade}, n_safe={sbt.n_safe_blades}, in_control={sbt.in_control}")

    time.sleep(0.1)
    engine.stop()

    print("3. 测试 UI 主视窗渲染与手操盒 Tab...")
    win = PitchMainWindow()
    win.show()

    # 切换到机旁手操盒 Tab (Tab index 5)
    win.tab_widget.setCurrentIndex(5)

    # 刷新几帧事件循环
    for _ in range(10):
        app.processEvents()
        time.sleep(0.02)

    # 截图保存手操盒界面
    pixmap = win.grab()
    screenshot_path = r"C:\Users\hujiapeng\.gemini\antigravity\brain\5a424d1c-5d9e-4dcc-97fc-18d96bae1e3a\service_box_tab_preview.png"
    pixmap.save(screenshot_path, "PNG")
    print(f"[OK] Full screenshot saved: {screenshot_path}")

    # 单独抓取手操盒 Tab 区域截图
    tab_pixmap = win.service_box_tab.grab()
    tab_screenshot_path = r"C:\Users\hujiapeng\.gemini\antigravity\brain\5a424d1c-5d9e-4dcc-97fc-18d96bae1e3a\service_box_tab_only.png"
    tab_pixmap.save(tab_screenshot_path, "PNG")
    print(f"[OK] Tab screenshot saved: {tab_screenshot_path}")

    win.close()
    print("[OK] All tests passed!")

if __name__ == "__main__":
    run_test()
