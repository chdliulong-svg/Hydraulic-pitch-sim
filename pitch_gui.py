"""
兆瓦级风力发电机组液压变桨仿真系统 - 数字孪生实时监控终端主入口
=============================================================================
本程序已重构为模块化架构 (gui/)，主窗口及各子系统监控组件均已解耦独立维护。
本入口文件保持完全向后兼容，支持直接运行或作为模块导入。
=============================================================================
"""

import sys
from PyQt6.QtWidgets import QApplication

from gui import (
    PitchMainWindow,
    HydraulicSchematicWidget,
    BladeFaceTabWidget,
    LeftControlPanel,
    LubricationTabWidget,
    TemperatureTabWidget,
)

__all__ = [
    "PitchMainWindow",
    "HydraulicSchematicWidget",
    "BladeFaceTabWidget",
    "LeftControlPanel",
    "LubricationTabWidget",
    "TemperatureTabWidget",
    "main",
]


def main():
    """仿真主程序入口"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    win = PitchMainWindow()
    if "--remote" in sys.argv:
        win._set_control_authority("REMOTE")
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
