#!/usr/bin/env python
# lpetrich 23/07/18

from .visual_servoing_teleop import VisualServoingTeleopWidget
from qt_gui.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
import rospy

class VisualServoingTeleopPlugin(Plugin):
    def __init__(self, context):
        super(VisualServoingTeleopPlugin, self).__init__(context)
        if context.serial_number() > 1:
            raise RuntimeError("You may not run more than one instance of vs_teleop.")
        self.setObjectName("Visual Servoing Plugin")
        self._widget = VisualServoingTeleopWidget()
        context.add_widget(self._widget)

        # self._update_parameter_timer = QTimer(self)
        # self._update_parameter_timer.timeout.connect(self._widget.spin)
        # self._update_parameter_timer.start(30)
