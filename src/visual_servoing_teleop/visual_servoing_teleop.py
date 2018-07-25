#!/usr/bin/env python
# lpetrich 23/07/18


import rospy, cv2, re, sys
import numpy as np
from std_msgs.msg import String, Bool, UInt32
from sensor_msgs.msg import CompressedImage, JointState, Image
from geometry_msgs.msg import PoseStamped, Pose
from uncalibrated_visual_servoing.msg import Error
from uncalibrated_visual_servoing.msg import TaskIds

from cv_bridge import CvBridge, CvBridgeError
from python_qt_binding.QtCore import *
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *

TEXT_WIDTH = 420
TEXT_HEIGHT = 150

class VisualServoingTeleopWidget(QWidget):
#############################################################################################################
# INITIALIZATION METHODS
#############################################################################################################
	def __init__(self):
		super(VisualServoingTeleopWidget, self).__init__()
		self.setWindowTitle("Visual Servoing Widget")
		# global variables
		self.cvbridge = CvBridge()
		self._clicked_points = []
		self.clicked_points = []
		self._tracked_points = []
		self.tracked_points = []
		self.currently_tracking = False
		self.task_ids = []
		self.stereo_vision = self.camera_check()
		self.move_now = False
		# setup subscribers and publishers
		sub_cam = rospy.Subscriber("/cam1/camera/image_raw/compressed", CompressedImage, self.callback_camera, queue_size = 3)
		sub_mtf = rospy.Subscriber("/cam1/trackers/patch_tracker", String, self.callback_mtf, queue_size = 3)
		sub_err = rospy.Subscriber("/image_error", Error, self.callback_error, queue_size = 3)
		self.pub_points = rospy.Publisher("/cam1/task_coordinates", String, queue_size = 1)
		self.pub_ids = rospy.Publisher("/task_ids", TaskIds, queue_size = 1)
		self.pub_calculate = rospy.Publisher("/calculate", Bool, queue_size = 1)
		self.pub_reset = rospy.Publisher("/reset", Bool, queue_size = 1)
		self.pub_move = rospy.Publisher("/move", Bool, queue_size = 1)
		if self.stereo_vision:
			self._clicked_points2 = []
			self.clicked_points2 = []
			self._tracked_points2 = []
			self.tracked_points2 = []
			sub_cam2 = rospy.Subscriber("/cam2/camera/image_raw/compressed", CompressedImage, self.callback_camera2, queue_size = 3)
			sub_mtf2 = rospy.Subscriber("/cam2/trackers/patch_tracker", String, self.callback_mtf2, queue_size = 3)
			self.pub_points2 = rospy.Publisher("/cam2/task_coordinates", String, queue_size = 1)
		self.initialize_widgets()

	def camera_check(self):
		topics = rospy.get_published_topics()
		text = ''.join(str(t) for topic in topics for t in topic)
		if re.match('.*?(cam2/camera).*?', text) != None:
			return True
		else:
			return False

	def initialize_widgets(self):
		self.current_widget = "vs"
		self.main_layout = QVBoxLayout()
		self.stacked_widget = QStackedWidget()
		self.vs_widget = QWidget()
		self.vs_widget.setLayout(self.menu_layout())
		self.stacked_widget.addWidget(self.vs_widget)
		self.grasp_widget = QWidget()
		self.grasp_widget.setLayout(self.grasp_layout())
		self.stacked_widget.addWidget(self.grasp_widget)
		self.main_layout.addWidget(self.stacked_widget)
		self.setLayout(self.main_layout)
		self.pens = [QPen(Qt.green), QPen(Qt.blue), QPen(Qt.red), QPen(Qt.magenta), QPen(Qt.yellow), QPen(Qt.darkBlue), QPen(Qt.darkMagenta), QPen(Qt.cyan)]
		for i in range(len(self.pens)):
			self.pens[i].setWidth(2)
			self.pens[i].setCapStyle(Qt.RoundCap)
			self.pens[i].setJoinStyle(Qt.RoundJoin)
		self.pens[0].setWidth(5)

	def spin(self):
		self.pub_move.publish(self.move_now)

#############################################################################################################
# WIDGETS
#############################################################################################################
	def publish_button_function(self):
		self.pub_points.publish(self.format_string(self.clicked_points))
		if self.stereo_vision:
			self.pub_points2.publish(self.format_string(self.clicked_points2))
			self.clicked_points2 = []
		self.pub_calculate.publish(True)
		self.set_ids()
		self.pub_ids.publish(self.task_ids)
		self.clicked_points = []
		self.currently_tracking = True

	def set_ids(self):
		self.task_ids = [0]*(len(self.clicked_points)/2)
		print "task ids: ", self.task_ids

	def reset(self):
		self.currently_tracking = False
		self.pub_reset.publish(True)
		self.pub_calculate.publish(False)
		self.clicked_points = []
		self.tracked_points = []
		self.error_msg.setText(" ")
		if self.stereo_vision:
			self.clicked_points2 = []
			self.tracked_points2 = []

	def grasp_now(self):
		self.change_widget("grasp")

	def go(self):
		self.move_now = True

	def stop(self):
		self.move_now = False

	def menu_layout(self):
		l = QVBoxLayout()
		layout = QHBoxLayout()
		button_layout = QHBoxLayout()
		move_layout = QVBoxLayout()

		select_msg = QLabel()
		select_msg.setText("Select 4 points to create a tracker\nSet end effector position first, then target")
		select_msg.setWordWrap(True)
		select_msg.setStyleSheet("font-size: 32px")
		select_msg.setAlignment(Qt.AlignCenter)
		select_msg.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

		self.error_msg = QLabel()
		self.error_msg.setText(" ")
		self.error_msg.setWordWrap(True)
		self.error_msg.setStyleSheet("font-size: 32px")
		self.error_msg.setAlignment(Qt.AlignCenter)
		self.error_msg.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)

		publish_button = QPushButton("publish")
		publish_button.clicked.connect(self.publish_button_function)
		publish_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 60%); font-size: 32px")
		publish_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		reset_button = QPushButton("reset")
		reset_button.clicked.connect(self.reset)
		reset_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 60%); font-size: 32px")
		reset_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		grasp_button = QPushButton("switch to grasping mode")
		grasp_button.clicked.connect(self.grasp_now)
		grasp_button.setStyleSheet("background-color: rgba(16, 123, 227, 90%); selection-background-color: rgba(16, 123, 227, 60%); font-size: 32px")
		grasp_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		go_button = QPushButton("go")
		go_button.clicked.connect(self.go)
		go_button.setStyleSheet("background-color: rgba(36, 162, 122, 90%); selection-background-color: rgba(36, 162, 122, 70%); font-size: 32px")
		go_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		stop_button = QPushButton("stop")
		stop_button.clicked.connect(self.stop)
		stop_button.setStyleSheet("background-color: rgba(36, 162, 122, 90%); selection-background-color: rgba(36, 162, 122, 70%); font-size: 32px")
		stop_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
		self._view = QLabel()
		self._view.setAlignment(Qt.AlignCenter)
		self._view.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
		self._view.mousePressEvent = self.mousePressEventCam1
		button_layout.addWidget(publish_button)
		button_layout.addWidget(reset_button)
		button_layout.addWidget(grasp_button)
		layout.addWidget(self._view)
		if self.stereo_vision:
			self._view2 = QLabel()
			self._view2.setAlignment(Qt.AlignCenter)
			self._view2.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
			self._view2.mousePressEvent = self.mousePressEventCam2
			layout.addWidget(self._view2)
		move_layout.addWidget(go_button)
		move_layout.addWidget(stop_button)
		# layout.addLayout(move_layout)
		l.addWidget(select_msg)
		l.addLayout(layout)
		l.addLayout(button_layout)
		l.addWidget(self.error_msg)
		return l

	def vs_now(self):
		self.change_widget("vs")

	def grasp_layout(self):
		l = QVBoxLayout()
		layout = QHBoxLayout()
		button_layout = QHBoxLayout()
		move_layout = QVBoxLayout()

		select_msg = QLabel()
		select_msg.setText("Select 4 points to create a tracker\nSet end effector position first, then target")
		select_msg.setWordWrap(True)
		select_msg.setStyleSheet("font-size: 32px")
		select_msg.setAlignment(Qt.AlignCenter)
		select_msg.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
		publish_button = QPushButton("publish")
		publish_button.clicked.connect(self.publish_button_function)
		publish_button.setStyleSheet("background-color: rgba(36, 162, 122, 90%); selection-background-color: rgba(36, 162, 122, 60%); font-size: 32px")
		publish_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
		reset_button = QPushButton("reset")
		reset_button.clicked.connect(self.reset)
		reset_button.setStyleSheet("background-color: rgba(36, 162, 122, 90%); selection-background-color: rgba(36, 162, 122, 60%); font-size: 32px")
		reset_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
		vs_button = QPushButton("switch to visual servoing mode")
		vs_button.clicked.connect(self.vs_now)
		vs_button.setStyleSheet("background-color: rgba(36, 162, 122, 90%); selection-background-color: rgba(36, 162, 122, 60%); font-size: 32px")
		vs_button.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
		self._viewg = QLabel()
		self._viewg.setAlignment(Qt.AlignCenter)
		self._viewg.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
		self._viewg.mousePressEvent = self.mousePressEventCam1
		button_layout.addWidget(publish_button)
		button_layout.addWidget(reset_button)
		button_layout.addWidget(vs_button)
		layout.addWidget(self._viewg)
		if self.stereo_vision:
			self._viewg2 = QLabel()
			self._viewg2.setAlignment(Qt.AlignCenter)
			self._viewg2.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
			self._viewg2.mousePressEvent = self.mousePressEventCam2
			layout.addWidget(self._viewg2)
		l.addWidget(select_msg)
		l.addLayout(layout)
		l.addLayout(button_layout)
		return l

#############################################################################################################
# WIDGET CHANGE METHODS
#############################################################################################################
	def activate_widget(self, index):
		i = self.stacked_widget.currentIndex()
		if i != index:
			self.stacked_widget.setCurrentIndex(index)

	def change_widget(self, w):
		self.current_widget = w
		if self.current_widget == "vs":
			self.activate_widget(0)
		elif self.current_widget == "grasp":
			self.activate_widget(1)
		else:
			print "Change widget error"

#############################################################################################################
# PUBLISHING METHODS
#############################################################################################################
	def format_string(self, my_list):
		msg = ""
		for i in range(len(my_list)):
			point_list = my_list[i]
			for j in range(len(point_list)):
				p = point_list[j]
				msg = msg + str(p[0]) + " " + str(p[1]) + " "
			msg = msg + "; "
		return msg

	def format_tasks(self, my_list):
		msg = ""
		for item in my_list:
			msg += (str(item) + " ")
		return msg

#############################################################################################################
# DEBUGGING METHODS
#############################################################################################################
	def debug(self):
		print "********************************************************\nDEBUGGING INFORMATION: "
		print "temp points: ", self._clicked_points
		print "clicked points: ", self.clicked_points
		print "tracked points: ", self.tracked_points
		print "********************************************************\n"

#############################################################################################################
# CALLBACK METHODS
#############################################################################################################
	def callback_camera(self, data):
		# self.pixmap = self.convert_img(data)
		self.pixmap = self.convert_compressed_img(data)
		self.paint_pixmap(self.pixmap)
		if self.current_widget == "vs":
			self._view.setPixmap(self.pixmap)
		else:
			self._viewg.setPixmap(self.pixmap)

	def callback_mtf(self, data):
		self.tracked_points = []
		if self.currently_tracking:
			pts = data.data.split()
			l = ""
			for i in range(0, len(pts), 2):
				self._tracked_points.append([float(pts[i]), float(pts[i + 1])])
				if len(self._tracked_points) == 4:
					self.tracked_points.append(self._tracked_points)
					self._tracked_points = []

	def callback_camera2(self, data):
		# self.pixmap = self.convert_img(data)
		self.pixmap2 = self.convert_compressed_img(data)
		self.paint_pixmap2(self.pixmap2)
		if self.current_widget == "vs":
			self._view2.setPixmap(self.pixmap2)
		else:
			self._viewg2.setPixmap(self.pixmap2)

	def callback_mtf2(self, data):
		self.tracked_points2 = []
		if self.currently_tracking:
			pts = data.data.split()
			l = ""
			for i in range(0, len(pts), 2):
				self._tracked_points2.append([float(pts[i]), float(pts[i + 1])])
				if len(self._tracked_points2) == 4:
					self.tracked_points2.append(self._tracked_points2)
					self._tracked_points2 = []

	def callback_error(self, data):
		err = data.error
		msg = "CURRENT IMAGE ERROR:\n("
		for i in range(0, len(err)):
			msg += ("%.3f, " % err[i])
		msg = msg[:-2] + ")"
		self.error_msg.setText(msg)
		err_norm = np.linalg.norm(err)
		if err_norm > 1000:
			print "LOST TRACKER -- RESET ALL"
			self.reset()

#############################################################################################################
# IMAGE METHODS
#############################################################################################################
	def paint_pixmap(self, pixmap):
		painter = QPainter(pixmap)
		if self._clicked_points:
			self.paint_points(painter, self._clicked_points)
		if self.clicked_points:
			self.paint_polygons(painter, self.clicked_points)
		if self.tracked_points:
			self.paint_polygons(painter, self.tracked_points)
		
	def paint_pixmap2(self, pixmap):
		painter = QPainter(pixmap)
		if self._clicked_points2:
			self.paint_points(painter, self._clicked_points2)
		if self.clicked_points2:
			self.paint_polygons(painter, self.clicked_points2)
		if self.tracked_points2:
			self.paint_polygons(painter, self.tracked_points2)
		
	def paint_points(self, painter, pt_list):
		painter.setPen(self.pens[0])
		for pt in pt_list:
			painter.drawPoint(pt[0], pt[1])

	def paint_polygons(self, painter, poly_list):
		idx = 1
		for p in poly_list:
			painter.setPen(self.pens[idx % len(self.pens)])
			painter.drawPolygon(QPointF(p[0][0], p[0][1]), QPointF(p[1][0], p[1][1]), QPointF(p[2][0], p[2][1]), QPointF(p[3][0], p[3][1]))
			idx += 1

	def convert_img(self, data):
		try:
			frame = self.cvbridge.imgmsg_to_cv2(data, "rgb8")
		except CvBridgeError as e:
			print(e)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

	def convert_compressed_img(self, data):
		np_arr = np.fromstring(data.data, np.uint8)
		frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
		frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		image = QImage(frame, frame.shape[1], frame.shape[0], frame.strides[0], QImage.Format_RGB888)
		pixmap = QPixmap.fromImage(image)
		return pixmap

#############################################################################################################
# EVENT METHODS
#############################################################################################################
	def mousePressEventCam1(self, event):
		pos = self._view.mapFromGlobal(event.globalPos())
		self._clicked_points.append([pos.x(), pos.y()])
		if len(self._clicked_points) == 4:
			self.clicked_points.append(self._clicked_points)
			self._clicked_points = []

	def mousePressEventCam2(self, event):
		pos = self._view2.mapFromGlobal(event.globalPos())
		self._clicked_points2.append([pos.x(), pos.y()])
		if len(self._clicked_points2) == 4:
			self.clicked_points2.append(self._clicked_points2)
			self._clicked_points2 = []

	def keyPressEvent(self, QKeyEvent):
		key = QKeyEvent.key()
		if int(key) == 82: # r
			self.reset()
		elif int(key) == 68: # d
			self.debug()
		elif int(key) == 81: # q
			sys.exit()
		else:
			print "Unknown key option: ", key
