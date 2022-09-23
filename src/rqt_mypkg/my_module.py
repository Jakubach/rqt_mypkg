#!/usr/bin/env python
import os
from re import L
import string
import rospy
import rospkg
import cv2 as cv

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from QtCore import Qt,Signal
from python_qt_binding.QtWidgets import QWidget, QPushButton, QLabel
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class MyPlugin(Plugin):

    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_mypkg'), 'resource', 'MyPluginExtended.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)

        # Adding behaviour to objects
        # Get the object of "Command" QPushButton
        #children = self._widget.findChildren(QPushButton,"Command")
        # Connect the button with slot
        #children[0].clicked.connect(self.command_slot)
        self._widget.CommandA.clicked.connect(self.command_a_slot)
        self._widget.CommandB.toggled[bool].connect(self.command_b_slot)
        self._widget.TopicsList.addItem('')
        self.refresh_topics()
        self._widget.TopicsList.activated[int].connect(self.refresh_topics)
        #self._widget.Command.clicked[bool].connect(self.command_slot)
        # Log information about signal and slot connection
        rospy.loginfo('Connected %s %s with %s slot'%(self._widget.CommandA.objectName(),QPushButton.__name__, self.command_a_slot.__name__))
        #self.subscriber = rospy.Subscriber('camera_image',Image,self.image_processing)

        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

    def command_a_slot(self):
        rospy.loginfo('Clicked %s'%(self.command_a_slot.__name__))
        camera = cv.VideoCapture(0)
        ret, frame = camera.read()
        self.show_image(frame)
        
    def refresh_topics(self,current_index = 0):
        rospy.loginfo('Clicked %s'%(self.refresh_topics.__name__))
        self._widget.TopicsList.clear()
        self._widget.TopicsList.addItem('')
        topics_list = rospy.get_published_topics()
        for topic_name,message_type in topics_list:
            if('sensor_msgs/Image' in message_type):
                self._widget.TopicsList.addItem(topic_name)
        self._widget.TopicsList.setCurrentIndex(current_index)
        self.command_b_slot(True,self._widget.TopicsList.currentText())

    def show_image(self,frame):
        frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        height, width = frame.shape[:2]
        frame_disp = QtGui.QImage(frame.data, width, height, QtGui.QImage.Format_RGB888)
        pix_map = QtGui.QPixmap.fromImage(frame_disp)
        self._widget.ImageFrame.setPixmap(pix_map.scaled(self._widget.ImageFrame.width(),self._widget.ImageFrame.height(),Qt.KeepAspectRatio))

    def image_processing(self,img_msg):
        try:
            frame = CvBridge().imgmsg_to_cv2(img_msg)
            self.show_image(frame)
        except CvBridgeError as e:
            rospy.logwarn(e)
        


    def command_b_slot(self, checked,image_topic='camera_image'):
        rospy.loginfo('Toggled %s into %s'%(self.command_b_slot.__name__,checked))
        if(image_topic==''):
            return
        if(checked):
            self.subscriber = rospy.Subscriber(image_topic,Image,self.image_processing)
        else:
            self.subscriber.unregister()


    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
