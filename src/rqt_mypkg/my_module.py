#!/usr/bin/env python
import os
import rospy
import rospkg
import cv2 as cv

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtGui, QtCore
from QtCore import Qt
from python_qt_binding.QtWidgets import QWidget, QPushButton, QLabel


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
        self._widget.Command.clicked.connect(self.command_slot)
        #self._widget.Command.clicked[bool].connect(self.command_slot)
        # Log information about signal and slot connection
        rospy.loginfo('Connected %s %s with %s slot'%(self._widget.Command.objectName(),QPushButton.__name__, self.command_slot.__name__))

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

    def command_slot(self):
        rospy.loginfo('Clicked %s'%(self.command_slot.__name__))
        camera = cv.VideoCapture(0)
        ret, frame = camera.read()
        height, width = frame.shape[:2]
        frame_disp = QtGui.QImage(
            frame.data, width, height, QtGui.QImage.Format_RGB888
        )
        pix_map = QtGui.QPixmap.fromImage(frame_disp)
        self._widget.ImageFrame.setPixmap(pix_map.scaled(self._widget.ImageFrame.width(),self._widget.ImageFrame.height(),Qt.KeepAspectRatio))
        #scene = QGraphicsScene()
        #scene.addPixmap(pix_map)
        #self._widget.GraphicsView.setScene(pix_map)
        
        pass


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
