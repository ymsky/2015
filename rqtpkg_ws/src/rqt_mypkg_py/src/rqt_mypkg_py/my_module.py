import os
import rospy
import rospkg


from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from std_msgs.msg import UInt16

from sensor_msgs.msg import Joy


class MyPlugin(Plugin):


    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPluginPy')
	
        self._publisher = rospy.Publisher('command_request',UInt16,queue_size=10)
	self._subscriber= rospy.Subscriber('joy', Joy, self._joy_callback)


        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'MyPlugin1.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
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

	self._widget.radioButton_1.clicked.connect(self._on_RB1_checked)
	self._widget.radioButton_2.clicked.connect(self._on_RB2_checked)
	self._widget.pushButton_1.pressed.connect(self._on_PB1_pressed)
	self._widget.pushButton_2.pressed.connect(self._on_PB2_pressed)
	self._widget.pushButton_3.pressed.connect(self._on_PB3_pressed)
	self._widget.pushButton_4.pressed.connect(self._on_PB4_pressed)
	self._widget.pushButton_5.pressed.connect(self._on_PB5_pressed)
	self._widget.pushButton_6.pressed.connect(self._on_PB6_pressed)
	self._widget.pushButton_7.pressed.connect(self._on_PB7_pressed)
	self._widget.pushButton_8.pressed.connect(self._on_PB8_pressed)
	self._widget.pushButton_9.pressed.connect(self._on_PB9_pressed)

	self.m_index=0
	self.b_gui_flag=1

    def moving_camera_dummy(self,_direction):
	if (_direction==1):
		m_index=90
		print ('1_up_left')
	if (_direction==2):
		m_index=0
		print ('2_up')
	if (_direction==3):
		m_index=91
		print ('3_up_right')
	if (_direction==4):
		m_index=6
		print ('4_right')
	if (_direction==5):
		m_index=93
		print ('5_down_right')
	if (_direction==6):
		m_index=2
		print ('6_down')
	if (_direction==7):
		m_index=92
		print ('7_down_left')
	if (_direction==8):
		m_index=4
		print ('8_left')
	if (_direction==9):
		m_index=1
		print ('9_stop')	
	self._publisher.publish(m_index)	
	
    
    def moving_camera(self,_direction,_method):
	if(_method == 1):
		if(self.b_gui_flag==1):
			self.moving_camera_dummy(_direction)
		else:
			print('gui disabled')
	if(_method ==0):
		if(self.b_gui_flag==0):
			self.moving_camera_dummy(_direction)
		else:
			print('joystick disabled')
	
		
    def _on_PB1_pressed(self):
	self.moving_camera(1,1)
	
    def _on_PB2_pressed(self):
	self.moving_camera(2,1)

    def _on_PB3_pressed(self):
	self.moving_camera(3,1)

    def _on_PB4_pressed(self):
	self.moving_camera(4,1)

    def _on_PB5_pressed(self):
	self.moving_camera(5,1)

    def _on_PB6_pressed(self):
	self.moving_camera(6,1)

    def _on_PB7_pressed(self):
	self.moving_camera(7,1)

    def _on_PB8_pressed(self):
	self.moving_camera(8,1)

    def _on_PB9_pressed(self):
	self.moving_camera(9,1)


    def _on_RB1_checked(self):
	self.b_gui_flag=0
	print 'joystick enabled'
    def _on_RB2_checked(self):
	self.b_gui_flag=1
	print 'gui_control enabled'

    def _joy_callback(self,data):
	if data.axes[0]==1 and data.axes[1]==1:
		self.moving_camera(1,0)

	if data.axes[0]==0 and data.axes[1]==1:
		self.moving_camera(2,0)

	if data.axes[0]==-1 and data.axes[1]==1:
		self.moving_camera(3,0)

	if data.axes[0]==-1 and data.axes[1]==0:
		self.moving_camera(4,0)

	if data.axes[0]==-1 and data.axes[1]==-1:
		self.moving_camera(5,0)

	if data.axes[0]==0 and data.axes[1]==-1:
		self.moving_camera(6,0)

	if data.axes[0]==1 and data.axes[1]==-1:
		self.moving_camera(7,0)

	if data.axes[0]==1 and data.axes[1]==0:
		self.moving_camera(8,0)

	if data.axes[0]==0 and data.axes[1]==0:
		self.moving_camera(9,0)





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
