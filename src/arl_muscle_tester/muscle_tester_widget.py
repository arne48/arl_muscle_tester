
import os
import sys
import xmlrpclib

import rospy
import rospkg
from std_msgs.msg import Float64

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsView, QWidget

class MuscleTesterView(QGraphicsView):
    def __init__(self, parent=None):
        super(MuscleTesterView, self).__init__()


class MuscleTesterWidget(QWidget):
    set_status_text = Signal(str)

    def __init__(self, context):
        super(MuscleTesterWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('arl_muscle_tester'), 'resource', 'muscle_tester_widget.ui')
        loadUi(ui_file, self, {'MuscleTesterView': MuscleTesterView})

        self.activation_amplitude = 0.01 * self.amplitudeSlider.sliderPosition()
        self.selected_muscle_controller = ''

        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])

        self.setObjectName('MuscleTesterWidget')

        self.activationButton.pressed.connect(self._handle_activation_button_press)
        self.activationButton.released.connect(self._handle_activation_button_release)
        self.amplitudeSlider.valueChanged.connect(self._handle_slider_change)
        self.muscleList.currentItemChanged.connect(self._handle_item_picked)

        self.amplitudeLabel.setNum(self.activation_amplitude)

        self._update_muscle_topics()

        self._activation_pub = rospy.Publisher('/' + self.selected_muscle_controller + '/activation_command', Float64,
                                               queue_size=1)

    def _update_muscle_topics(self):
        try:
            code, msg, val = self._ros_master.getPublishedTopics('/arl_muscle_tester_script', "")
            if code == 1:
                published_topics = dict(val)
                controller_topics = []

                for topic in published_topics:
                    if topic.split('/')[1].startswith('muscle_'):
                        controller_topics.append(topic.split('/')[1])

                unique_controller_topics = sorted(set(controller_topics))

                self.selected_muscle_controller = unique_controller_topics[0]

                for topic in unique_controller_topics:
                    self.muscleList.addItem(topic)

            else:
                rospy.logerr("Communication with ROS Master failed")
        except:
            sys.exit(1)

    def _handle_activation_button_press(self):
        self._activation_pub.publish(self.activation_amplitude)

    def _handle_activation_button_release(self):
        self._activation_pub.publish(-0.5)

    def _handle_slider_change(self):
        self.activation_amplitude = 0.01 * self.amplitudeSlider.sliderPosition()
        self.amplitudeLabel.setNum(self.activation_amplitude)

    def _handle_item_picked(self):
        self.selected_muscle_controller = self.muscleList.currentItem().text()
        self._activation_pub = rospy.Publisher('/' + self.selected_muscle_controller + '/activation_command', Float64,
                                               queue_size=1)

