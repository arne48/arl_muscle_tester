#!/usr/bin/env python

# Copyright (c) 2017, Various Authors of the rqt_common_plugins
# Copyright (c) 2017, Arne Hitzmann
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import xmlrpclib

import rospy
import rospkg
import roslib
from std_msgs.msg import Float64
from arl_hw_msgs.msg import MusculatureCommand, MuscleCommand

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, qWarning, Signal, QTimer
from python_qt_binding.QtGui import QIcon
from python_qt_binding.QtWidgets import QFileDialog, QGraphicsView, QWidget

from . rosplot import ROSData, RosPlotException
from rqt_py_common import topic_helpers


def get_topic_properties(topic_name):
    topic_type, real_topic, _ = topic_helpers.get_topic_type(topic_name)
    if topic_type is None:
        message = "topic %s does not exist" % (topic_name)
        return [], message

    slot_type, is_array, array_size = roslib.msgs.parse_type(topic_type)
    field_class = roslib.message.get_message_class(slot_type)

    return field_class, slot_type, is_array, array_size


def get_plot_fields(topic_name):
    field_class, slot_type, is_array, array_size = get_topic_properties(topic_name)

    if not roslib.msgs.is_valid_constant_type(slot_type):
        numeric_fields = []
        for i, slot in enumerate(field_class.__slots__):
            slot_type = field_class._slot_types[i]
            slot_type, is_array, array_size = roslib.msgs.parse_type(slot_type)
            slot_class = topic_helpers.get_type_class(slot_type)
            if slot_class in (int, float) and not is_array:
                numeric_fields.append(slot)
        message = ""
        if len(numeric_fields) > 0:
            message = "%d plottable fields in %s" % (len(numeric_fields), topic_name)
        else:
            message = "No plottable fields in %s" % (topic_name)
        return ["%s/%s" % (topic_name, f) for f in numeric_fields], message
    else:
        message = "Topic %s is not numeric" % (topic_name)
        return [], message


class MuscleTesterView(QGraphicsView):
    def __init__(self, parent=None):
        super(MuscleTesterView, self).__init__()


class MuscleTesterWidget(QWidget):
    set_status_text = Signal(str)
    _redraw_interval = 40

    def __init__(self, context):
        super(MuscleTesterWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('arl_muscle_tester'), 'resource', 'muscle_tester_widget.ui')
        loadUi(ui_file, self, {'MuscleTesterView': MuscleTesterView})

        self.activation_amplitude = 0.01 * self.amplitudeSlider.sliderPosition()
        self.selected_muscle_controller = ''
        self.data_plot = None
        self._update_plot_timer = QTimer(self)
        self._update_plot_timer.timeout.connect(self.update_plot)
        self._rosdata = {}
        self._start_time = rospy.get_time()

        self._ros_master = xmlrpclib.ServerProxy(os.environ['ROS_MASTER_URI'])
        self._activation_pub = rospy.Publisher('/musculature/command', MusculatureCommand, queue_size=10)

        self.setObjectName('MuscleTesterWidget')

        self.activationButton.pressed.connect(self._handle_activation_button_press)
        self.activationButton.released.connect(self._handle_activation_button_release)
        self.amplitudeSlider.valueChanged.connect(self._handle_slider_change)
        self.muscleList.currentItemChanged.connect(self._handle_item_picked)
        self.activationScaleButton.toggled.connect(self._set_ylim_by_selection)
        self.pressureScaleButton.toggled.connect(self._set_ylim_by_selection)
        self.tensionScaleButton.toggled.connect(self._set_ylim_by_selection)

        self.amplitudeLabel.setNum(self.activation_amplitude)

        self._update_muscle_topics()

    def switch_data_plot_widget(self, data_plot):
        self.enable_timer(enabled=False)

        self.data_plot_layout.removeWidget(self.data_plot)
        if self.data_plot is not None:
            self.data_plot.close()

        self.data_plot = data_plot
        self.data_plot_layout.addWidget(self.data_plot)
        self.data_plot.autoscroll(True)

        # setup drag 'n drop
        self.data_plot.dropEvent = self.dropEvent
        self.data_plot.dragEnterEvent = self.dragEnterEvent

        for topic_name, rosdata in self._rosdata.items():
            data_x, data_y = rosdata.next()
            self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    def enable_timer(self, enabled=True):
        if enabled:
            self._update_plot_timer.start(self._redraw_interval)
        else:
            self._update_plot_timer.stop()

    def update_plot(self):
        if self.data_plot is not None:
            needs_redraw = False
            for topic_name, rosdata in self._rosdata.items():
                try:
                    data_x, data_y = rosdata.next()
                    if data_x or data_y:
                        self.data_plot.update_values(topic_name, data_x, data_y)
                        needs_redraw = True
                except RosPlotException as e:
                    qWarning('PlotWidget.update_plot(): error in rosplot: %s' % e)
            if needs_redraw:
                self.data_plot.redraw()

    def _subscribed_topics_changed(self):
        self.enable_timer(self._rosdata)
        self.data_plot.redraw()

    def clear_plot(self):
        for topic_name, _ in self._rosdata.items():
            self.data_plot.clear_values(topic_name)
        self.data_plot.redraw()

    def remove_topic(self, topic_name):
        self._rosdata[topic_name].close()
        del self._rosdata[topic_name]
        self.data_plot.remove_curve(topic_name)

        self._subscribed_topics_changed()

    def add_topic(self, topic_name):
        self.clear_plot()
        for topic_name in get_plot_fields(topic_name)[0]:
            if topic_name in self._rosdata:
                qWarning('PlotWidget.add_topic(): topic already subscribed: %s' % topic_name)
                continue
            self._rosdata[topic_name] = ROSData(topic_name, self._start_time)
            if self._rosdata[topic_name].error is not None:
                qWarning(str(self._rosdata[topic_name].error))
                del self._rosdata[topic_name]
            else:
                data_x, data_y = self._rosdata[topic_name].next()
                self.data_plot.add_curve(topic_name, topic_name, data_x, data_y)

        self._subscribed_topics_changed()

    def clean_up_subscribers(self):
        for topic_name, rosdata in self._rosdata.items():
            rosdata.close()
            self.data_plot.remove_curve(topic_name)
        self._rosdata = {}

        self._subscribed_topics_changed()

    def _set_ylim_by_selection(self):
        limits = [-100, 100]
        if self.activationScaleButton.isChecked():
            limits = [-2, 2]
        elif self.pressureScaleButton.isChecked():
            limits = [0, 65535]
        elif self.tensionScaleButton.isChecked():
            limits = [0, 16777215]

        self.data_plot.set_ylim(limits)
        self.update_plot()

    def _update_muscle_topics(self):
        try:
            code, msg, val = self._ros_master.getPublishedTopics('/arl_muscle_tester_script', "")
            if code == 1:
                published_topics = dict(val)
                controller_topics = []

                for topic in published_topics:
                    # ensure that topic has the right name and type to be a
                    # topic of a MuscleController
                    field_class, _, _, _ = get_topic_properties(topic)
                    if topic.split('/')[-1] == 'state' and field_class._type == 'arl_hw_msgs/Muscle':
                        controller_topics.append(topic.split('/')[1])

                unique_controller_topics = sorted(set(controller_topics))

                self.selected_muscle_controller = unique_controller_topics[0]

                for topic in unique_controller_topics:
                    self.muscleList.addItem(topic)

            else:
                rospy.logerr("Communication with ROS Master failed")
        except:
            rospy.logerr("Something went wrong while discovering muscles, maybe no muscles present")

    def _create_musculature_msg(self, pressure=None, activation=None):
        musculature_command = MusculatureCommand()
        musculature_command.header.stamp = rospy.get_rostime()
        musculature_command.header.frame_id = '0'
        muscle_command = MuscleCommand()
        muscle_command.name = self.selected_muscle_controller.replace('_controller', '')

        if activation is not None:
            muscle_command.activation = activation
            muscle_command.control_mode = MuscleCommand.CONTROL_MODE_BY_ACTIVATION

        if pressure is not None:
            muscle_command.pressure = pressure
            muscle_command.control_mode = MuscleCommand.CONTROL_MODE_BY_PRESSURE

        musculature_command.muscle_commands.append(muscle_command)
        return musculature_command

    def _handle_activation_button_press(self):
        self._activation_pub.publish(self._create_musculature_msg(None, self.activation_amplitude))

    def _handle_activation_button_release(self):
        self._activation_pub.publish(self._create_musculature_msg(None, -0.5))

    def _handle_slider_change(self):
        self.activation_amplitude = 0.01 * self.amplitudeSlider.sliderPosition()
        self.amplitudeLabel.setNum(self.activation_amplitude)

    def _handle_item_picked(self):
        self.activationButton.setEnabled(True)
        self.clean_up_subscribers()
        self.selected_muscle_controller = self.muscleList.currentItem().text()
        self.add_topic('/' + self.selected_muscle_controller + '/state')
