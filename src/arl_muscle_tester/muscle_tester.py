#!/usr/bin/env python
import argparse

from qt_gui.plugin import Plugin

from .muscle_tester_widget import MuscleTesterWidget
from .data_plot import DataPlot


class MuscleTester(Plugin):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(MuscleTester, self).__init__(context)
        self.setObjectName('MuscleTester')

        args = self._parse_args(context.argv())

        self._widget = MuscleTesterWidget(context)

        self._data_plot = DataPlot(self._widget)

        # disable autoscaling of X, and set a sane default range
        self._data_plot.set_autoscale(x=False)
        self._data_plot.set_autoscale(y=False)
        self._data_plot.set_ylim([-2, 2])
        self._data_plot.set_xlim([0, 10.0])

        self._widget.switch_data_plot_widget(self._data_plot)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

    def _parse_args(self, argv):
        parser = argparse.ArgumentParser(prog='rqt_arl_muscle_tester', add_help=False)
        MuscleTester.add_arguments(parser)
        return parser.parse_args(argv)

    @staticmethod
    def add_arguments(parser):
        group = parser.add_argument_group('Options for the rqt_arl_muscle_tester plugin')
        group.add_argument('--muscle-number', type=int, default=31, help='sets the number of muscles to choose from')

    def shutdown_plugin(self):
        self._widget.clean_up_subscribers()
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

