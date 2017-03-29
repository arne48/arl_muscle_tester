
import os
import argparse
import threading

from qt_gui.plugin import Plugin

from .muscle_tester_widget import MuscleTesterWidget


class MuscleTester(Plugin):
    def __init__(self, context):
        """
        :param context: plugin context hook to enable adding widgets as a ROS_GUI pane, ''PluginContext''
        """
        super(MuscleTester, self).__init__(context)
        self.setObjectName('MuscleTester')

        args = self._parse_args(context.argv())

        self._widget = MuscleTesterWidget(context)
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

