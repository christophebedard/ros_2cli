# Copyright 2023 Christophe Bedard
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from argcomplete import autocomplete
import argparse
import os
import subprocess
import sys

from ros2cli.command import _is_completion_requested
from ros2cli import cli


def main():
    # Overall, the trick to make this work is to remove '2' from the command in autocompletion mode
    # and in normal command/execution mode (but not both at the same time) and relay both
    # autocompletion and command execution to ros2cli

    # Parser to autocomplete *to* 'ROS 1' and 'ROS 2' and print a help message
    ros_1_and_2_parser = argparse.ArgumentParser()
    ros_1_and_2_cmds = ros_1_and_2_parser.add_subparsers(title='Commands')
    ros_1_and_2_cmds.add_parser('1')
    ros_1_and_2_cmds.add_parser('2')

    # Autocompletion mode
    if _is_completion_requested():
        # When in autocompletion mode, the commandline string (that should be autocompleted) is
        # provided through this env var
        comp_line = os.environ['COMP_LINE']

        # Autocomplete *to* 'ROS 1' and 'ROS 2'
        comp_line_stripped = comp_line.strip(' ')
        if comp_line_stripped in ('ROS', 'ROS 1', 'ROS 2'):
            # Stop autocompletion after 'ROS 1', i.e., do not autocomplete 'ROS 1 '
            if comp_line_stripped == 'ROS 1' and comp_line.endswith(' '):
                return 0
            # And make sure to only autocomplete *to* 'ROS 2'; we want to let ros2cli do the rest
            elif not (comp_line_stripped == 'ROS 2' and comp_line.endswith(' ')):
                autocomplete(ros_1_and_2_parser)
                ros_1_and_2_parser.parse_args()
                return 0

        # Otherwise, to autocomplete 'ROS 2 ...', remove '2' from commandline string and let
        # ros2cli do the autocompletion
        comp_line = comp_line.replace('2 ', '')
        os.environ['COMP_LINE'] = comp_line
    # Normal execution mode
    else:
        # If no arguments, print help
        if len(sys.argv) <= 1:
            ros_1_and_2_parser.print_help()
            return 0

        first_cmd = sys.argv[1]
        # If help is requested, or if the command is just 'ROS 1', print help
        if first_cmd in ('-h', '--help') or first_cmd == '1' and len(sys.argv) <= 2:
            ros_1_and_2_parser.print_help()
            return 0
        # For 'ROS 1 ...', remove the '1' and concatenate first actual command with 'ros'
        # (e.g., 'ROS 1 topic list' --> 'rostopic list')
        elif first_cmd == '1':
            ros_1_cmd = ['ros' + sys.argv[2], *sys.argv[3:]]
            return subprocess.run(ros_1_cmd).returncode
        # For 'ROS 2 ...', simply remove the '2'
        # No need to replace '*ROS', since the first argument (executable name) is ignored anyway
        elif first_cmd == '2':
            sys.argv = [sys.argv[0], *sys.argv[2:]]

    # Let ros2cli do the rest
    return cli.main()
