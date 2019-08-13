#!/usr/bin/env python

# Copyright 2019 Danish Technological Institute (DTI)

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Author: Mads Vainoe Baatrup

import os
import rospy
import rospkg

from python_qt_binding import loadUi


class Utils():
    @staticmethod
    def load_ui(file, widget):
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_pattern_manager'), 'resource', file)
        loadUi(ui_file, widget)
