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
from python_qt_binding.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton


class PatternManagerWidget(QWidget):
    def __init__(self):
        super(PatternManagerWidget, self).__init__()
        
        self.load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        self.w_new_grp = QWidget()
        self.w_new_pat = QWidget()

        self.load_ui("new_group.ui", self.w_new_grp)
        self.load_ui("new_pattern.ui", self.w_new_pat)

        btn_new_grp = self.newGroupButton
        btn_new_pat = self.newPatternButton

        btn_new_grp.clicked.connect(self.w_new_grp.show)
        btn_new_pat.clicked.connect(self.w_new_pat.show)

        self.w_new_grp.dialogButton.accepted.connect(self.w_new_grp.close)
        self.w_new_grp.dialogButton.rejected.connect(self.w_new_grp.close)

        self.w_new_pat.dialogButton.accepted.connect(self.w_new_pat.close)
        self.w_new_pat.dialogButton.rejected.connect(self.w_new_pat.close)


    def load_ui(self, file, widget):
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_pattern_manager'), 'resource', file)
        loadUi(ui_file, widget)
