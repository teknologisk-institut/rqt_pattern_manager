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
from python_qt_binding.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton,QComboBox
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from .utils import Utils
from .pattern_manager_client import PatternManagerClient


class NewGroupWidget(QWidget):
    def __init__(self, pmc):
        super(NewGroupWidget, self).__init__()

        self.pmc = pmc

        Utils.load_ui('new_group.ui', self)
        self.setObjectName('NewGroupWidget')

        self.selected_gtype = self.groupBox.currentText()
        self.groupBox.activated.connect(self._on_group_box_activated)

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self.close)

    def _on_group_box_activated(self):
        self.selected_gtype = self.groupBox.currentText()

    def _on_accepted(self):
        self.pmc.create_group(self.selected_gtype, self.nameBox.text())
        self.close()


class NewPatternWidget(QWidget):
    def __init__(self, pmc):
        super(NewPatternWidget, self).__init__()

        self.pmc = pmc

        Utils.load_ui('new_pattern.ui', self)
        self.setObjectName('NewPatternWidget')

        model = QStandardItemModel(self.patternView)

        pats = self.pmc._get_pattern_types()
        for p in pats:
            item = QStandardItem(p)
            model.appendRow(item)

        self.patternView.setModel(model)

        self.dialogButton.accepted.connect(self.close)
        self.dialogButton.rejected.connect(self.close)


class PatternManagerWidget(QWidget):
    def __init__(self):
        super(PatternManagerWidget, self).__init__()

        self.pmc = PatternManagerClient()
        
        Utils.load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        btn_new_grp = self.newGroupButton
        btn_new_pat = self.newPatternButton

        self.wdg_new_grp = NewGroupWidget(self.pmc)
        self.wdg_new_pat = NewPatternWidget(self.pmc)

        btn_new_grp.clicked.connect(self.wdg_new_grp.show)
        btn_new_pat.clicked.connect(self.wdg_new_pat.show)
