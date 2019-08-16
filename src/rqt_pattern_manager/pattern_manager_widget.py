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
from PyQt5.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton, QComboBox, QTreeView
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtCore import QAbstractItemModel, QModelIndex
from .utils import Utils
from .pattern_manager_client import PatternManagerClient
import pattern_manager.msg as pm_msg
import types


class NewGroupWidget(QWidget):
    def __init__(self, pmc):
        super(NewGroupWidget, self).__init__()

        self.pmc = pmc

        Utils.load_ui('new_group.ui', self)
        self.setObjectName('NewGroupWidget')

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self.close)

    def _on_accepted(self):
        self.pmc.create_group(self.groupBox.currentText(), self.nameBox.text())
        self.close()


class NewPatternWidget(QWidget):
    def __init__(self, pmc):
        super(NewPatternWidget, self).__init__()

        self.pmc = pmc

        Utils.load_ui('new_pattern.ui', self)
        self.setObjectName('NewPatternWidget')

        self._populate_pattern_view()

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self.close)

    def _populate_pattern_view(self):
        model = QStandardItemModel(self.patternView)

        pats = self.pmc._get_pattern_types()
        for p in pats:
            item = QStandardItem(p)
            model.appendRow(item)

        self.patternView.setModel(model)

    def _on_accepted(self):
        self.pmc.create_pattern(self.patternView.currentItem(), self.nameBox.text(), self.groupView.currentItem())
        self.close()


class PatternManagerWidget(QWidget):
    def __init__(self):
        super(PatternManagerWidget, self).__init__()

        self.pmc = PatternManagerClient()
        
        Utils.load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        self._populate_tree_view()

        btn_new_grp = self.newGroupButton       # type : QPushButton
        btn_new_pat = self.newPatternButton
        btn_update = self.updateButton

        self.wdg_new_grp = NewGroupWidget(self.pmc)
        self.wdg_new_pat = NewPatternWidget(self.pmc)

        btn_new_grp.clicked.connect(self.wdg_new_grp.show)
        btn_new_pat.clicked.connect(self.wdg_new_pat.show)
        btn_update.clicked.connect(self._populate_tree_view)

    def _populate_tree_view(self):
        grp_deps = self.pmc.get_groups()

        # create nodes dictionary
        nodes = {}
        for i in grp_deps:
            name, par_name = i.name_and_parent
            nodes[name] = {}

        # create tree of parent-child relations
        tree = {}
        for i in grp_deps:
            name, par_name = i.name_and_parent
            node = nodes[name]

            if name == par_name:
                tree[name] = node
            else:
                parent = nodes[par_name]
                parent[name] = node

        model = QStandardItemModel()
        self.treeView.setModel(model)
        self._populate_model(tree, model.invisibleRootItem())

    def _populate_model(self, children, parent):
        for child in sorted(children):
            child_item = QStandardItem(child)
            parent.appendRow(child_item)
            self._populate_model(children[child], child_item)
