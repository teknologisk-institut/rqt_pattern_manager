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
from PyQt5.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton, QComboBox, QTreeView, QMenu, qApp, QAction
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtCore import QModelIndex, Qt
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
        self.wdg_new_grp = NewGroupWidget(self.pmc)
        self.wdg_new_pat = NewPatternWidget(self.pmc)

        Utils.load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        self.tree_model = QStandardItemModel()
        self.treeView.setModel(self.tree_model)
        self.treeView.setHeaderHidden(True)
        self._populate_tree_view(self.tree_model)
        self.treeView.expandAll()

        self.table_model = QStandardItemModel()
        self.parameterView.setModel(self.table_model)
        self.parameterView.horizontalHeader().hide()

        selectionModel = self.treeView.selectionModel()
        selectionModel.selectionChanged.connect(self._populate_parameter_view)

        btn_update = self.updateButton
        btn_update.clicked.connect(self._on_update_clicked)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self.openMenu)

    def openMenu(self, position):
        menu = QMenu()
        addPatAction = menu.addAction("Add Pattern..")
        addGrpAction = menu.addAction("Add Group..")

        action = menu.exec_(self.treeView.mapToGlobal(position))

        if action == addPatAction:
            wdg_new_pat = NewPatternWidget(self.pmc)
            wdg_new_pat.show()
        elif action == addGrpAction:
            wdg_new_grp = NewGroupWidget(self.pmc)
            wdg_new_grp.show()

    def _populate_tree_view(self, model):
        grp_deps = self.pmc.get_groups()
        pat_deps = self.pmc.get_patterns()
        deps = grp_deps + pat_deps

        nodes = {}
        types = {}
        for i in deps:
            name, par_name = i.name_and_parent
            nodes[name] = {}
            types[name] = i.type

        tree = Utils.deps_tree_from_nested_lists(deps, nodes)
        self._populate_model(tree, model.invisibleRootItem(), types)

    def _populate_model(self, children, parent, types):
        for child in sorted(children):
            child_item = QStandardItem(child)
            child_item.setWhatsThis(types[child])
            child_item.setEditable(False)
            parent.appendRow(child_item)
            self._populate_model(children[child], child_item, types)

    def _on_update_clicked(self):
        index = self.treeView.currentIndex()
        item = self.tree_model.itemFromIndex(index)

        print 'item_name: \'{}\' | type: {}'.format(item.text(), item.whatsThis())

    def _populate_parameter_view(self):
        self.table_model.clear()

        self.table_model.setColumnCount(1)
        self.table_model.setRowCount(2)

        index = self.treeView.currentIndex()
        cur_item = self.tree_model.itemFromIndex(index)

        type_header = QStandardItem('Type')
        type = QStandardItem(cur_item.whatsThis())
        type.setEditable(False)

        self.table_model.setVerticalHeaderItem(0, type_header)
        self.table_model.setItem(0, 0, type)

        name_header = QStandardItem('Name')
        name = QStandardItem(cur_item.text())
        name.setEditable(False)

        self.table_model.setVerticalHeaderItem(1, name_header)
        self.table_model.setItem(1, 0, name)
