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

import random
import pattern_manager_client as pmc

from PyQt5.QtWidgets import QWidget, QPushButton, QComboBox, QTreeView, QMenu, QAction, QApplication
from PyQt5.QtGui import QStandardItem, QStandardItemModel, QBrush, QColor
from PyQt5.QtCore import QModelIndex, pyqtSignal, Qt
from .utils import *

from pprint import pprint


class MainWidget(QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()

        load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        self.tree_model = TreeItemModel()
        self.treeView.setModel(self.tree_model)
        self.treeView.setHeaderHidden(True)
        self.treeView.expandAll()

        self.param_model = TableItemModel()
        self.parameterView.setModel(self.param_model)
        self.parameterView.horizontalHeader().hide()

        selection_model = self.treeView.selectionModel()
        selection_model.selectionChanged.connect(lambda: self.param_model.update(self.get_cur_selection(self.treeView)))
        selection_model.selectionChanged.connect(self.parameterView.resizeColumnsToContents)

        self.param_model.dataChanged.connect(lambda: self.param_model.on_data_changed(
            self.get_cur_selection(self.parameterView),
            self.get_cur_selection(self.treeView)))

        self.param_model.cellUpdated.connect(self.tree_model.update)
        self.param_model.cellUpdated.connect(self.treeView.expandAll)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._show_context_menu)

        self.magicButton.clicked.connect(self.magic)
        self.quitButton.clicked.connect(QApplication.quit)

    def magic(self):
        self.setAutoFillBackground(True)
        colors = [Qt.black, Qt.red, Qt.green, Qt.gray, Qt.yellow, Qt.blue]
        p = self.palette()
        p.setColor(self.backgroundRole(), random.choice(colors))
        self.setPalette(p)

    @staticmethod
    def get_cur_selection(view):
        selection_model = view.selectionModel()
        index = selection_model.currentIndex()
        cur_selection = view.model().itemFromIndex(index)

        return cur_selection

    def _show_context_menu(self, position):
        cur_selection = self.get_cur_selection(self.treeView)

        if not cur_selection:
            return

        menu = QMenu()
        ac_new_tf = menu.addAction("Add...")
        ac_remove = menu.addAction("Remove")
        menu.addSeparator()
        ac_set_actv = menu.addAction("Set Active")
        ac_set_inactive = menu.addAction("Set Inactive")

        action = menu.exec_(self.treeView.mapToGlobal(position))

        if action == ac_new_tf:
            self.wdg = CreateWidget()
            self.wdg.show()
        elif action == ac_remove:
            pmc.remove_transform(cur_selection.data()[0][1])
        elif action == ac_set_actv:
            pmc.set_active(cur_selection.data()[0][1], True)
        elif action == ac_set_inactive:
            pmc.set_active(cur_selection.data()[0][1], False)
        else:
            return

        self.tree_model.update()
        self.treeView.expandAll()


class CreateWidget(QWidget):

    def __init__(self):
        super(CreateWidget, self).__init__()

        load_ui('create.ui', self)
        self.setObjectName('CreateWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

    def _on_accepted(self):
        # TODO: create transform via pmc
        self.close()

    def _on_rejected(self):
        self.nameText.clear()
        self.referenceText.clear()
        self.xText.clear()
        self.yText.clear()
        self.zText.clear()
        self.qxText.clear()
        self.qyText.clear()
        self.qzText.clear()
        self.qwText.clear()

        self.close()


class TreeItemModel(QStandardItemModel):

    def __init__(self):
        super(TreeItemModel, self).__init__()

        self.update()

    def update(self):
        self.clear()

        nested_param_lists = pmc.get_transforms()

        params = {
            'ids': {},
            'ref_frames': {},
            'active': {},
            'translation': {},
            'rotation': {}
        }

        nodes = {}
        for i in nested_param_lists:
            name = i.name
            nodes[name] = {}
            params['ids'][name] = i.id
            params['ref_frames'][name] = i.ref_frame
            params['active'][name] = i.active
            params['translation'][name] = [i.tf.translation.x, i.tf.translation.y, i.tf.translation.z]
            params['rotation'][name] = [i.tf.rotation.x, i.tf.rotation.y, i.tf.rotation.z, i.tf.rotation.w]

        tree = param_tree_from_nested_lists(nested_param_lists, nodes)
        self._build_model(tree, self.invisibleRootItem(), params)

    def _build_model(self, children, parent, params):
        for child in sorted(children):
            child_item = QStandardItem(child)
            child_item.setData(
                [
                    ('id', params['ids'][child]),
                    ('ref_frame', params['ref_frames'][child]),
                    ('active', params['active'][child]),
                    ('translation', params['translation'][child]),
                    ('rotation', params['rotation'][child])
                ])
            child_item.setEditable(False)

            if params['active'][child]:
                child_item.setForeground(QBrush(QColor('green')))

            parent.appendRow(child_item)
            self._build_model(children[child], child_item, params)


class TableItemModel(QStandardItemModel):

    cellUpdated = pyqtSignal()

    def __init__(self):
        super(TableItemModel, self).__init__()

    def on_data_changed(self, item, parent):
        if not item or not parent:
            return

        item_header = self.verticalHeaderItem(item.row()).text().lower()
        pmc.update_transform_var(parent.data()[0][1], item_header, item.text())

        self.cellUpdated.emit()

    def update(self, parent):
        self.clear()
        self.setColumnCount(1)

        def add_item(str_header, str_item, row, editable=False):
            h = QStandardItem(str_header)
            t = QStandardItem(str_item)
            t.setEditable(editable)

            self.setVerticalHeaderItem(row, h)
            self.setItem(row, 0, t)

        add_item('name', str(parent.text()), 0, True)
        add_item(str(parent.data()[2][0]), str(parent.data()[2][1]), 1)
        add_item(str(parent.data()[0][0]), str(parent.data()[0][1]), 2)
        add_item(str(parent.data()[1][0]), str(parent.data()[1][1]), 3, True)
        add_item(str(parent.data()[3][0]), str(parent.data()[3][1]), 4, True)
        add_item(str(parent.data()[4][0]), str(parent.data()[4][1]), 5, True)
