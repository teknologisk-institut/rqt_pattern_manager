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

from PyQt5.QtWidgets import \
    QWidget, \
    QPushButton, \
    QComboBox, \
    QTreeView, \
    QMenu, \
    QAction, \
    QApplication, \
    QLineEdit, \
    QCheckBox
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

        self.param_model.dataChanged.connect(self._update_transform_variable)
        self.param_model.dataChanged.connect(self.tree_model.update)
        self.param_model.dataChanged.connect(self.treeView.expandAll)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._context_menu)

        self.magicButton.clicked.connect(self._magic)
        self.quitButton.clicked.connect(QApplication.quit)

    def _magic(self):
        self.setAutoFillBackground(True)
        colors = [Qt.black, Qt.red, Qt.green, Qt.gray, Qt.yellow, Qt.blue, Qt.white]
        p = self.palette()
        p.setColor(self.backgroundRole(), random.choice(colors))
        self.setPalette(p)

    def _update_transform_variable(self):
        table_item = self.get_cur_selection(self.parameterView)
        tree_item = self.get_cur_selection(self.treeView)

        if not table_item or not tree_item:
            return

        item_header = self.param_model.verticalHeaderItem(table_item.row()).text()
        pmc.update_transform_var(tree_item.data()['id'], str(item_header), str(table_item.text()))

    def _context_menu(self, position):
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
            self.wdg = CreateWidget(cur_selection)
            self.wdg.show()

            self.wdg.tfCreated.connect(self.tree_model.update)
            self.wdg.tfCreated.connect(self.treeView.expandAll)

            return
        elif action == ac_remove:
            pmc.remove_transform(cur_selection.data()['id'])
            cur_selection.parent().removeRow(cur_selection.row())
        elif action == ac_set_actv:
            pmc.set_active(cur_selection.data()['id'], True)
        elif action == ac_set_inactive:
            pmc.set_active(cur_selection.data()['id'], False)
        else:
            return

        self.tree_model.update()
        self.treeView.expandAll()

    @staticmethod
    def get_cur_selection(view):
        selection_model = view.selectionModel()
        index = selection_model.currentIndex()
        cur_selection = view.model().itemFromIndex(index)

        return cur_selection


class CreateWidget(QWidget):

    tfCreated = pyqtSignal()

    def __init__(self, parent):
        super(CreateWidget, self).__init__()

        self.parent = parent

        load_ui('create.ui', self)
        self.setObjectName('CreateWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self.patternWidget.close()

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

        self.typeBox.currentIndexChanged.connect(self._on_type_index_changed)
        self.patternBox.currentIndexChanged.connect(self._on_pattern_index_changed)

    def _on_type_index_changed(self):
        wdgs = [
            self.patternWidget,
            self.transformWidget
        ]

        for w in wdgs:
            w.close()

        if self.typeBox.currentText() == 'Transform':
            self.transformWidget.show()
        elif self.typeBox.currentText() == 'Pattern':
            self.patternWidget.show()
            self._on_pattern_index_changed()

    def _on_pattern_index_changed(self):
        wdgs = [
            self.linearWidget,
            self.rectangularWidget,
            self.circularWidget,
            self.scatterWidget
        ]

        for w in wdgs:
            w.close()

        if self.patternBox.currentText() == 'Linear':
            self.linearWidget.show()
        elif self.patternBox.currentText() == 'Rectangular':
            self.rectangularWidget.show()
        elif self.patternBox.currentText() == 'Circular':
            self.circularWidget.show()
        elif self.patternBox.currentText() == 'Scatter':
            self.scatterWidget.show()

    def _on_accepted(self):
        pmc.create_transform(self.nameText.text(), self.parent.data()['id'])

        self.tfCreated.emit()
        self._on_rejected()

    def _on_rejected(self):
        wgds = self.findChildren(QLineEdit)
        wgds.extend(self.findChildren(QComboBox))

        for w in wgds:
            w.clear()

        for c in self.findChildren(QCheckBox):
            c.setChecked(False)

        self.close()


class TreeItemModel(QStandardItemModel):

    def __init__(self):
        super(TreeItemModel, self).__init__()

        self.tree = {}
        self.items = {}
        self.params = {
            'ids': {},
            'ref_frames': {},
            'active': {},
            'translation': {},
            'rotation': {}
        }

        self.update()

    def update(self):
        self._update_tree(self.params)
        self._update_model(self.tree, self.invisibleRootItem())

    def _update_tree(self, params):
        nested_param_lists = pmc.get_transforms()

        nodes = {}
        for i in nested_param_lists:
            name = i.name
            nodes[name] = {}
            params['ids'][name] = i.id
            params['ref_frames'][name] = i.ref_frame
            params['active'][name] = i.active
            params['translation'][name] = i.translation
            params['rotation'][name] = i.rotation

        self.tree = param_tree_from_nested_lists(nested_param_lists, nodes)

    def _update_model(self, tree=None, parent=None):

        if not tree:
            tree = self.tree

        if not parent:
            parent = self.invisibleRootItem()

        for child in sorted(tree):
            child_item = find_item_child_by_id(parent, self.params['ids'][child])

            if not child_item:
                child_item = QStandardItem()
                self.items[self.params['ids'][child]] = child_item
                parent.appendRow(child_item)

            child_item.setText(child)
            child_item.setData(
                {
                    'name': child,
                    'id': self.params['ids'][child],
                    'ref_frame': self.params['ref_frames'][child],
                    'active': self.params['active'][child],
                    'translation': self.params['translation'][child],
                    'rotation': self.params['rotation'][child]
                })
            child_item.setEditable(False)

            if self.params['active'][child]:
                child_item.setForeground(QBrush(QColor(Qt.green)))
            else:
                child_item.setForeground(QBrush(QColor(Qt.black)))

            if tree[child]:
                self._update_model(tree[child], child_item)


class TableItemModel(QStandardItemModel):

    def __init__(self):
        super(TableItemModel, self).__init__()

        self.locked = ['id', 'active']

    def update(self, parent):

        if not parent:
            return

        self.clear()
        self.setColumnCount(1)

        def add_item(header, item, row, editable=False):
            h = QStandardItem(header)
            t = QStandardItem(str(item))
            t.setEditable(editable)

            self.setVerticalHeaderItem(row, h)
            self.setItem(row, 0, t)

        i = 0
        for k, v in parent.data().items():
            edit = False if k in self.locked else True
            add_item(k, v, i, edit)

            i += 1
