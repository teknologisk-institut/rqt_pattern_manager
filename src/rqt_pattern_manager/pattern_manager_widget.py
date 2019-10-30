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

import pattern_manager_client as pmc

from PyQt5.QtWidgets import \
    QWidget, \
    QComboBox, \
    QMenu, \
    QApplication, \
    QLineEdit, \
    QCheckBox
from PyQt5.QtGui import QStandardItem, QStandardItemModel, QBrush, QColor, QPainter, QPen, QIcon
from PyQt5.QtCore import pyqtSignal, Qt, QRect
from .util import *
from pprint import pprint


class MainWidget(QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()

        load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')
        self.setBaseSize(self.minimumSize())

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

        self.param_model.dataChanged.connect(self._on_param_data_changed)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._context_menu)
        self.treeView.doubleClicked.connect(self._on_tree_double_clicked)

        self.refreshButton.clicked.connect(self._on_refresh_clicked)
        self.quitButton.clicked.connect(QApplication.quit)
        self.stepButton.clicked.connect(self._on_iterate)
        self.activeButton.clicked.connect(lambda: self._call_action(
            "toggle_activate_item",
            self.get_cur_selection(self.treeView)
        ))
        self.addButton.clicked.connect(lambda: self._call_action(
            "open_create_widget",
            self.get_cur_selection(self.treeView)
        ))
        self.removeButton.clicked.connect(lambda: self._call_action(
            "remove_item",
            self.get_cur_selection(self.treeView)
        ))
        self.resetButton.clicked.connect(self._on_reset_all)

        self.init_cnt_actv = len(pmc.get_active_ids())
        if self.init_cnt_actv > 0:
            self.progressBar.setValue(100)

    def _on_refresh_clicked(self):
        self.tree_model.update()
        self.treeView.expandAll()

    def _on_tree_double_clicked(self):
        item = self.get_cur_selection(self.treeView)
        self._call_action('toggle_activate_item', item)

    def _on_reset_all(self):
        actv_ids = pmc.get_active_ids()

        for id_ in actv_ids:
            pmc.set_active(id_, False)

        self.progressBar.setValue(0)
        self.tree_model.update()

    def _on_iterate(self):
        cnt_actv = len(pmc.get_active_ids())

        if cnt_actv == 0:
            return

        if cnt_actv > 0:
            self.progressBar.setValue(int((float(cnt_actv) - 1.0) / float(self.init_cnt_actv) * 100.0))

        pmc.iterate()

        self.tree_model.update()

    def _on_param_data_changed(self):
        selection_model = self.parameterView.selectionModel()

        if selection_model.hasSelection():
            self._update_transform_variable()
            self.tree_model.update()
            self.treeView.expandAll()

    def _update_transform_variable(self):
        table_item = self.get_cur_selection(self.parameterView)
        tree_item = self.get_cur_selection(self.treeView)

        if not table_item or not tree_item:
            return

        item_header = self.param_model.verticalHeaderItem(table_item.row()).text()
        pmc.update_transform_var(tree_item.data()['id'], str(item_header), str(table_item.text()))

    def _context_menu(self, position):
        cur_selection = self.get_cur_selection(self.treeView)

        menu = QMenu(self)
        ac_new_tf = menu.addAction("Add...")
        ac_remove = menu.addAction("Remove")

        menu.addSeparator()

        ac_set_actv = menu.addAction("Active")

        action = menu.exec_(self.treeView.mapToGlobal(position))
        if action == ac_new_tf:
            self._call_action("open_create_widget", cur_selection)
        elif action == ac_remove:
            self._call_action("remove_item", cur_selection)
        elif action == ac_set_actv:
            self._call_action("toggle_activate_item", cur_selection)
        else:
            return

    def _call_action(self, action, item):
        param_changed = False
        node_changed = False

        if not item:
            return

        if action == "open_create_widget":
            self.wdg = CreateWidget(item)
            self.wdg.show()

            self.wdg.tfCreated.connect(self.tree_model.update)
            self.wdg.tfCreated.connect(self.treeView.expandAll)

            return
        elif action == "remove_item":
            pmc.remove_transform(item.data()['id'])
            item.parent().removeRow(item.row())

            node_changed = True
        elif action == "toggle_activate_item":
            if item.data()['active']:
                pmc.set_active(item.data()['id'], False)
            else:
                pmc.set_active(item.data()['id'], True)

            self.init_cnt_actv = len(pmc.get_active_ids())
            self.progressBar.setValue(100)

            node_changed = True
            param_changed = True
        else:
            return

        if node_changed:
            self.tree_model.update()
            self.treeView.expandAll()

        if param_changed:
            self.param_model.update(item)
            self.parameterView.resizeColumnsToContents()

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
        self.type_ = None

        load_ui('create.ui', self)
        self.setObjectName('CreateWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.parentText.setText(parent.text())

        self.patternWidget.close()

        self.adjustSize()

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

        self.typeBox.currentIndexChanged.connect(self._on_type_index_changed)
        self.patternBox.currentIndexChanged.connect(self._on_pattern_index_changed)

        for w in self.linearWidget.findChildren(QLineEdit):
            w.textChanged.connect(self._on_text_changed)

        for w in self.rectangularWidget.findChildren(QLineEdit):
            w.textChanged.connect(self._on_text_changed)

    def _on_text_changed(self):
        les = self.focusWidget().parentWidget().findChildren(QLineEdit)

        count = 0
        for le in les:
            if not len(le.text()) == 0:
                count += 1

        if count > 1:
            for le in les:
                if len(le.text()) == 0:
                    le.setEnabled(False)
        else:
            for le in les:
                le.setEnabled(True)

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

            return

        self.adjustSize()

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

        self.adjustSize()

    def _on_accepted(self):

        if self.typeBox.currentText() == 'Transform':
            pmc.create_transform(self.nameText.text(), self.parent.data()['id'], self.referenceText.text())
        elif self.typeBox.currentText() == 'Pattern':

            if self.patternBox.currentText() == 'Linear':
                args = [
                    str(self.nameText.text()),
                    str(self.numPointsText.text()),
                    str(self.stepSizeText.text()),
                    str(self.lengthText.text()),
                    self.parent.data()['id']
                ]
                pmc.create_linear_pattern(*args)
            elif self.patternBox.currentText() == 'Rectangular':
                args = [
                    str(self.nameText.text()),
                    str(self.numPointsText_2.text()),
                    str(self.stepSizesText.text()),
                    str(self.lengthsText.text()),
                    self.parent.data()['id']
                ]
                pmc.create_rectangular_pattern(*args)

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
            params['translation'][name] = [i.translation.x, i.translation.y, i.translation.z]
            params['rotation'][name] = [i.rotation.x, i.rotation.y, i.rotation.z, i.rotation.w]

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
            self._assign_item_color(child_item)

            if tree[child]:
                self._update_model(tree[child], child_item)

    def _assign_item_color(self, item):
        cur_tf_id = pmc.get_current_tf_id()

        def set_italic(it):
            f = item.font()
            f.setItalic(it)
            item.setFont(f)

        if cur_tf_id == item.data()['id']:
            if not item.font().italic():
                set_italic(True)
        else:
            if item.font().italic():
                set_italic(False)

        if self.params['active'][item.text()]:
            color = Qt.white
        else:
            color = Qt.gray

        item.setForeground(QBrush(QColor(color)))


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
