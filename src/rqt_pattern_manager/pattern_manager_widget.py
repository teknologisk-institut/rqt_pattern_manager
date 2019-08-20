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

from PyQt5.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton, QComboBox, QTreeView, QMenu, qApp, QAction
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtCore import QModelIndex, Qt
from .utils import Utils
from .pattern_manager_client import PatternManagerClient as PMC


class NewGroupWidget(QWidget):
    def __init__(self):
        super(NewGroupWidget, self).__init__()

        Utils.load_ui('new_group.ui', self)
        self.setObjectName('NewGroupWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

    def _on_accepted(self):
        PMC.create_group(self.groupBox.currentText(), self.nameBox.text())
        self.close()

    def _on_rejected(self):
        self.groupBox.clear()
        self.nameBox.clear()
        self.close()


class NewPatternWidget(QWidget):
    def __init__(self):
        super(NewPatternWidget, self).__init__()

        Utils.load_ui('new_pattern.ui', self)
        self.setObjectName('NewPatternWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self._populate_pattern_view()

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

    def _populate_pattern_view(self):
        model = QStandardItemModel(self.patternView)

        pats = PMC.get_pattern_types()
        for p in pats:
            item = QStandardItem(p)
            model.appendRow(item)

        self.patternView.setModel(model)

    def _on_accepted(self):
        PMC.create_pattern(self.patternView.currentItem(), self.nameBox.text(), self.groupView.currentItem())
        self.close()

    def _on_rejected(self):
        self.nameBox.clear()
        self.close()


class TreeItemModel(QStandardItemModel):
    def __init__(self):
        super(TreeItemModel, self).__init__()

        self.update_model()

    def update_model(self):
        self.clear()

        grp_deps = PMC.get_groups()
        pat_deps = PMC.get_patterns()
        deps = grp_deps + pat_deps

        nodes = {}
        types = {}
        for i in deps:
            name, par_name = i.name_and_parent
            nodes[name] = {}
            types[name] = i.type

        tree = Utils.deps_tree_from_nested_lists(deps, nodes)
        self._build_model(tree, self.invisibleRootItem(), types)

    def _build_model(self, children, parent, typs):
        for child in sorted(children):
            child_item = QStandardItem(child)
            child_item.setWhatsThis(typs[child])
            child_item.setEditable(False)
            parent.appendRow(child_item)
            self._build_model(children[child], child_item, typs)


class TableItemModel(QStandardItemModel):
    def __init__(self):
        super(TableItemModel, self).__init__()

    def update_model(self, cur_selection):
        self.clear()

        self.setColumnCount(1)
        self.setRowCount(2)

        type_header = QStandardItem('Type')
        type = QStandardItem(cur_selection.whatsThis())
        type.setEditable(False)

        self.setVerticalHeaderItem(0, type_header)
        self.setItem(0, 0, type)

        name_header = QStandardItem('Name')
        name = QStandardItem(cur_selection.text())
        name.setEditable(False)

        self.setVerticalHeaderItem(1, name_header)
        self.setItem(1, 0, name)


class PatternManagerWidget(QWidget):
    def __init__(self):
        super(PatternManagerWidget, self).__init__()

        Utils.load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')

        self.tree_model = TreeItemModel()
        self.treeView.setModel(self.tree_model)
        self.treeView.setHeaderHidden(True)
        self.treeView.expandAll()

        self.table_model = TableItemModel()
        self.parameterView.setModel(self.table_model)
        self.parameterView.horizontalHeader().hide()

        selection_model = self.treeView.selectionModel()

        def update_model():
            self.table_model.update_model(self._get_cur_selection(self.treeView))

        selection_model.selectionChanged.connect(update_model)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._show_context_menu)

    def _get_cur_selection(self, view):
        selection_model = view.selectionModel()
        index = selection_model.currentIndex()
        cur_selection = selection_model.model().itemFromIndex(index)

        return cur_selection

    def _show_context_menu(self, position):
        menu = QMenu()
        a_new_pat = menu.addAction("Add Pattern..")
        a_new_grp = menu.addAction("Add Group..")

        if not self._get_cur_selection(self.treeView).whatsThis() == 'Group':
            a_new_pat.setEnabled(False)
            a_new_grp.setEnabled(False)

        action = menu.exec_(self.treeView.mapToGlobal(position))

        if action == a_new_pat:
            self.wdg_new_pat = NewPatternWidget()
            self.wdg_new_pat.show()
            self.wdg_new_pat.destroyed.connect(self.tree_model.update_model)
        elif action == a_new_grp:
            self.wdg_new_grp = NewGroupWidget()
            self.wdg_new_grp.show()
            self.wdg_new_grp.destroyed.connect(self.tree_model.update_model)
