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

from pattern_manager_client import *

from PyQt5.QtWidgets import QWidget, QPushButton, QComboBox, QTreeView, QMenu, QAction, QApplication
from PyQt5.QtGui import QStandardItem, QStandardItemModel, QBrush, QColor
from PyQt5.QtCore import QModelIndex, Qt
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

        self.table_model = TableItemModel()
        self.parameterView.setModel(self.table_model)
        self.parameterView.horizontalHeader().hide()

        selection_model = self.treeView.selectionModel()

        def update_model():
            self.table_model.update_model(self.get_cur_selection(self.treeView))
            self.parameterView.resizeColumnsToContents()

        selection_model.selectionChanged.connect(update_model)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._show_context_menu)

        self.quitButton.clicked.connect(QApplication.quit)

    @staticmethod
    def get_cur_selection(view):
        selection_model = view.selectionModel()
        index = selection_model.currentIndex()
        cur_selection = selection_model.model().itemFromIndex(index)

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

        action = menu.exec_(self.treeView.mapToGlobal(position))

        if action == ac_new_tf:
            # self.wdg = NewTransformWidget()
            # self.wdg.show()
            # self.wdg.destroyed.connect(self.tree_model.update_model)
            # self.wdg.destroyed.connect(self.treeView.expandAll)

            return
        elif action == ac_remove:
            remove_transform(cur_selection.data()[0])
        elif action == ac_set_actv:
            set_active(cur_selection.data()[0])
        else:
            return

        self.tree_model.update_model()
        self.treeView.expandAll()


class TreeItemModel(QStandardItemModel):

    def __init__(self):
        super(TreeItemModel, self).__init__()

        self.update_model()

    def update_model(self):
        self.clear()

        nested_param_lists = get_transforms()

        params = {
            'ids': {},
            'ref_frames': {},
            'active': {}
        }

        nodes = {}
        for i in nested_param_lists:
            name = i.name
            nodes[name] = {}
            params['ids'][name] = i.id
            params['ref_frames'][name] = i.ref_frame
            params['active'][name] = i.active

        tree = param_tree_from_nested_lists(nested_param_lists, nodes)
        self._build_model(tree, self.invisibleRootItem(), params)

    def _build_model(self, children, parent, params):
        for child in sorted(children):
            child_item = QStandardItem(child)
            child_item.setData(
                [
                    params['ids'][child],
                    params['ref_frames'][child],
                    params['active'][child]
                ])
            child_item.setEditable(False)

            if params['active'][child]:
                child_item.setForeground(QBrush(QColor('green')))

            parent.appendRow(child_item)
            self._build_model(children[child], child_item, params)


class TableItemModel(QStandardItemModel):

    def __init__(self):
        super(TableItemModel, self).__init__()

    def update_model(self, cur_selection):
        self.clear()
        self.setColumnCount(1)

        def add_item(str_header, str_item, row):
            h = QStandardItem(str_header)
            t = QStandardItem(str_item)
            t.setEditable(False)

            self.setVerticalHeaderItem(row, h)
            self.setItem(row, 0, t)

        add_item('Name', str(cur_selection.text()), 0)
        add_item('Active', str(cur_selection.data()[2]), 1)
        add_item('ID', str(cur_selection.data()[0]), 2)
        add_item('Reference Frame', str(cur_selection.data()[1]), 3)


# class MainWidget(QWidget):
#
#     def __init__(self):
#         super(MainWidget, self).__init__()
#
#         load_ui('pattern_manager_widget.ui', self)
#         self.setObjectName('PatternManagerWidget')
#
#         self.tree_model = TreeItemModel()
#         self.treeView.setModel(self.tree_model)
#         self.treeView.setHeaderHidden(True)
#         self.treeView.expandAll()
#
#         self.table_model = TableItemModel()
#         self.parameterView.setModel(self.table_model)
#         self.parameterView.horizontalHeader().hide()
#         self.parameterView.setWordWrap(True)
#
#         selection_model = self.treeView.selectionModel()
#
#         def update_model():
#             self.table_model.update_model(self.get_cur_selection(self.treeView))
#             self.parameterView.resizeColumnsToContents()
#
#         selection_model.selectionChanged.connect(update_model)
#
#         self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
#         self.treeView.customContextMenuRequested.connect(self._show_context_menu)
#
#         self.quitButton.clicked.connect(QApplication.quit)
#
#     @staticmethod
#     def get_cur_selection(view):
#         selection_model = view.selectionModel()
#         index = selection_model.currentIndex()
#         cur_selection = selection_model.model().itemFromIndex(index)
#
#         return cur_selection
#
#     def _show_context_menu(self, position):
#         cur_selection = self.get_cur_selection(self.treeView)
#
#         if not cur_selection:
#             return
#
#         menu = QMenu()
#         ac_set_actv = menu.addAction("Set Active")
#         menu.addSeparator()
#         ac_new_grp = menu.addAction("Add Node..")
#         ac_new_pat = menu.addAction("Add Pattern..")
#         menu.addSeparator()
#         ac_remove = menu.addAction("Remove")
#
#         if cur_selection.data()[1] == 'Pattern':
#             ac_new_pat.setEnabled(False)
#             ac_new_grp.setEnabled(False)
#         elif cur_selection.data()[1] == 'Tree' and cur_selection.hasChildren():
#             ac_set_actv.setEnabled(False)
#
#             if cur_selection.child(0, 0).data()[1] == 'Tree':
#                 ac_new_pat.setEnabled(False)
#             elif cur_selection.child(0, 0).data()[1] == 'Pattern':
#                 ac_new_grp.setEnabled(False)
#
#         action = menu.exec_(self.treeView.mapToGlobal(position))
#
#         self.wdg = None
#         if action == ac_new_pat:
#             self.wdg = NewPatternWidget(cur_selection.data()[0])
#         elif action == ac_new_grp:
#             self.wdg = NewGroupWidget(cur_selection.data()[0])
#         elif action == ac_remove:
#             if cur_selection.data()[1] == "Tree":
#                 remove_group(cur_selection.data()[0])
#             elif cur_selection.data()[1] == "Pattern":
#                 remove_pattern(cur_selection.data()[0])
#         elif action == ac_set_actv:
#             print 'Setting active'
#         else:
#             return
#
#         if self.wdg:
#             self.wdg.show()
#             self.wdg.destroyed.connect(self.tree_model.update_model)
#             self.wdg.destroyed.connect(self.treeView.expandAll)
#
#             return
#
#         self.tree_model.update_model()
#         self.treeView.expandAll()
#
#
# class NewGroupWidget(QWidget):
#
#     def __init__(self, par_id):
#         super(NewGroupWidget, self).__init__()
#
#         self.par_id = par_id
#
#         load_ui('new_group.ui', self)
#         self.setObjectName('NewGroupWidget')
#         self.setAttribute(Qt.WA_DeleteOnClose, True)
#
#         self.dialogButton.accepted.connect(self._on_accepted)
#         self.dialogButton.rejected.connect(self._on_rejected)
#
#     def _on_accepted(self):
#         create_group(self.par_id, self.nameBox.text())
#
#         self.close()
#
#     def _on_rejected(self):
#         self.groupBox.clear()
#         self.nameBox.clear()
#         self.close()
#
#
# class NewPatternWidget(QWidget):
#
#     def __init__(self, grp_id):
#         super(NewPatternWidget, self).__init__()
#
#         load_ui('new_pattern.ui', self)
#         self.setObjectName('NewPatternWidget')
#         self.setAttribute(Qt.WA_DeleteOnClose, True)
#
#         self.group_id = grp_id
#
#         self._populate_pattern_view()
#
#         self.dialogButton.accepted.connect(self._on_accepted)
#         self.dialogButton.rejected.connect(self._on_rejected)
#
#     def _populate_pattern_view(self):
#         model = QStandardItemModel(self.patternView)
#
#         typs = get_pattern_types()
#         for t in typs:
#             item = QStandardItem(t)
#             model.appendRow(item)
#
#         self.patternView.setModel(model)
#
#     def _on_accepted(self):
#         selection = MainWidget.get_cur_selection(self.patternView)
#         pat_typ = selection.text().encode('ascii', 'ignore')
#         pat_nm = self.nameBox.text().encode('ascii', 'ignore')
#
#         create_pattern(pat_typ, pat_nm, self.group_id)
#
#         self.close()
#
#     def _on_rejected(self):
#         self.nameBox.clear()
#         self.close()
#
#
# class TreeItemModel(QStandardItemModel):
#
#     def __init__(self):
#         super(TreeItemModel, self).__init__()
#
#         self.update_model()
#
#     def update_model(self):
#         self.clear()
#
#         nested_param_lists = get_groups() + get_patterns()
#
#         params = {
#             'types': {},
#             'ids': {},
#             'active': {}
#         }
#
#         nodes = {}
#         for i in nested_param_lists:
#             name = i.name
#             nodes[name] = {}
#             params['types'][name] = i.type
#             params['ids'][name] = i.id
#             params['active'][name] = i.active
#
#         tree = param_tree_from_nested_lists(nested_param_lists, nodes)
#         self._build_model(tree, self.invisibleRootItem(), params)
#
#     def _build_model(self, children, parent, params):
#         for child in sorted(children):
#             child_item = QStandardItem(child)
#             child_item.setData(
#                 [
#                     params['ids'][child],
#                     params['types'][child],
#                     params['active'][child]
#                 ]
#             )
#             child_item.setEditable(False)
#
#             if params['active'][child]:
#                 child_item.setForeground(QBrush(QColor('green')))
#
#             parent.appendRow(child_item)
#             self._build_model(children[child], child_item, params)
#
#
# class TableItemModel(QStandardItemModel):
#
#     def __init__(self):
#         super(TableItemModel, self).__init__()
#
#     def update_model(self, cur_selection):
#         self.clear()
#         self.setColumnCount(1)
#
#         def add_item(str_header, str_item, row):
#             h = QStandardItem(str_header)
#             t = QStandardItem(str_item)
#             t.setEditable(False)
#
#             self.setVerticalHeaderItem(row, h)
#             self.setItem(row, 0, t)
#
#         add_item('Name', str(cur_selection.text()), 0)
#         add_item('Active', str(cur_selection.data()[2]), 1)
#         add_item('ID', str(cur_selection.data()[0]), 2)
#         add_item('Type', str(cur_selection.data()[1]), 3)
#
#         if cur_selection.data()[1] == 'Pattern':
#             add_item('Pattern Type', str(get_pattern_type(cur_selection.data()[0])), 4)
