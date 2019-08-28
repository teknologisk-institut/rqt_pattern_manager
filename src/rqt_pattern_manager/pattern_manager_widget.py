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

from PyQt5.QtWidgets import QWidget, QDockWidget, QFrame, QPushButton, QComboBox, QTreeView, QMenu, qApp, QAction
from PyQt5.QtGui import QStandardItem, QStandardItemModel
from PyQt5.QtCore import QModelIndex, Qt
from .utils import Utils


_pmc = pmc.PatternManagerClient()


class NewGroupWidget(QWidget):

    def __init__(self):
        super(NewGroupWidget, self).__init__()

        Utils.load_ui('new_group.ui', self)
        self.setObjectName('NewGroupWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

    def _on_accepted(self):
        srv_create_grp = _pmc.get_service('pattern_manager/create_group')
        srv_create_grp.call_service(self.groupBox.currentText(), self.nameBox.text())

        self.close()

    def _on_rejected(self):
        self.groupBox.clear()
        self.nameBox.clear()
        self.close()


class NewPatternWidget(QWidget):

    def __init__(self, grp_id):
        super(NewPatternWidget, self).__init__()

        Utils.load_ui('new_pattern.ui', self)
        self.setObjectName('NewPatternWidget')
        self.setAttribute(Qt.WA_DeleteOnClose, True)

        self.group_id = grp_id

        self._populate_pattern_view()

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

    def _populate_pattern_view(self):
        model = QStandardItemModel(self.patternView)

        srv_get_pat_types = _pmc.get_service('pattern_manager/get_pattern_types')

        resp = srv_get_pat_types.call_service()
        for p in resp.pattern_types:
            item = QStandardItem(p)
            model.appendRow(item)

        self.patternView.setModel(model)

    def _on_accepted(self):
        selection = MainWidget.get_cur_selection(self.patternView)
        pat_typ = selection.text().encode('ascii', 'ignore')
        pat_nm = self.nameBox.text().encode('ascii', 'ignore')

        srv_create_pat = _pmc.get_service('pattern_manager/create_pattern')
        srv_create_pat.call_service(pat_typ, pat_nm, self.group_id)

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

        srv_get_grps = _pmc.get_service('pattern_manager/get_groups')
        srv_get_pats = _pmc.get_service('pattern_manager/get_patterns')

        resp_grp = srv_get_grps.call_service()
        resp_pat = srv_get_pats.call_service()

        deps = resp_grp.group_deps + resp_pat.group_deps

        nodes = {}
        types = {}
        ids = {}
        for i in deps:
            name, par_name = i.name_and_parent
            nodes[name] = {}
            types[name] = i.type
            ids[name] = i.id

        tree = Utils.deps_tree_from_nested_lists(deps, nodes)
        self._build_model(tree, self.invisibleRootItem(), types, ids)

    def _build_model(self, children, parent, typs, ids):
        for child in sorted(children):
            child_item = QStandardItem(child)
            child_item.setWhatsThis(typs[child])
            child_item.setData(ids[child])
            child_item.setEditable(False)
            parent.appendRow(child_item)
            self._build_model(children[child], child_item, typs, ids)


class TableItemModel(QStandardItemModel):

    def __init__(self):
        super(TableItemModel, self).__init__()

    def update_model(self, cur_selection):
        self.clear()

        self.setColumnCount(1)

        name_header = QStandardItem('Name')
        name = QStandardItem(cur_selection.text())
        name.setEditable(False)

        self.setVerticalHeaderItem(0, name_header)
        self.setItem(0, 0, name)

        typ_header = QStandardItem('Type')
        typ = QStandardItem(cur_selection.whatsThis())
        typ.setEditable(False)

        self.setVerticalHeaderItem(1, typ_header)
        self.setItem(1, 0, typ)

        id_header = QStandardItem('Id')
        id = QStandardItem(str(cur_selection.data()))
        id.setEditable(False)

        self.setVerticalHeaderItem(2, id_header)
        self.setItem(2, 0, id)


class MainWidget(QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()

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
            self.table_model.update_model(self.get_cur_selection(self.treeView))

        selection_model.selectionChanged.connect(update_model)

        self.treeView.setContextMenuPolicy(Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self._show_context_menu)

    @staticmethod
    def get_cur_selection(view):
        selection_model = view.selectionModel()
        index = selection_model.currentIndex()
        cur_selection = selection_model.model().itemFromIndex(index)

        return cur_selection

    def _show_context_menu(self, position):
        menu = QMenu()
        ac_new_pat = menu.addAction("Add Pattern..")
        ac_new_grp = menu.addAction("Add Group..")

        cur_selection = self.get_cur_selection(self.treeView)

        if not cur_selection.whatsThis() == 'Group':
            ac_new_pat.setEnabled(False)
            ac_new_grp.setEnabled(False)

        action = menu.exec_(self.treeView.mapToGlobal(position))

        self.wdg = None
        if action == ac_new_pat:
            self.wdg = NewPatternWidget(cur_selection.data())
        elif action == ac_new_grp:
            self.wdg = NewGroupWidget()
        else:
            return

        self.wdg.show()
        self.wdg.destroyed.connect(self.tree_model.update_model)
