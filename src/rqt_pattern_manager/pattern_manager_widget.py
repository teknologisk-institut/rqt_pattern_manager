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
import resources
import math
import string

from PyQt5.QtWidgets import \
    QWidget, \
    QComboBox, \
    QMenu, \
    QApplication, \
    QLineEdit, \
    QCheckBox, \
    QLayout, \
    QTreeView, \
    QAbstractItemView, \
    QTableView, \
    QSpacerItem, \
    QProxyStyle, \
    QFileDialog
from PyQt5.QtGui import QStandardItem, QStandardItemModel, QBrush, QColor, QPen, QDropEvent, QDragMoveEvent
from PyQt5.QtCore import pyqtSignal, Qt, QAbstractListModel, QCoreApplication
from rqt_pattern_manager.util import *
from collections import OrderedDict


class CustomTreeItemModel(QStandardItemModel):

    updated = pyqtSignal()

    def __init__(self):
        super(CustomTreeItemModel, self).__init__()

        self.update()

    def update(self):
        self.clear()

        ids = pmc.get_transform_ids()

        # create nodes from ids
        nodes = []
        for i in ids:
            node = pmc.get_transform(i)
            nodes.append(node)

        # create items from nodes
        items = {}
        for n in nodes:
            item = QStandardItem('%s [tf_%s]' % (n.name, n.number))
            params = {
                'name': n.name,
                'id': n.id,
                'ref_frame': n.ref_frame,
                'active': n.active,
                'translation': [
                    n.translation.x,
                    n.translation.y,
                    n.translation.z
                ],
                'rotation': [
                    n.rotation.x,
                    n.rotation.y,
                    n.rotation.z,
                    n.rotation.w
                ]
            }
            item.setData(params)
            item.setEditable(False)
            self._set_item_font(item)
            self._set_item_color(item)

            items[n.id] = item

        # assign ancestry of items
        for n in nodes:
            item = items[n.id]
            par_item = items[n.parent_id]

            if item.data()['id'] == par_item.data()['id']:
                self.invisibleRootItem().appendRow(item)
            elif par_item:
                par_item.appendRow(item)

        self.updated.emit()

    def dropMimeData(self, data, action, row, column, parent):
        success = super(CustomTreeItemModel, self).dropMimeData(data, action, row, column, parent)

        parent_item = self.itemFromIndex(parent)

        if row < 0:
            row = parent_item.rowCount() - 1

        item = parent_item.child(row)
        pmc.set_transform_parent(item.data()['id'], parent_item.data()['id'])

        order = get_child_ids(parent_item)
        pmc.set_iteration_order(parent_item.data()['id'], order)

        return success

    @staticmethod
    def _set_item_font(item):
        cur_tf_id = pmc.get_current_tf_id()
        f = item.font()

        if cur_tf_id == item.data()['id']:
            if not item.font().italic():
                f.setItalic(True)
        else:

            if item.font().italic():
                f.setItalic(True)

        item.setFont(f)

    @staticmethod
    def _set_item_color(item):

        if item.data()['active']:
            color = Qt.white
        else:
            color = Qt.gray

        item.setForeground(QBrush(QColor(color)))


class CustomTableItemModel(QStandardItemModel):

    def __init__(self):
        super(CustomTableItemModel, self).__init__()

        self.locked = ['active', 'ref_frame']

    def update(self, parent):

        if not parent:
            return

        self.clear()
        self.setColumnCount(1)

        parent_tf = pmc.get_transform(parent.data()['id'])

        if not parent_tf:
            return

        dict_ = OrderedDict()
        dict_['name'] = parent_tf.name
        dict_['ref_frame'] = parent_tf.ref_frame
        dict_['active'] = parent_tf.active
        dict_['translation'] = [parent_tf.translation.x, parent_tf.translation.y, parent_tf.translation.z]
        dict_['rotation'] = [parent_tf.rotation.x, parent_tf.rotation.y, parent_tf.rotation.z, parent_tf.rotation.w]

        i = 0
        for k, v in dict_.items():
            h = QStandardItem(k)
            t = QStandardItem(str(v))

            t.setData(parent_tf.id)

            editable = True if k not in self.locked else False
            t.setEditable(editable)

            self.setVerticalHeaderItem(i, h)
            self.setItem(i, 0, t)

            i += 1


class CustomProxyStyle(QProxyStyle):

    def __init__(self):
        super(CustomProxyStyle, self).__init__()

    def drawPrimitive(self, element, option, painter, widget=None):

        if element == self.PE_IndicatorItemViewItemDrop and not option.rect.isNull():
            pen = QPen(QColor(Qt.white))
            painter.setPen(pen)

        super(CustomProxyStyle, self).drawPrimitive(element, option, painter, widget)


class CustomTreeView(QTreeView):

    def __init__(self, parent):
        super(CustomTreeView, self).__init__(parent)

        self.parent = parent
        self.order = []

        self.setStyle(CustomProxyStyle())
        self.setDragEnabled(True)
        self.setDragDropMode(QAbstractItemView.InternalMove)
        self.setAcceptDrops(True)
        self.setDropIndicatorShown(True)
        self.setHeaderHidden(True)
        self.setContextMenuPolicy(Qt.CustomContextMenu)

        model = CustomTreeItemModel()
        self.setModel(model)

        model.updated.connect(lambda: self.expandToDepth(-1))
        self.customContextMenuRequested.connect(self._context_menu)

    def _context_menu(self, pos):
        cur_selection = get_current_selection(self)

        if not cur_selection:
            return

        menu = QMenu(self)
        ac_new_tf = menu.addAction("Add...")
        ac_remove = menu.addAction("Remove")

        menu.addSeparator()

        ac_set_actv = menu.addAction("Activate")
        ac_set_deactv = menu.addAction("Deactivate")

        action = menu.exec_(self.mapToGlobal(pos))
        if action == ac_new_tf:
            self.parent.call_action("open_create_widget", cur_selection)
        elif action == ac_remove:
            self.parent.call_action("remove_item", cur_selection)
        elif action == ac_set_actv:
            self.parent.call_action("activate_item", cur_selection)
        elif action == ac_set_deactv:
            self.parent.call_action("deactive_item", cur_selection)
        else:
            return


class CustomTableView(QTableView):

    def __init__(self, parent):
        super(CustomTableView, self).__init__(parent)

        self.parent = parent
        self.horizontalHeader().hide()

        model = CustomTableItemModel()
        self.setModel(model)

        model.dataChanged.connect(self._on_data_changed)

    def _on_data_changed(self):

        if self.selectionModel().hasSelection():
            index = self.selectionModel().currentIndex()
            item = self.model().itemFromIndex(index)
            item_header = self.model().verticalHeaderItem(item.row()).text()

            pmc.update_transform_var(item.data()['id'], str(item_header), str(item.text()))


class MainWidget(QWidget):

    def __init__(self):
        super(MainWidget, self).__init__()

        load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')
        self.setBaseSize(self.minimumSize())

        self.tree_view = CustomTreeView(self)
        self.param_view = CustomTableView(self)

        self.containerWidget.layout().addWidget(self.topWidget)
        self.containerWidget.layout().addWidget(self.progressWidget)
        self.containerWidget.layout().addItem(QSpacerItem(10, 10))
        self.containerWidget.layout().addWidget(self.bottomWidget)
        self.containerWidget.layout().addItem(QSpacerItem(20, 20))
        self.containerWidget.layout().addWidget(self.endWidget)

        self.topWidget.layout().addWidget(self.tree_view)
        self.topWidget.layout().addWidget(self.topButtonWidget)
        self.bottomWidget.layout().addWidget(self.param_view)
        self.bottomWidget.layout().addWidget(self.bottomButtonWidget)

        self.tree_view.selectionModel().selectionChanged.connect(
            lambda: self.param_view.model().update(get_current_selection(self.tree_view)))
        self.tree_view.selectionModel().selectionChanged.connect(self.param_view.resizeColumnsToContents)

        self.quitButton.clicked.connect(QApplication.quit)
        self.stepButton.clicked.connect(self._on_iterate)
        self.activeButton.clicked.connect(lambda: self.call_action(
            "activate_item",
            get_current_selection(self.tree_view)
        ))
        self.deactivateButton.clicked.connect(lambda: self.call_action(
            "deactivate_item",
            get_current_selection(self.tree_view)
        ))
        self.addButton.clicked.connect(lambda: self.call_action(
            "open_create_widget",
            get_current_selection(self.tree_view)
        ))
        self.removeButton.clicked.connect(lambda: self.call_action(
            "remove_item",
            get_current_selection(self.tree_view)
        ))
        self.resetButton.clicked.connect(self._on_reset_all)
        self.expandButton.clicked.connect(lambda: self.tree_view.expandToDepth(-1))
        self.saveButton.clicked.connect(self.save_tree)
        self.loadButton.clicked.connect(self.load_tree)

        self.init_cnt_actv = len(pmc.get_active_ids())
        if self.init_cnt_actv > 0:
            self.progressBar.setValue(100)

    def load_tree(self):
        dialog = QFileDialog()
        dialog.setDefaultSuffix('yaml')
        filename, _ = dialog.getOpenFileName(self, 'Open File', '/', "YAML (*.yaml *.yml)")

        if not filename:
            return

        pmc.load(filename)
        self.tree_view.model().update()

    def save_tree(self):
        dialog = QFileDialog()
        dialog.setDefaultSuffix('yaml')
        filename, _ = dialog.getSaveFileName(self, 'Open File', '/', "YAML (*.yaml *.yml);;All files (*.*)")

        if not filename:
            return

        if not filename.lower().endswith(('.yml', '.yaml')):
            filename += '.yaml'

        pmc.save(filename)

    def _on_reset_all(self):
        actv_ids = pmc.get_active_ids()

        for id_ in actv_ids:
            pmc.set_active(id_, False)

        self.progressBar.setValue(0)
        self.tree_view.model().update()

    def _on_iterate(self):
        cnt_actv = len(pmc.get_active_ids())

        if cnt_actv == 0:
            return

        if cnt_actv > 0:
            self.progressBar.setValue(int((float(cnt_actv) - 1.0) / float(self.init_cnt_actv) * 100.0))

        pmc.iterate()
        self.tree_view.model().update()

    def call_action(self, action, item):

        if not item:
            return

        if action == "open_create_widget":
            self.create_widget = CreateWidget(get_current_selection(self.tree_view))
            self.create_widget.show()

            self.create_widget.tfCreated.connect(self.tree_view.model().update)
            self.create_widget.tfCreated.connect(lambda: self.tree_view.expandToDepth(-1))

            return
        elif action == "remove_item":
            if item.parent():
                pmc.remove_transform(item.data()['id'])
                item.parent().removeRow(item.row())
        elif action == "activate_item":
            pmc.set_active(item.data()['id'], True)

            self.init_cnt_actv = len(pmc.get_active_ids())

            if self.init_cnt_actv > 0:
                self.progressBar.setValue(100)
        elif action == "deactivate_item":
            pmc.set_active(item.data()['id'], False)

            self.init_cnt_actv = len(pmc.get_active_ids())
            if self.init_cnt_actv == 0:
                self.progressBar.setValue(0)
        else:
            raise Exception()

        self.tree_view.model().update()
        self.tree_view.expandToDepth(-1)


class CreateWidget(QWidget):

    tfCreated = pyqtSignal()

    def __init__(self, parent):
        super(CreateWidget, self).__init__()

        self.parent = parent
        self.type_ = None

        load_ui('create.ui', self)
        self.setAttribute(Qt.WA_DeleteOnClose, True)
        self.layout().setSizeConstraint(QLayout.SetFixedSize)

        ref_text = str(self.parent.text()).split('[', 1)[0].strip()
        self.referenceText.setText(ref_text)
        self.parentText.setText(self.parent.text())

        self.patternWidget.close()

        self.scatter_model = QStandardItemModel(self.patternWidget)
        self.scatterListView.setModel(self.scatter_model)

        self.dialogButton.accepted.connect(self._on_accepted)
        self.dialogButton.rejected.connect(self._on_rejected)

        self.typeBox.currentIndexChanged.connect(self._on_type_index_changed)
        self.patternBox.currentIndexChanged.connect(self._on_pattern_index_changed)

        self.addPointButton.clicked.connect(self._on_add_scatter_point)
        self.removePointButton.clicked.connect(self._on_remove_scatter_point)

        lin_les = [self.numPointsText, self.stepSizeText, self.lengthText]
        rect_x_les = [self.numPointsXText, self.stepSizesXText, self.lengthsXText]
        rect_y_les = [self.numPointsYText, self.stepSizesYText, self.lengthsYText]

        for le in lin_les:
            le.textChanged.connect(lambda: self._restrict_num_fields(lin_les))

        for le in rect_x_les:
            le.textChanged.connect(lambda: self._restrict_num_fields(rect_x_les))

        for le in rect_y_les:
            le.textChanged.connect(lambda: self._restrict_num_fields(rect_y_les))

    @staticmethod
    def _restrict_num_fields(line_edits):
        count = len(line_edits) - 1

        for le in line_edits:
            if len(le.text()) > 0:
                count -= 1

        if count == 0:
            for le in line_edits:
                if len(le.text()) == 0:
                    le.setEnabled(False)
        else:
            for le in line_edits:
                le.setEnabled(True)

    def _on_add_scatter_point(self):
        point = [float(self.pointXText.text()), float(self.pointYText.text()), float(self.pointZText.text())]

        item = QStandardItem(str(point))
        self.scatter_model.appendRow(item)

    def _on_remove_scatter_point(self):
        index = self.scatterListView.selectionModel().currentIndex()
        cur_selection = self.scatter_model.itemFromIndex(index)

        if not cur_selection:
            return

        self.scatter_model.removeRow(cur_selection.row())

    def _on_type_index_changed(self):
        self.patternWidget.close()

        if self.typeBox.currentText() == 'Pattern':
            self.patternWidget.show()
            self._on_pattern_index_changed()

            return

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

        empty = self.check_required_field_empty(self.nameText)
        empty = self.check_required_field_empty(self.referenceText) or empty

        if empty:
            return

        if self.typeBox.currentText() == 'Transform':
            pmc.create_transform(self.nameText.text(), self.parent.data()['id'], self.referenceText.text())
        elif self.typeBox.currentText() == 'Pattern':
            self._create_pattern(self.patternBox.currentText())

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

    def _create_pattern(self, type_):
        in_ = [
            str(self.nameText.text()),
            self.parent.data()['id'],
            [
                self.xText,
                self.yText,
                self.zText,
                self.qxText,
                self.qyText,
                self.qzText,
                self.qwText
            ]
        ]

        for le in in_[2]:
            self._handle_empty_line(le)

        args = [
            in_[0],
            in_[1],
            [
                float(in_[2][0].text()),
                float(in_[2][1].text()),
                float(in_[2][2].text())
            ],
            [
                float(in_[2][3].text()),
                float(in_[2][4].text()),
                float(in_[2][5].text()),
                float(in_[2][6].text())
            ]
        ]

        if type_ == 'Linear':
            self._create_linear_pattern(args)
        elif type_ == 'Rectangular':
            self._create_rectangular_pattern(args)
        elif type_ == 'Scatter':
            self._create_scatter_pattern(args)
        elif type_ == 'Circular':
            self._create_circular_pattern(args)
        else:
            raise ValueError('Pattern type is not implemented')

    def _create_linear_pattern(self, args):
        in_ = [
            self.numPointsText,
            self.stepSizeText,
            self.lengthText
        ]

        for le in in_:
            self._handle_empty_line(le)

        args.extend([int(in_[0].text()), float(in_[1].text()), float(in_[2].text())])
        pmc.create_linear_pattern(*args)

    def _create_rectangular_pattern(self, args):
        in_ = [
            self.numPointsXText,
            self.numPointsYText,
            self.stepSizesXText,
            self.stepSizesYText,
            self.lengthsXText,
            self.lengthsYText
        ]

        for le in in_:
            self._handle_empty_line(le)

        args.extend([
            [int(in_[0].text()), int(in_[1].text())],
            [float(in_[2].text()), float(in_[3].text())],
            [float(in_[4].text()), float(in_[5].text())]
        ])

        pmc.create_rectangular_pattern(*args)

    def _create_circular_pattern(self, args):
        in_ = [
            self.numPointsText2,
            self.radiusText,
            self.angularText
        ]

        for le in in_:
            self._handle_empty_line(le)

        args.extend([
            int(in_[0].text()),
            float(in_[1].text()),
            self.tanRotCheck.isChecked(),
            self.cwCheck.isChecked(),
            float(in_[2].text())
        ])

        pmc.create_circular_pattern(*args)

    def _create_scatter_pattern(self, args):
        points = []
        for i in range(self.scatter_model.rowCount()):
            str_point_lst = list_string_to_list(str(self.scatter_model.item(i).text()))

            point = pm_msg.Point()
            for s in str_point_lst:
                point.point.append(float(s))

            points.append(point)

        args.append(points)
        pmc.create_scatter_pattern(*args)

    @staticmethod
    def _handle_empty_line(line_edit):
        if line_edit.text():
            return

        line_edit.setText(line_edit.placeholderText())

    @staticmethod
    def check_required_field_empty(field):

        if len(field.text()) == 0:
            field.setStyleSheet("border: 2px solid red;")
            rospy.logwarn('Required fields cannot be empty')

            return True

        return False
