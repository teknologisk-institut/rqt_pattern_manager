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
import rospy
import pickle

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
from PyQt5.QtGui import QStandardItem, QStandardItemModel, QBrush, QColor, QPen
from PyQt5.QtCore import pyqtSignal, Qt, QVariant, QDataStream, QIODevice, QSize
from .util import *
from collections import OrderedDict


class CustomTreeItemModel(QStandardItemModel):
    """
    This class is a subclass of `QStandardItemModel` used as the model of the `CustomTreeView` class
    """

    updated = pyqtSignal()

    def __init__(self):
        super(CustomTreeItemModel, self).__init__()

        self.update()

    def update(self):
        """
        This function fetches transform information from the pattern_manager node and updates the model accordingly
        """

        self.clear()

        ids = pmc.get_transform_ids()

        nodes = []
        for i in ids:
            node = pmc.get_transform(i)
            nodes.append(node)

        items = {}
        for n in nodes:
            item = QStandardItem('%s [tf_%s]' % (n.name, n.number))
            params = {
                'name': n.name,
                'id': n.id,
                'parent_id': n.parent_id,
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

        for n in nodes:
            item = items[n.id]
            par_item = items[n.parent_id]

            if item.data()['id'] == par_item.data()['id']:
                self.invisibleRootItem().appendRow(item)
            elif par_item:
                par_item.appendRow(item)

        self.updated.emit()

    def dropMimeData(self, data, action, row, column, parent):
        """
        Reimplementation of super function. This function updates a transform's parent data and updates the ordering
        of the parent's children after a drop event has occurred

        :param data: The dropped item's MIME data
        :type data: QMimeData
        :param action: The assigned drop action
        :type action: Qt_DropAction
        :param row: The row where the item is dropped (-1 if dropped between items)
        :type row: int
        :param column: The column where the item is dropped
        :type column: int
        :param parent: The model index of the parent of the dropped item
        :type parent: QModelIndex
        :return: `True` if the drop was successful, otherwise `False`
        :rtype: bool
        """

        # if row < 0:
        #     return False

        parent_item = self.itemFromIndex(parent)

        bytearray_ = data.data('application/x-qabstractitemmodeldatalist')
        data_items = decode_mime_data(bytearray_)
        item_name = data_items[0][Qt.DisplayRole].value()
        item = self.findItems(item_name, Qt.MatchRecursive | Qt.MatchExactly)[0]

        if row > 0 and item.data()['parent_id'] != parent_item.data()['id']:
            return False

        super(CustomTreeItemModel, self).dropMimeData(data, action, row, column, parent)

        if row < 0:
            pmc.set_transform_parent(item.data()['id'], parent_item.data()['id'])

            return True

        ids = []
        for i in range(parent_item.rowCount()):
            child = parent_item.child(i)
            id_ = child.data()['id']

            if id_ == item.data()['id'] and i != row:
                continue

            ids.append(id_)

        if len(ids) > 1:
            pmc.set_iteration_order(parent_item.data()['id'], ids)

        return True

    @staticmethod
    def _set_item_font(item):
        """
        This function assigns the font of a supplied item

        :param item: The item to assign a font
        :type item: QStandardItem
        """

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
        """
        This function assigns the color of a supplied item

        :param item: The item to assign a color
        :type item: QStandardItem
        """

        if item.data()['active']:
            color = Qt.white
        else:
            color = Qt.gray

        item.setForeground(QBrush(QColor(color)))


class CustomTableItemModel(QStandardItemModel):
    """
    This class is a subclass of `QStandardItemModel` used as the model of the `CustomTableView` class
    """

    def __init__(self):
        super(CustomTableItemModel, self).__init__()

        self.locked = ['active']

    def update(self, parent):
        """
        This function fetches transform information from the pattern_manager node and updates the model accordingly
        """

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
    """
    This class is a subclass of `QProxyStyle` used as the style proxy of the `CustomTreeView` class
    """

    def __init__(self):
        super(CustomProxyStyle, self).__init__()

    # def drawPrimitive(self, QStyle_PrimitiveElement, QStyleOption, QPainter, QWidget_widget=None):
    def drawPrimitive(self, element, option, painter, widget=None):
        """
        Reimplementation of super function. This function handles the drawing of the item view drop indicator

        :param element: The primitive element to be drawn
        :type element: QStyle_PrimitiveElement
        :param option: Style options for the draw event
        :type option: QStyleOption
        :param painter: The painter used to draw the primitive
        :type painter: QPainter
        :param widget: An associated widget object
        :type widget: QWidget
        """

        if element == self.PE_IndicatorItemViewItemDrop and not option.rect.isNull():
            pen = QPen(QColor(Qt.white))
            painter.setPen(pen)

        super(CustomProxyStyle, self).drawPrimitive(element, option, painter, widget)


class CustomTreeView(QTreeView):
    """
    This class is a subclass of `QTreeView` and is the view of the `CustomTreeItemModel` class

    :param parent: The parent widget of the class
    :type parent: QWidget
    """

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
        """
        This callback function creates a context menu and assigns the various actions to it

        :param pos: The pointer position at the time of the call
        :type pos: tuple
        """

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
    """
    This class is a subclass of `QTableView` and is the view of the `CustomTableItemModel` class

    :param parent: The parent widget of the class
    :type parent: QWidget
    """

    updated = pyqtSignal()

    def __init__(self, parent):
        super(CustomTableView, self).__init__(parent)

        self.parent = parent
        self.horizontalHeader().hide()

        model = CustomTableItemModel()
        self.setModel(model)

        model.dataChanged.connect(self._on_data_changed)

    def _on_data_changed(self):
        """
        This callback function updates the associated transform when data is changed in the model

        """

        if not self.selectionModel().hasSelection():
            return

        index = self.selectionModel().currentIndex()
        item = self.model().itemFromIndex(index)

        table_dict = {'id_': item.data()}
        for i in range(self.model().rowCount()):
            item = self.model().item(i)
            item_header = self.model().verticalHeaderItem(i).text()

            table_dict[str(item_header)] = str(item.text())

        pmc.update_transform(**table_dict)

        self.updated.emit()


class MainWidget(QWidget):
    """
    This class is the main widget of the app
    """

    def __init__(self):
        super(MainWidget, self).__init__()

        load_ui('pattern_manager_widget.ui', self)
        self.setObjectName('PatternManagerWidget')
        self.setBaseSize(self.minimumSize())

        self.tree_view = CustomTreeView(self)
        self.param_view = CustomTableView(self.tree_view)

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
        self.param_view.updated.connect(self.tree_view.model().update)

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
        self.saveButton.clicked.connect(self._save_tree)
        self.loadButton.clicked.connect(self._load_tree)

        self.init_cnt_actv = len(pmc.get_active_ids())
        if self.init_cnt_actv > 0:
            self.progressBar.setValue(100)

    def _load_tree(self):
        """
        This callback function creates a new transform tree from a chosen .yaml file
        and populates the models accordingly
        """

        dialog = QFileDialog()
        dialog.setDefaultSuffix('yaml')
        filename, _ = dialog.getOpenFileName(self, 'Open File', '/', "YAML (*.yaml *.yml)")

        if not filename:
            return

        pmc.load(filename)
        self.tree_view.model().update()

    def _save_tree(self):
        """
        This callback function saves the current transform tree to a .yaml file
        """

        dialog = QFileDialog()
        dialog.setDefaultSuffix('yaml')
        filename, _ = dialog.getSaveFileName(self, 'Open File', '/', "All files (All files (*.*)")

        if not filename:
            return

        if not filename.lower().endswith(('.yml', '.yaml')):
            filename += '.yaml'

        pmc.save(filename)

    def _on_reset_all(self):
        """
        This callback function deactivates all currently active transforms and resets the progress bar
        """

        actv_ids = pmc.get_active_ids()

        for id_ in actv_ids:
            pmc.set_active(id_, False)

        self.progressBar.setValue(0)
        self.tree_view.model().update()

    def _on_iterate(self):
        """
        This callback function iterates the active transforms and updates the progress bar
        """

        cnt_actv = len(pmc.get_active_ids())

        if cnt_actv == 0:
            return

        if cnt_actv > 0:
            self.progressBar.setValue(int((float(cnt_actv) - 1.0) / float(self.init_cnt_actv) * 100.0))

        pmc.iterate()
        self.tree_view.model().update()

    def call_action(self, action, item):
        """
        This function executes a chosen action selected in the user interfaces

        :param action: The action to be executed
        :type action: str
        :param item: The item associated with the action
        :type item: QStandardItem
        """

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
    """
    This class is a subclass of `QWidget` which opens a widget used to create transforms

    :param parent: The parent widget
    :type parent: QWidget
    """

    tfCreated = pyqtSignal()

    def __init__(self, parent):
        super(CreateWidget, self).__init__()

        self.parent = parent
        self.type_ = None

        load_ui('create_widget.ui', self)
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
        """
        This function specifically disables QLineEdit widgets of a parent widget when the required number of
        field have been assigned

        :param line_edits: A list of QLineEdit widgets
        :type line_edits: list
        """

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
        """
        This callback function updates the scatterListView model when a new entry is
        entered and added (i.e. a new position)
        """

        point = [float(self.pointXText.text()), float(self.pointYText.text()), float(self.pointZText.text())]

        item = QStandardItem(str(point))
        self.scatter_model.appendRow(item)

    def _on_remove_scatter_point(self):
        """
        This callback function updates the scatterListView model when an item is removed
        """

        index = self.scatterListView.selectionModel().currentIndex()
        cur_selection = self.scatter_model.itemFromIndex(index)

        if not cur_selection:
            return

        self.scatter_model.removeRow(cur_selection.row())

    def _on_type_index_changed(self):
        """
        This callback function administers the opening and closing of the appropriate widgets
        when a type (i.e. transform or pattern) is chosen in the typeBox dropdown widget
        """

        self.patternWidget.close()

        if self.typeBox.currentText() == 'Pattern':
            self.patternWidget.show()
            self._on_pattern_index_changed()

            return

    def _on_pattern_index_changed(self):
        """
        This callback function administers the opening and closing of the appropriate widgets
        when a pattern (e.g. linear, rectangular) is chosen in the patternBox dropdown widget
        """

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

    def _get_transform_pose(self):
        """
        This function retrieves the pose information of root transform from the transformWidget widget,
        handles empty entries, and casts the data to the appropriate data type

        :return: The position and orientation of the root transform
        :rtype: tuple
        """

        in_ = [
            self.xText,
            self.yText,
            self.zText,
            self.qxText,
            self.qyText,
            self.qzText,
            self.qwText
        ]

        for le in in_:
            self._handle_empty_line(le)

        xyz = [
            float(self.xText.text()),
            float(self.yText.text()),
            float(self.zText.text())
        ]

        q = [
            float(self.qxText.text()),
            float(self.qyText.text()),
            float(self.qzText.text()),
            float(self.qwText.text())
        ]

        return xyz, q

    def _on_accepted(self):
        """
        This callback function handles the creation of the requested transforms when 'OK' (accepted)
        is selected in the dialogButton widget
        """

        empty = self.check_required_field_empty(self.nameText)
        empty = self.check_required_field_empty(self.referenceText) or empty

        if empty:
            return

        tf_pose = self._get_transform_pose()

        if self.typeBox.currentText() == 'Transform':
            pmc.create_transform(self.nameText.text(),
                                 self.parent.data()['id'],
                                 self.referenceText.text(),
                                 tf_pose[0],
                                 tf_pose[1])
        elif self.typeBox.currentText() == 'Pattern':
            self._create_pattern(self.patternBox.currentText(),
                                 tf_pose[0],
                                 tf_pose[1])

        self.tfCreated.emit()
        self._on_rejected()

    def _on_rejected(self):
        """
        This callback function handles the resetting of widget and closing of the widget when 'Cancel' (rejected)
        is selected in the dialogButton widget
        """

        wgds = self.findChildren(QLineEdit)
        wgds.extend(self.findChildren(QComboBox))

        for w in wgds:
            w.clear()

        for c in self.findChildren(QCheckBox):
            c.setChecked(False)

        self.close()

    def _create_pattern(self, type_, xyz, q):
        """
        This function calls the appropriate function, responsible for the creation of a transform pattern,
        from the supplied type

        :param type_: The type of pattern to be created
        :type type_: str
        :param xyz: The position of the root transform (parent)
        :type xyz: list
        :param q: The orientation of the root transform (parent)
        :type q: list
        """

        for le in self.patternWidget.findChildren(QLineEdit):
            le.setEnabled(True)

        args = [
            str(self.nameText.text()),
            self.parent.data()['id'],
            xyz,
            q
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
        """
        This function creates a linear pattern of transforms under the parent transform

        :param args: data about the parent transform
        :type args: list
        """

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
        """
        This function creates a rectangular pattern of transforms under the parent transform

        :param args: data about the parent transform
        :type args: list
        """

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
        """
        This function creates a circular pattern of transforms under the parent transform

        :param args: data about the parent transform
        :type args: list
        """

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
        """
        This function creates a scatter pattern of transforms under the parent transform

        :param args: data about the parent transform
        :type args: list
        """

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
        """
        This function checks if a field has been left empty, and if so, assigns it its default value

        :param line_edit: The input field for parameters
        :type line_edit: QLineEdit
        """

        if line_edit.text():
            return

        line_edit.setText(line_edit.placeholderText())

    @staticmethod
    def check_required_field_empty(field):
        """
        This function checks if a required field has been left empty, and if so,
        highlights the field with a red triangle

        :param field: The input field to check
        :type field: QLineEdit
        :return: `True` if field is empty, otherwise `False`
        :rtype: bool
        """

        if len(field.text()) == 0:
            field.setStyleSheet("border: 2px solid red;")
            rospy.logwarn('Required fields cannot be empty')

            return True

        return False
