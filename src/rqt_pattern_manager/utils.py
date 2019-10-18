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


def load_ui(file_, widget):
    rp = rospkg.RosPack()
    ui_file = os.path.join(rp.get_path('rqt_pattern_manager'), 'resource', file_)
    loadUi(ui_file, widget)


def param_tree_from_nested_lists(lst, nodes):
    tree = {}
    for i in lst:
        name = i.name
        par_name = i.parent_name
        node = nodes[name]

        if name == par_name:
            tree[name] = node
        else:
            parent = nodes[par_name]
            parent[name] = node

    return tree


def find_item_child_by_id(parent, id_):
    for i in range(0, parent.rowCount()):
        cur_child = parent.child(i)

        if cur_child.data()['id'] == id_:
            return cur_child

    return None
