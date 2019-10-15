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

import rospy
import pattern_manager.srv as pm_srv

from std_srvs.srv import Trigger


def update_transform_var(id_, var, val):
    rospy.wait_for_service('pattern_manager/change_name')
    try:
        chng_nm = rospy.ServiceProxy('pattern_manager/change_name', pm_srv.UpdateVar)
        resp = chng_nm(id_, var, val)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_transforms():
    rospy.wait_for_service('pattern_manager/get_transforms')
    try:
        get_tfs = rospy.ServiceProxy('pattern_manager/get_transforms', pm_srv.GroupTree)
        resp = get_tfs()

        return resp.group_deps
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_transform(name, parent_id=None):
    rospy.wait_for_service('pattern_manager/create_transform')
    try:
        crt_tf = rospy.ServiceProxy('pattern_manager/create_transform', pm_srv.CreateGroup)
        resp = crt_tf(name, parent_id)

        return resp.id
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def remove_transform(id_):
    rospy.wait_for_service('pattern_manager/remove_transform')
    try:
        rm_tf = rospy.ServiceProxy('pattern_manager/remove_transform', pm_srv.NodeId)
        resp = rm_tf(id_)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def set_active(id_, active):
    rospy.wait_for_service('pattern_manager/set_active')
    try:
        set_actv = rospy.ServiceProxy('pattern_manager/set_active', pm_srv.SetActive)
        resp = set_actv(id_, active)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def iterate():
    rospy.wait_for_service('pattern_manager/iterate')
    try:
        iter_ = rospy.ServiceProxy('pattern_manager/iterate', Trigger)
        resp = iter_()

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


# def set_pattern_active(id):
#     rospy.wait_for_service('pattern_manager/set_pattern_active')
#     try:
#         set_pat_actv = rospy.ServiceProxy('pattern_manager/set_pattern_active', pm_srv.NodeId)
#         resp = set_pat_actv(id)
#
#         return resp.success
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def get_pattern_types():
#     rospy.wait_for_service('pattern_manager/get_pattern_types')
#     try:
#         get_pat_typs = rospy.ServiceProxy('pattern_manager/get_pattern_types', pm_srv.PatternTypes)
#         resp = get_pat_typs()
#
#         return resp.pattern_types
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def get_pattern_type(id):
#     rospy.wait_for_service('pattern_manager/get_pattern_type')
#     try:
#         get_pat_typ = rospy.ServiceProxy('pattern_manager/get_pattern_type', pm_srv.PatternType)
#         resp = get_pat_typ(id)
#
#         return resp.type
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def get_patterns():
#     rospy.wait_for_service('pattern_manager/get_patterns')
#     try:
#         get_pats = rospy.ServiceProxy('pattern_manager/get_patterns', pm_srv.GroupTree)
#         resp = get_pats()
#
#         return resp.group_deps
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def get_groups():
#     rospy.wait_for_service('pattern_manager/get_groups')
#     try:
#         get_grps = rospy.ServiceProxy('pattern_manager/get_groups', pm_srv.GroupTree)
#         resp = get_grps()
#
#         return resp.group_deps
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def create_pattern(typ, nm, parent_id):
#     rospy.wait_for_service('pattern_manager/create_pattern')
#     try:
#         crt_pat = rospy.ServiceProxy('pattern_manager/create_pattern', pm_srv.CreatePattern)
#         resp = crt_pat(typ, nm, parent_id)
#
#         return resp.id
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def create_group(name, parent_id=None):
#     rospy.wait_for_service('pattern_manager/create_group')
#     try:
#         crt_grp = rospy.ServiceProxy('pattern_manager/create_group', pm_srv.CreateGroup)
#         resp = crt_grp(name, parent_id)
#
#         return resp.id
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def remove_group(id):
#     rospy.wait_for_service('pattern_manager/remove_group')
#     try:
#         rm_grp = rospy.ServiceProxy('pattern_manager/remove_group', pm_srv.NodeId)
#         resp = rm_grp(id)
#
#         return resp.success
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
#
#
# def remove_pattern(id):
#     rospy.wait_for_service('pattern_manager/remove_pattern')
#     try:
#         rm_pat = rospy.ServiceProxy('pattern_manager/remove_pattern', pm_srv.NodeId)
#         resp = rm_pat(id)
#
#         return resp.success
#     except rospy.ServiceException, e:
#         print 'Service call failed: %s' % e
