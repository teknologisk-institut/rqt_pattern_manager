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
import std_srvs.srv as std_srv


def create_linear_pattern(parent_id, num_points, step_size, length):
    rospy.wait_for_service('pattern_manager/create_linear_pattern')
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_linear_pattern', pm_srv.CreateLinearPattern)
        resp = crt_pat(num_points, step_size, length, parent_id)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_rectangular_pattern(parent_id, num_points, step_size, length):
    rospy.wait_for_service('pattern_manager/create_rectangular_pattern')
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_rectangular_pattern', pm_srv.CreateRectangularPattern)
        resp = crt_pat(num_points, step_size, length, parent_id)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_active_ids():
    rospy.wait_for_service('pattern_manager/get_active_ids')
    try:
        actv_ids = rospy.ServiceProxy('pattern_manager/get_active_ids', pm_srv.ActiveIds)
        resp = actv_ids()

        return resp.ids
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_current_tf_id():
    rospy.wait_for_service('pattern_manager/get_current_tf_id')
    try:
        cur_tf_id = rospy.ServiceProxy('pattern_manager/get_current_tf_id', pm_srv.IdRequest)
        resp = cur_tf_id()

        return resp.id
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def iterate():
    rospy.wait_for_service('pattern_manager/iterate')
    try:
        iter_ = rospy.ServiceProxy('pattern_manager/iterate', std_srv.Trigger)
        resp = iter_()

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def update_transform_var(id_, var, val):
    rospy.wait_for_service('pattern_manager/update_tf_variable')
    try:
        chng_nm = rospy.ServiceProxy('pattern_manager/update_tf_variable', pm_srv.UpdateVar)
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


def create_transform(name, parent_id, ref_frame):
    rospy.wait_for_service('pattern_manager/create_transform')
    try:
        crt_tf = rospy.ServiceProxy('pattern_manager/create_transform', pm_srv.CreateGroup)
        resp = crt_tf(name, parent_id, ref_frame)

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
