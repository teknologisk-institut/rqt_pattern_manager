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
import geometry_msgs.msg as gm_msg
import ast

from .util import make_pattern_params_msg


def load(filename):
    rospy.wait_for_service('pattern_manager/load', 20)
    try:
        ld = rospy.ServiceProxy('pattern_manager/load', pm_srv.Filename)
        resp = ld(filename)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def save(filename):
    rospy.wait_for_service('pattern_manager/save', 20)
    try:
        sv = rospy.ServiceProxy('pattern_manager/save', pm_srv.Filename)
        resp = sv(filename)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def set_transform_parent(id_, parent_id):
    rospy.wait_for_service('pattern_manager/set_transform_parent', 20)
    try:
        set_par = rospy.ServiceProxy('pattern_manager/set_transform_parent', pm_srv.SetParent)
        resp = set_par(id_, parent_id)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_transform_ids():
    rospy.wait_for_service('pattern_manager/get_transform_ids', 20)
    try:
        get_ids = rospy.ServiceProxy('pattern_manager/get_transform_ids', pm_srv.GetIds)
        resp = get_ids()

        return resp.ids
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def set_iteration_order(parent_id, order):
    rospy.wait_for_service('pattern_manager/set_iteration_order', 20)
    try:
        set_order = rospy.ServiceProxy('pattern_manager/set_iteration_order', pm_srv.SetIterationOrder)
        resp = set_order(parent_id, order)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_linear_pattern(name, parent_id, translation, rotation, num_points, step_size, length):
    rospy.wait_for_service('pattern_manager/create_linear_pattern', 20)
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_linear_pattern', pm_srv.CreateLinearPattern)

        pparent = make_pattern_params_msg(name, parent_id, translation, rotation)
        resp = crt_pat(pparent, num_points, step_size, length)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_rectangular_pattern(name, parent_id, translation, rotation, num_points, step_size, length):
    rospy.wait_for_service('pattern_manager/create_rectangular_pattern', 20)
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_rectangular_pattern', pm_srv.CreateRectangularPattern)

        pparent = make_pattern_params_msg(name, parent_id, translation, rotation)
        resp = crt_pat(pparent, num_points, step_size, length)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_scatter_pattern(name, parent_id, translation, rotation, points):
    rospy.wait_for_service('pattern_manager/create_scatter_pattern', 20)
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_scatter_pattern', pm_srv.CreateScatterPattern)

        pparent = make_pattern_params_msg(name, parent_id, translation, rotation)
        resp = crt_pat(pparent, points)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_circular_pattern(name, parent_id, translation, rotation, num_points, r, tan_rot, cw, angular_section):
    rospy.wait_for_service('pattern_manager/create_circular_pattern', 20)
    try:
        crt_pat = rospy.ServiceProxy('pattern_manager/create_circular_pattern', pm_srv.CreateCircularPattern)

        pparent = make_pattern_params_msg(name, parent_id, translation, rotation)
        resp = crt_pat(pparent, num_points, r, tan_rot, cw, angular_section)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_active_ids():
    rospy.wait_for_service('pattern_manager/get_active_ids', 20)
    try:
        actv_ids = rospy.ServiceProxy('pattern_manager/get_active_ids', pm_srv.GetIds)
        resp = actv_ids()

        return resp.ids
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_current_tf_id():
    rospy.wait_for_service('pattern_manager/get_current_transform_id', 20)
    try:
        cur_tf_id = rospy.ServiceProxy('pattern_manager/get_current_transform_id', pm_srv.GetCurrentId)
        resp = cur_tf_id()

        return resp.id
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def iterate():
    rospy.wait_for_service('pattern_manager/iterate', 20)
    try:
        iter_ = rospy.ServiceProxy('pattern_manager/iterate', std_srv.Trigger)
        resp = iter_()

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def update_transform(id_, name, ref_frame, active, translation, rotation):
    rospy.wait_for_service('pattern_manager/update_transform', 20)
    try:
        trans = ast.literal_eval(translation)
        rot = ast.literal_eval(rotation)

        req = pm_srv.UpdateTransformRequest()
        req.id = id_
        req.name = str(name)
        req.ref_frame = str(ref_frame)
        req.active = bool(active)
        req.translation = gm_msg.Vector3(x=trans[0], y=trans[1], z=trans[2])
        req.rotation = gm_msg.Quaternion(x=rot[0], y=rot[1], z=rot[2], w=rot[3])

        chng_nm = rospy.ServiceProxy('pattern_manager/update_transform', pm_srv.UpdateTransform)
        resp = chng_nm(req)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def get_transform(id_):
    rospy.wait_for_service('pattern_manager/get_transform', 20)
    try:
        get_tf = rospy.ServiceProxy('pattern_manager/get_transform', pm_srv.GetTransformParams)
        resp = get_tf(id_)

        return resp.params
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def create_transform(name, parent_id, ref_frame, translation, rotation):
    rospy.wait_for_service('pattern_manager/create_transform', 20)
    try:
        crt_tf = rospy.ServiceProxy('pattern_manager/create_transform', pm_srv.CreateTransform)

        params = make_pattern_params_msg(name, parent_id, translation, rotation, ref_frame)
        resp = crt_tf(params)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def remove_transform(id_):
    rospy.wait_for_service('pattern_manager/remove_transform', 20)
    try:
        rm_tf = rospy.ServiceProxy('pattern_manager/remove_transform', pm_srv.TransformId)
        resp = rm_tf(id_)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e


def set_active(id_, active):
    rospy.wait_for_service('pattern_manager/set_active', 20)
    try:
        set_actv = rospy.ServiceProxy('pattern_manager/set_active', pm_srv.SetActive)
        resp = set_actv(id_, active)

        return resp.success
    except rospy.ServiceException, e:
        print 'Service call failed: %s' % e
