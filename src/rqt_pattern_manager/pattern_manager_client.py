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

import sys
import rospy
import pattern_manager.srv as pm_srv


class PatternManagerClient():
    def __init__(self):
        pass
    
    def _get_pattern_types(self):
        rospy.wait_for_service('pattern_manager/get_pattern_types')
        try:
            get_pat_typs = rospy.ServiceProxy('pattern_manager/get_pattern_types', pm_srv.PatternTypes)
            resp = get_pat_typs()

            return resp.pattern_types
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_patterns(self):
        rospy.wait_for_service('pattern_manager/get_patterns')
        try:
            get_pats = rospy.ServiceProxy('pattern_manager/get_patterns', pm_srv.GroupTree)
            resp = get_pats()

            return resp.group_deps
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_groups(self):
        rospy.wait_for_service('pattern_manager/get_groups')
        try:
            get_grps = rospy.ServiceProxy('pattern_manager/get_groups', pm_srv.GroupTree)
            resp = get_grps()

            return resp.group_deps
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def create_pattern(self, pat_type, pat_name, gop_id):
        rospy.wait_for_service('pattern_manager/create_pattern')
        try:
            crt_pat = rospy.ServiceProxy('pattern_manager/create_pattern', pm_srv.CreatePattern)
            resp = crt_pat(pat_type, pat_name, gop_id)

            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def create_group(self, g_type, name):
        rospy.wait_for_service('pattern_manager/create_group')
        try:
            crt_grp = rospy.ServiceProxy('pattern_manager/create_group', pm_srv.CreateGroup)
            resp = crt_grp(g_type, name)

            return resp.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


if __name__ == '__main__':
    pmc = PatternManagerClient()
    print pmc._get_pattern_types()