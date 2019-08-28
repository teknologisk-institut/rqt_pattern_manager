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
import subprocess
import rosnode
import pattern_manager.srv as pm_srv

from abc import ABCMeta, abstractmethod


class Service(rospy.ServiceProxy):

    def __init__(self, srv_name, srv_type):
        self.srv = super(Service, self).__init__(srv_name, srv_type)
        self.srv_name = srv_name

    def go(self, *args):
        rospy.wait_for_service(self.srv_name)

        try:
            print self
            return self.call(*args)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


@abstractmethod
class ServiceFactory(object):
    __metaclass__ = ABCMeta

    _services = {}

    @staticmethod
    def register(name, srv_class):
        ServiceFactory._services[name] = Service(name, srv_class)

    @staticmethod
    def get_proxy(name):
        srv_proxy = ServiceFactory._services[name]

        if not srv_proxy:
            raise ValueError(name)

        return srv_proxy


class Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]


class PatternManagerClient():
    __metaclass__ = Singleton

    def __init__(self):
        ServiceFactory.register(
            'pattern_manager/get_pattern_types',
            pm_srv.PatternTypes
        )
        ServiceFactory.register(
            'pattern_manager/get_patterns',
            pm_srv.GroupTree
        )
        ServiceFactory.register(
            'pattern_manager/get_groups',
            pm_srv.GroupTree
        )
        ServiceFactory.register(
            'pattern_manager/create_pattern',
            pm_srv.CreatePattern
        )
        ServiceFactory.register(
            'pattern_manager/create_group',
            pm_srv.CreateGroup
        )

        if '/pattern_manager' in rosnode.get_node_names():
            return

        self.run_node('pattern_manager', 'pattern_manager')

    def get_service(self, name):
        return ServiceFactory.get_proxy(name)

    def run_node(self, pkg_name, exec_name):
        try:
            subprocess.Popen(['rosrun', pkg_name, exec_name])
        except subprocess.CalledProcessError, e:
            print "Error: could not run {} node: {}".format(pkg_name, e)
