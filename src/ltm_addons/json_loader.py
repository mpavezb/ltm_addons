#!/usr/bin/env python
# -*- coding: utf-8 -*-
__author__ = 'Matías Pavez'
__email__ = 'matias.pavez@ing.uchile.cl'

from os import listdir
from os.path import abspath, isdir, isfile, join
import rospy
from ltm_addons.json_parser import JsonParser
from ltm.srv import *


class JsonLoader(object):

    def __init__(self, source=None, ns="", logging=True):
        self._is_file = None
        self.parser = JsonParser()
        self.saved_episodes = []
        self.failed_episodes = []
        self.root_episodes = []

        self.loginfo = lambda *args: None
        self.logwarn = lambda *args: None
        self.logerr = lambda *args: None
        if logging:
            self.loginfo = rospy.loginfo
            self.logwarn = rospy.logwarn
            self.logerr = rospy.logerr

        # ROS parameters
        self.source = source
        if not self.source:
            self.source = rospy.get_param("~source", None)
        self.setup_path()

        # ROS clients

        self.loginfo("Waiting for LTM server to be up.")
        self.register_episode_client = rospy.ServiceProxy(ns + 'ltm/episode/register', RegisterEpisode)
        self.add_episode_client = rospy.ServiceProxy(ns + 'ltm/episode/add', AddEpisode)
        self.update_tree_client = rospy.ServiceProxy(ns + 'ltm/episode/update_tree', UpdateTree)

        # Wait for ROS services
        self.add_episode_client.wait_for_service()
        self.update_tree_client.wait_for_service()

    def setup_path(self):
        if self.source is None:
            self.logerr("Source parameter cannot be empty.")
            exit(1)
        self.source = abspath(self.source)
        if isdir(self.source):
            self._is_file = False
            self.loginfo("Loading directory: " + self.source)
        elif isfile(self.source):
            self._is_file = True
            self.loginfo("Loading JSON file: " + self.source)
        else:
            self.logerr("Source parameter is not a valid filename or directory: '" + self.source + "'")
            exit(1)

    def save(self, episode):
        try:
            # register episode
            reg_req = RegisterEpisodeRequest()
            reg_req.gather_emotion = False
            reg_req.gather_location = False
            reg_req.gather_streams = False
            reg_req.gather_entities = False
            reg_req.replace = True
            reg_req.generate_uid = False
            reg_req.uid = episode.uid
            self.loginfo("Registering episode: " + str(episode.uid))
            self.register_episode_client(reg_req)

            # add episode
            req = AddEpisodeRequest()
            req.episode = episode
            req.replace = True
            self.loginfo("Adding episode: " + str(episode.uid))
            res = self.add_episode_client(req)
            if res.succeeded:
                self.saved_episodes.append(episode.uid)
                if episode.parent_id == 0:
                    self.root_episodes.append(episode.uid)
                return True
        except rospy.ServiceException, e:
            self.logerr("Service call failed: %s" % e)
        self.failed_episodes.append(episode.uid)
        return False

    def load(self):
        if self._is_file:
            self.load_file(self.source)
            if len(self.saved_episodes) == 1:
                self.update_tree(self.saved_episodes[0])
        else:
            self.load_folder()

    def load_file(self, filename):
        data = self.parser.load_json(filename)
        episode = self.parser.json_to_episode(data)
        return self.save(episode)

    def update_tree(self, uid):
        self.loginfo(" - updating tree with root uid: '" + str(uid) + "'.")
        try:
            self.update_tree_client(uid)
            return True
        except rospy.ServiceException, e:
            self.logerr("Service call failed: %s" % e)
        return False

    def load_folder(self):
        # get JSONs
        files = [f for f in listdir(self.source) if isfile(join(self.source, f)) and f.endswith('.json')]
        files.sort()
        if not files:
            self.logwarn("No .json files were found.")
            return

        # load JSONs
        for filename in files:
            full_filename = join(self.source, filename)
            self.loginfo(" - loading json: " + filename)
            self.load_file(full_filename)
        self.loginfo("The following episodes were saved: " + str(self.saved_episodes))
        self.loginfo("The following episodes were not saved: " + str(self.failed_episodes))
        self.loginfo("The following roots were saved: " + str(self.root_episodes))

        # update tree
        for uid in self.root_episodes:
            self.update_tree(uid)


def main():
    try:
        rospy.init_node("ltm_json_loader")
        node = JsonLoader()
        node.load()
    except rospy.ROSInterruptException:
        pass
