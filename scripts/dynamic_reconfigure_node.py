#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamic_reconfigure.server import Server
from sphero_formation.cfg import ReynoldsConfig


class DynReconf():
    """
    Dynamic reconfigure server.

    Process parameter changes from rqt_reconfigure and update parameter server.
    Publish empty message to let other nodes know there are updated parameters
    on server.
    """
    def __init__(self):
        """Initialize dynamic reconfigure server."""
        Server(ReynoldsConfig, self.callback)

        # Keep program from exiting
        rospy.spin()

    def callback(self, config, level):
        """Display all parameters when changed and signal to update."""
        rospy.loginfo("[Dynamic reconfigure] => \n" +
                      """\tReconfigure Request:
                        Alignment: {alignment_factor}
                        Cohesion: {cohesion_factor}
                        Separation: {separation_factor}
                        Avoid: {avoid_factor}
                        Max speed: {max_speed}
                        Max force: {max_force}
                        Friction: {friction}
                        Crowd radius: {crowd_radius}
                        Search radius: {search_radius}
                        Avoid radius: {avoid_radius}""".format(**config))
        return config


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("dyn_reconf", anonymous=False)

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        dr = DynReconf()
    except rospy.ROSInterruptException:
        pass
