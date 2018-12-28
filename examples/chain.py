# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import numpy as np


class DampingController(object):
    """ Add damping force to the skeleton """
    def __init__(self, skel):
        self.skel = skel

    def compute(self):
        damping = -0.01 * self.skel.dq
        damping[1::3] *= 0.1
        return damping


if __name__ == '__main__':
    import pydart2 as pydart
    from pydart2 import skeleton_builder

    pydart.init(verbose=True)
    print('pydart initialization OK')

    #pydart.world__addCapsule()

    #skel_empty = skeleton_builder.SkeletonBuilder("new human")

    #world = pydart.World(0.0002, './data/skel/chain.skel')
    world = pydart.World(0.0002, "EMPTY")
    world.set_gravity([ -9.81, 0,  0])
    print('pydart create_world OK')

    #pydart.world__addCapsule()

    skel = world.skeletons[0]
    ##skel_q_init = np.random.rand(skel.ndofs) - 0.5
    #skel_q_init = skel.ndofs * [0]
    #skel.set_positions(skel_q_init)
    #print skel.root_bodynode()
    #print skel.name
    #from pydart2 import bodynode
    #bn = bodynode.BodyNode(skel, 8)
    for body in skel.bodynodes:
        print body.C

    for joint in skel.joints:
        print joint

    for marker in skel.markers:
        print marker.world_position()


    print('init pose = %s' % skel.q)
    skel.controller = DampingController(skel)

    pydart.gui.viewer.launch(world)

    # # Or, you can manually create the window...
    # win = pydart.gui.viewer.PydartWindow(world)
    # win.camera_event(1)
    # win.run_application()
