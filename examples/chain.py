# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import numpy as np

RENDER_DART = True

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
    world.set_gravity([0, 0,  9.81])
    print('pydart create_world OK')

    #pydart.world__addCapsule()

    skel = world.skeletons[0]
    #skel_q_init = np.random.rand(skel.ndofs) - 0.5
    skel_q_init = skel.ndofs * [0]
    skel_q_init[3] = np.pi/2
    print skel_q_init
    skel.set_positions(skel_q_init)
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

    #pydart.gui.viewer.launch(world)

    if RENDER_DART == False:
        for i in range(0, 100):
            world.step()
            print "did a step"
            skel = world.skeletons[0]
            print skel.q

    elif RENDER_DART == True:
        from pydart2.gui.glut.window import GLUTWindow
        win = GLUTWindow(world, title=None)
        default_camera = None
        if default_camera is not None:
            win.scene.set_camera(default_camera)
        win.run()
