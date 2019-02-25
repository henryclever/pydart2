# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import numpy as np
import pydart2 as pydart
from pydart2 import skeleton_builder
from dart_opengl_window import GLUTWindow


import OpenGL.GL as GL
import OpenGL.GLU as GLU
import OpenGL.GLUT as GLUT
import sys
from pydart2.gui.opengl.scene import OpenGLScene
import math
from time import time

GRAVITY = -9.81
STARTING_HEIGHT = 1.0

K = 1269.0
B = 10000.0
FRICTION_COEFF = 0.5

NUM_CAPSULES = 20
DART_TO_FLEX_CONV = 2.58872



class DartSkelSim(object):
    def __init__(self):

        joint_names = ['pelvis', 'leftThigh', 'rightThigh', 'spine', 'leftCalf', 'rightCalf', 'spine1', 'leftFoot', 'rightFoot',
                     'spine2', 'neck', 'leftShoulder', 'rightShoulder', 'head', 'leftUpperArm', 'rightUpperArm', 'leftForeArm',
                     'rightForeArm', 'leftHand', 'rightHand']

        red_parent_ref = [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 9, 9, 9, 10, 11, 12, 14, 15, 16, 17]

        radii = [0.12769461096148343, 0.09182758062402535, 0.09183365264505983, 0.128195873394974, 0.054201506359127107,
                 0.054204782651247056, 0.11441021953758712, 0.05134816175061338, 0.051367681280828095, 0.12840909729388708,
                 0.08568376456832136, 0.09205466746236975, 0.09207195048860886, 0.09417674072728775, 0.04450030029080607,
                 0.04453954596015603, 0.03869968571382823, 0.03875571491372652, 0.02922814811888941, 0.0297973538063161]

        radii_med = [0.1443921929584865, 0.09879753554393726, 0.09880958937889255, 0.14038170079741627, 0.05892499163491047,
                     0.05891232648197925, 0.13581042138687494, 0.04932037409824811, 0.049343780055406926, 0.1396929637576165,
                     0.0861958797422231, 0.09078340056685348, 0.09077927107473645, 0.09547074699086751, 0.05319511539915989,
                     0.0532042033919998, 0.041963113363422504, 0.04198389621382932, 0.02922814811888941, 0.0297973538063161]

        lengths = [0.1331743261250832, 0.25341930101960214, 0.2534154318813515, 0.0726371962432468, 0.49113402834862235,
                   0.4911362895591663, 0.11898002646639905, 0.1501895966849679, 0.15021745070394746, 0.01754202138831551,
                   0.13059433732982026, 0.020496847079791814, 0.020512735422737065, 0.12692070216147233, 0.2900605373659263,
                   0.29364024694729224, 0.28099209587280943, 0.2945346732747498, 0.1390784801895961, 0.13718088501877865]

        lengths_med = [0.11221012660894233, 0.21838745393591003, 0.21838652247014215, 0.04009886721901377, 0.4404043065827791,
                       0.4404005018107258, 0.03352836109003122, 0.14624878028636093, 0.14626475496405048, 0.09201591904274159,
                       0.10279722790210173, 0.03407651369086422, 0.03408094322130216, 0.11318665800279451, 0.25693470642127425,
                       0.2624116078945138, 0.26616023158348917, 0.2692486717111876, 0.1390784801895961, 0.13718088501877865]

        initial_rots =  [[0.,           0.,          1.57079633],
                         [ 0.,          0.,          3.14159265],
                         [ 0.,          0.,          3.14159265],
                         [ 0.,          0.,          1.57079633],
                         [ 0.,          0.,          3.14159265],
                         [ 0.,          0.,          3.14159265],
                         [ 0.,          0.,          1.57079633],
                         [ 1.57079633,  0.,          0.        ],
                         [ 1.57079633,  0.,          0.        ],
                         [ 0.,          0.,          1.57079633],
                         [ 0.,          0.,          0.        ],
                         [ 0.,          0.,         -1.57079633],
                         [ 0.,          0.,          1.57079633],
                         [ 0.,          0.,          0.        ],
                         [ 0.,          0.,         -1.57079633],
                         [ 0.,          0.,          1.57079633],
                         [ 0.,          0.,         -1.57079633],
                         [ 0.,          0.,          1.57079633],
                         [ 0.,          0.,         -1.57079633],
                         [ 0.,          0.,          1.57079633]]

        cap_offsets =  [[0.0, -0.04, 0.0],
                        [0.011169376037587928, -0.15284656946574843, 0.012903663900451865],
                        [-0.008170654460734582, -0.14484365810893893, 0.009715733426505214],
                        [-0.004528416367088686, 0.013605244090683206, 0.04478019087771228],
                        [-0.010037273572433072, -0.23605176764955693, -0.016171902309391355],
                        [0.01309094764543177, -0.2308922510404945, -0.016202019999160024],
                        [-0.00591252018941266, -0.01317568744365618, -0.002010005222645302],
                        [0.01094751596360935, -0.03146133114580548, 0.07573397320647444],
                        [-0.012013303281514093, -0.033379105586368994, 0.07790374457555227],
                        [-0.0035521415314855153, 0.036178626564513906, -0.0033071152353626437],
                        [0.009242849296194584, 0.03218511724478738, 0.020464375843508388],
                        [0.07173393305388479, 0.03415988444216093, -0.021207023426983884],
                        [-0.07011977756584435, 0.03500467757275714, -0.01431098898502919],
                        [0.0017877181471058897, -0.003604552239007318, -0.013366571797805468],
                        [0.1670164481950132, -0.01047385968513933, -0.00955946729325003],
                        [-0.17489038653427352, -0.012404766368258179, -0.01493462681539015],
                        [0.13686313856829088, 0.012178408710815582, 0.009027286895816854],
                        [-0.14869936653600047, 0.012743380671612937, 0.010315014780206785],
                        [0.07295133648723572, 0.003332175704076991, 0.00477568727285764],
                        [-0.06900674634442043, 0.0053072850629911975, 0.005648618042937342]]

        joint_locs = [[0.048575008136347936, 0.024119849828711848, 0.0],
                      [0.05603581582384387, -0.08435470924277266, -0.02125171045905388],
                      [-0.056734287288310335, -0.09246840627063796, -0.019865012003098714],
                      [0.00478469354216688, 0.12540467056158064, -0.029426528753042178],
                      [0.05438855330810292, -0.43938729822764777, 0.010090597202908305],
                      [-0.05687942203278569, -0.4329298092709282, -0.002264318594297732],
                      [0.004335110280848386, 0.1424258678301017, 0.032243119346393],
                      [-0.013017916423372161, -0.47630985208241616, -0.03525983240121225],
                      [0.01879643192824705, -0.47063802124032617, -0.031908165182390605],
                      [-0.0037562386956846213, 0.057957653859279396, -0.0026084214232746233],
                      [-0.012891345804552689, 0.23382254793308366, -0.024546227595192645],
                      [0.06810633105445495, 0.12066967354888963, -0.025617194319459552],
                      [-0.08312624481554046, 0.1213346306423565, -0.02646990522515696],
                      [0.010160433355839844, 0.11431142528985228, 0.06561347872079941],
                      [0.11340919931011699, 0.04872322320654052, -0.014010241744952277],
                      [-0.10906087070470047, 0.050948984849806256, -0.0034195487756392123],
                      [0.2909446510941778, -0.020225692509760418, -0.016255943269017036],
                      [-0.2940904813191163, -0.024448930412666936, -0.02096928534590224],
                      [0.281254400830233, 0.011761597131565299, -0.011719547351094269],
                      [-0.2957180548455181, 0.014828733728343285, -0.00917540965007943],
                      [0.08686301274055253, -0.010243204490181151, -0.01588782879746012],
                      [-0.09127255488333685, -0.008439567905351025, -0.010547811064669982]]




        ########################################## SET UP DART ###########################################
        pydart.init(verbose=True)
        self.world = pydart.World(0.002, "EMPTY") #0.003, .0002
        self.world.set_gravity([0, 0, GRAVITY])#([0, 0,  -9.81])
        self.world.set_collision_detector(detector_type=2)


        ################## CREATE SKELETON FOR HUMAN, ADD TO WORLD, AND SET PARAMETERS ###################
        self.world.add_empty_skeleton(_skel_name="human")

        for count in range(NUM_CAPSULES):
            cap_rad = radii[count]
            cap_len = lengths[count]
            cap_init_rot = initial_rots[count]
            cap_offset = cap_offsets[count]
            joint_loc = joint_locs[count]

            if count == 0:
                self.world.add_capsule(parent=int(red_parent_ref[count]), radius=cap_rad, length=cap_len,
                                       cap_rot=cap_init_rot, cap_offset=cap_offset, joint_loc=joint_loc,
                                       joint_type="FREE", joint_name=joint_names[count])
            elif count == 4 or count == 5:
                self.world.add_capsule(parent=int(red_parent_ref[count]), radius=cap_rad, length=cap_len,
                                       cap_rot=cap_init_rot, cap_offset=cap_offset, joint_loc=joint_loc,
                                       joint_type="REVOLUTE_X", joint_name=joint_names[count])
            elif count == 16 or count == 17:
                self.world.add_capsule(parent=int(red_parent_ref[count]), radius=cap_rad, length=cap_len,
                                       cap_rot=cap_init_rot, cap_offset=cap_offset, joint_loc=joint_loc,
                                       joint_type="REVOLUTE_Y", joint_name=joint_names[count])
            else:
                self.world.add_capsule(parent=int(red_parent_ref[count]), radius=cap_rad, length=cap_len,
                                       cap_rot=cap_init_rot, cap_offset=cap_offset, joint_loc=joint_loc,
                                       joint_type="BALL", joint_name=joint_names[count])



        skel = self.world.add_built_skeleton(_skel_id=0, _skel_name="human")
        skel.set_self_collision_check(True)
        skel.set_adjacent_body_check(False)
        skel.test_filter()

        #blacklist things
        for i in range(NUM_CAPSULES):
            for j in range(NUM_CAPSULES):
                if i != j: skel.set_collision_filter(i, j, False)

        #set parameters
        skel = self.assign_joint_rest_and_stiffness(skel)
        skel = self.assign_joint_limits_and_damping(skel)
        skel = self.assign_body_masses_and_inertia(skel, initial_rots, radii, lengths, radii_med, lengths_med)



        ########################### CREATE SKELETON FOR FLOOR AND ADD TO WORLD ###########################
        self.world.add_empty_skeleton(_skel_name="floor")
        self.world.add_weld_box(width=10.0, length=10.0, height=0.2, joint_loc=[0.0, 0.0, -STARTING_HEIGHT / DART_TO_FLEX_CONV / 2 - 0.05],  box_rot=[0.0, 0.0, 0.0], joint_name="floor")  # -0.05
        skel_floor = self.world.add_built_skeleton(_skel_id=1, _skel_name="floor")







        #now setup the open GL window
        self.title = "GLUT Window"
        self.window_size = (1280, 720)
        self.scene = OpenGLScene(*self.window_size)

        self.mouseLastPos = None
        self.is_simulating = False
        self.is_animating = False
        self.frame_index = 0
        self.capture_index = 0

        self.force_application_count = 0
        self.count = 0




    def assign_body_masses_and_inertia(self, skel, initial_rots, radii, lengths, radii_med, lengths_med):

        # weight the capsules appropriately
        volume = []
        volume_median = []

        for count in range(20):
            cap_rad = radii[count]
            cap_rad_median = radii_med[count]
            cap_len = lengths[count]
            cap_len_median = lengths_med[count]
            volume.append(np.pi*np.square(cap_rad)*(cap_rad*4/3 + cap_len))
            volume_median.append(np.pi*np.square(cap_rad_median)*(cap_rad_median*4/3 + cap_len_median))



        volume_torso = volume[0] + volume[3] + volume[6] + volume[9] + volume[11] + volume[12]
        volume_head = volume[10] + volume[13]
        body_masses =  [72.6 * 0.4830 * (volume[0]/volume_torso) * (volume[0]/volume_median[0]),
                        72.6 * 0.1050 * (volume[1]/volume_median[1]),
                        72.6 * 0.1050 * (volume[2]/volume_median[2]),
                        72.6 * 0.4830 * (volume[3]/volume_torso) * (volume[3]/volume_median[3]),
                        72.6 * 0.0450 * (volume[4]/volume_median[4]),
                        72.6 * 0.0450 * (volume[5]/volume_median[5]),
                        72.6 * 0.4830 * (volume[6]/volume_torso) * (volume[6]/volume_median[6]),
                        72.6 * 0.0150 * (volume[7]/volume_median[7]),
                        72.6 * 0.0150 * (volume[8]/volume_median[8]),
                        72.6 * 0.4830 * (volume[9]/volume_torso) * (volume[9]/volume_median[9]),
                        72.6 * 0.0710 * (volume[10]/volume_head) * (volume[10]/volume_median[10]),
                        72.6 * 0.4830 * (volume[11]/volume_torso) * (volume[11]/volume_median[11]),
                        72.6 * 0.4830 * (volume[12]/volume_torso * (volume[12]/volume_median[12])),
                        72.6 * 0.0710 * (volume[13]/volume_head) * (volume[13]/volume_median[13]),
                        72.6 * 0.0330 * (volume[14]/volume_median[14]),
                        72.6 * 0.0330 * (volume[15]/volume_median[15]),
                        72.6 * 0.0190 * (volume[16]/volume_median[16]),
                        72.6 * 0.0190 * (volume[17]/volume_median[17]),
                        72.6 * 0.0060 * (volume[18]/volume_median[18]),
                        72.6 * 0.0060 * (volume[19]/volume_median[19])]

        body_mass = 0.0
        #set the mass moment of inertia matrices
        for body_ct in range(NUM_CAPSULES):
            skel.bodynodes[0].set_mass(body_masses[body_ct])

            radius = radii[body_ct]
            length = lengths[body_ct]
            radius2 = radius * radius
            length2 = length * length
            mass = skel.bodynodes[body_ct].m

            cap_init_rot = list(np.asarray(initial_rots[body_ct]))

            volumeCylinder = np.pi*radius2*length
            volumeSphere = np.pi*radius*radius*radius*4/3

            density = mass / (volumeCylinder + volumeSphere)
            massCylinder = density * volumeCylinder
            massSphere = density * volumeSphere
            Ixx = massCylinder * (length2 / 12.0 + radius2 / 4.0) + massSphere * (length2 + (3.0 / 8.0) * length * radius + (2.0 / 5.0) * radius2)
            Izz = massCylinder * (radius2 / 2.0) + massSphere * ((2.0 / 5.0) * radius2)

            RotMatInit = self.eulerAnglesToRotationMatrix([np.pi/2, 0.0, 0.0])
            RotMat = self.eulerAnglesToRotationMatrix(cap_init_rot)
            I = np.matmul(np.matmul(RotMatInit, RotMat), np.asarray([ Ixx, Izz, Ixx]))
            Ixx = np.abs(I[0])
            Iyy = np.abs(I[1])
            Izz = np.abs(I[2])
            #print body_ct, I

            skel.bodynodes[body_ct].set_inertia_entries(Ixx, Iyy, Izz)

            body_mass += skel.bodynodes[body_ct].m

        return skel

    def assign_joint_rest_and_stiffness(self, skel):

        ################################# ASSIGN JOINT REST POSITION AND SPRING COEFF ##################################
        r_arm_stiffness = 1.0
        l_arm_stiffness = 1.0
        head_stiffness = 20.0
        r_leg_stiffness = 10.0
        l_leg_stiffness = 10.0
        r_knee_stiffness = 10.0
        l_knee_stiffness = 10.0
        torso_stiffness = 200.0


        for joint in skel.joints:
            #print joint.spring_stiffness(0)
            if joint.name == "leftThigh":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, l_leg_stiffness)
                joint.set_spring_stiffness(1, l_leg_stiffness)
                joint.set_spring_stiffness(2, l_leg_stiffness)
            elif joint.name == "rightThigh":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, r_leg_stiffness)
                joint.set_spring_stiffness(1, r_leg_stiffness)
                joint.set_spring_stiffness(2, r_leg_stiffness)
            elif joint.name == "spine":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, torso_stiffness)
                joint.set_spring_stiffness(1, torso_stiffness)
                joint.set_spring_stiffness(2, torso_stiffness)
            elif joint.name == "leftCalf":
                joint.set_rest_position(0, 0.0)
                joint.set_spring_stiffness(0, l_knee_stiffness)
            elif joint.name == "rightCalf":
                joint.set_rest_position(0, 0.0)
                joint.set_spring_stiffness(0, r_knee_stiffness)
            elif joint.name == "spine1":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, torso_stiffness)
                joint.set_spring_stiffness(1, torso_stiffness)
                joint.set_spring_stiffness(2, torso_stiffness)
            elif joint.name == "leftFoot":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, l_leg_stiffness)
                joint.set_spring_stiffness(1, l_leg_stiffness)
                joint.set_spring_stiffness(2, l_leg_stiffness)
            elif joint.name == "rightFoot":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, r_leg_stiffness)
                joint.set_spring_stiffness(1, r_leg_stiffness)
                joint.set_spring_stiffness(2, r_leg_stiffness)
            elif joint.name == "spine2":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, torso_stiffness)
                joint.set_spring_stiffness(1, torso_stiffness)
                joint.set_spring_stiffness(2, torso_stiffness)
            elif joint.name == "neck":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, head_stiffness)
                joint.set_spring_stiffness(1, head_stiffness)
                joint.set_spring_stiffness(2, head_stiffness)
            elif joint.name == "leftShoulder":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, l_arm_stiffness)
                joint.set_spring_stiffness(1, l_arm_stiffness)
                joint.set_spring_stiffness(2, l_arm_stiffness)
            elif joint.name == "rightShoulder":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, r_arm_stiffness)
                joint.set_spring_stiffness(1, r_arm_stiffness)
                joint.set_spring_stiffness(2, r_arm_stiffness)
            elif joint.name == "head":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, head_stiffness)
                joint.set_spring_stiffness(1, head_stiffness)
                joint.set_spring_stiffness(2, head_stiffness)
            elif joint.name == "leftUpperArm":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, l_arm_stiffness)
                joint.set_spring_stiffness(1, l_arm_stiffness)
                joint.set_spring_stiffness(2, l_arm_stiffness)
            elif joint.name == "rightUpperArm":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, r_arm_stiffness)
                joint.set_spring_stiffness(1, r_arm_stiffness)
                joint.set_spring_stiffness(2, r_arm_stiffness)
            elif joint.name == "leftForeArm":
                joint.set_rest_position(0, 0.0)
                joint.set_spring_stiffness(0, l_arm_stiffness)
            elif joint.name == "rightForeArm":
                joint.set_rest_position(0, 0.0)
                joint.set_spring_stiffness(0, r_arm_stiffness)
            elif joint.name == "leftHand":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, l_arm_stiffness)
                joint.set_spring_stiffness(1, l_arm_stiffness)
                joint.set_spring_stiffness(2, l_arm_stiffness)
            elif joint.name == "rightHand":
                joint.set_rest_position(0, 0.0)
                joint.set_rest_position(1, 0.0)
                joint.set_rest_position(2, 0.0)
                joint.set_spring_stiffness(0, r_arm_stiffness)
                joint.set_spring_stiffness(1, r_arm_stiffness)
                joint.set_spring_stiffness(2, r_arm_stiffness)

        return skel



    def assign_joint_limits_and_damping(self, skel):
        ######################################## ASSIGN JOINT LIMITS AND DAMPING #######################################

        arm_damping = 1.0  # 5.0
        shoulder_damping = 1.0
        head_damping = 5.0  # 10.0
        leg_damping = 5.0  # 15.0
        knee_damping = 5.0  # 50.0
        torso_damping = 20.0  # 75.0

        for joint in skel.joints:

            if joint.name == "leftThigh":
                joint.set_position_lower_limit(0, -2.047187297216041)  # ext
                joint.set_position_upper_limit(0, 0.0008725992352640336)
                joint.set_position_lower_limit(1, -1.0056561780573234)  # yaw
                joint.set_position_upper_limit(1, 0.9792596381050885)
                joint.set_position_lower_limit(2, -0.83127128871961)  # abd
                joint.set_position_upper_limit(2, 0.9840833280290882)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, leg_damping)
                joint.set_damping_coefficient(1, leg_damping)
                joint.set_damping_coefficient(2, leg_damping)
            elif joint.name == "rightThigh":
                joint.set_position_lower_limit(0, -2.0467908364949654)  # ext
                joint.set_position_upper_limit(0, 0.005041331594134009)
                joint.set_position_lower_limit(1, -0.9477521700520728)  # yaw
                joint.set_position_upper_limit(1, 1.0038579032816006)
                joint.set_position_lower_limit(2, -0.8767199629302654)  # abd
                joint.set_position_upper_limit(2, 0.35738032396710084)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, leg_damping)
                joint.set_damping_coefficient(1, leg_damping)
                joint.set_damping_coefficient(2, leg_damping)
            elif joint.name == "leftCalf":
                joint.set_position_lower_limit(0, 0.0)
                joint.set_position_upper_limit(0, 2.307862756803765)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, knee_damping)
            elif joint.name == "rightCalf":
                joint.set_position_lower_limit(0, 0.0)
                joint.set_position_upper_limit(0, 2.320752282574325)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, knee_damping)
            elif joint.name == "leftShoulder":
                joint.set_position_lower_limit(0, -1.7636153960682888 * 1 / 3)  # roll
                joint.set_position_upper_limit(0, 1.5740500958475525 * 1 / 3)
                joint.set_position_lower_limit(1, -1.5168279883317557 * 1 / 3)  # yaw
                joint.set_position_upper_limit(1, 1.6123857573735045 * 1 / 3)
                joint.set_position_lower_limit(2, -1.7656139149798185 * 1 / 3)  # pitch
                joint.set_position_upper_limit(2, 1.9844820788036448 * 1 / 3)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, shoulder_damping)
                joint.set_damping_coefficient(1, shoulder_damping)
                joint.set_damping_coefficient(2, shoulder_damping)
            elif joint.name == "leftUpperArm":
                joint.set_position_lower_limit(0, -1.7636153960682888 * 2 / 3)  # roll
                joint.set_position_upper_limit(0, 1.5740500958475525 * 2 / 3)
                joint.set_position_lower_limit(1, -1.5168279883317557 * 2 / 3)  # yaw
                joint.set_position_upper_limit(1, 1.6123857573735045 * 2 / 3)
                joint.set_position_lower_limit(2, -1.7656139149798185 * 2 / 3)  # pitch
                joint.set_position_upper_limit(2, 1.9844820788036448 * 2 / 3)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, arm_damping)
                joint.set_damping_coefficient(1, arm_damping)
                joint.set_damping_coefficient(2, arm_damping)
            elif joint.name == "rightShoulder":
                joint.set_position_lower_limit(0, -1.8819412381973686 * 1 / 3)  # roll
                joint.set_position_upper_limit(0, 1.5386423137579994 * 1 / 3)
                joint.set_position_lower_limit(1, -1.6424506514942065 * 1 / 3)  # yaw
                joint.set_position_upper_limit(1, 2.452871806175492 * 1 / 3)
                joint.set_position_lower_limit(2, -1.9397148210114974 * 1 / 3)  # pitch
                joint.set_position_upper_limit(2, 1.8997886932520462 * 1 / 3)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, shoulder_damping)
                joint.set_damping_coefficient(1, shoulder_damping)
                joint.set_damping_coefficient(2, shoulder_damping)
            elif joint.name == "rightUpperArm":
                joint.set_position_lower_limit(0, -1.8819412381973686 * 2 / 3)  # roll
                joint.set_position_upper_limit(0, 1.5386423137579994 * 2 / 3)
                joint.set_position_lower_limit(1, -1.6424506514942065 * 2 / 3)  # yaw
                joint.set_position_upper_limit(1, 2.452871806175492 * 2 / 3)
                joint.set_position_lower_limit(2, -1.9397148210114974 * 2 / 3)  # pitch
                joint.set_position_upper_limit(2, 1.8997886932520462 * 2 / 3)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, arm_damping)
                joint.set_damping_coefficient(1, arm_damping)
                joint.set_damping_coefficient(2, arm_damping)
            elif joint.name == "leftForeArm":
                joint.set_position_lower_limit(0, -2.146677709782182)
                joint.set_position_upper_limit(0, 0.0)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, arm_damping)
            elif joint.name == "rightForeArm":
                joint.set_position_lower_limit(0, 0.0)
                joint.set_position_upper_limit(0, 2.136934895040784)
                joint.set_position_limit_enforced(True)
                joint.set_damping_coefficient(0, arm_damping)
            elif joint.name == "leftHand" or joint.name == "rightHand":
                joint.set_damping_coefficient(0, arm_damping)
                joint.set_damping_coefficient(1, arm_damping)
                joint.set_damping_coefficient(2, arm_damping)
            elif joint.name == "neck" or joint.name == "head":
                joint.set_damping_coefficient(0, head_damping)
                joint.set_damping_coefficient(1, head_damping)
                joint.set_damping_coefficient(2, head_damping)
            else:
                joint.set_damping_coefficient(0, torso_damping)
                joint.set_damping_coefficient(1, torso_damping)
                joint.set_damping_coefficient(2, torso_damping)

        return skel

    def eulerAnglesToRotationMatrix(self, theta):
        R_x = np.array([[1, 0, 0],
                        [0, math.cos(theta[0]), -math.sin(theta[0])],
                        [0, math.sin(theta[0]), math.cos(theta[0])]
                        ])

        R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                        [0, 1, 0],
                        [-math.sin(theta[1]), 0, math.cos(theta[1])]
                        ])

        R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                        [math.sin(theta[2]), math.cos(theta[2]), 0],
                        [0, 0, 1]
                        ])

        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

        #print "joint locs",
    def destroyWorld(self):
        self.world.destroy()

    def initGL(self, w, h):
        self.scene.init()

    def resizeGL(self, w, h):
        self.scene.resize(w, h)

    def drawGL(self, ):
        self.scene.render(self.world)
        # GLUT.glutSolidSphere(0.3, 20, 20)  # Default object for debugging
        GLUT.glutSwapBuffers()

    # The function called whenever a key is pressed.
    # Note the use of Python tuples to pass in: (key, x, y)
    def keyPressed(self, key, x, y):
        keycode = ord(key)
        key = key.decode('utf-8')
        # print("key = [%s] = [%d]" % (key, ord(key)))

        # n = sim.num_frames()
        if keycode == 27:
            GLUT.glutDestroyWindow(self.window)
            sys.exit()
        elif key == ' ':
            self.is_simulating = not self.is_simulating
            self.is_animating = False
            print("self.is_simulating = %s" % self.is_simulating)
        elif key == 'a':
            self.is_animating = not self.is_animating
            self.is_simulating = False
            print("self.is_animating = %s" % self.is_animating)
        elif key == ']':
            self.frame_index = (self.frame_index + 1) % self.world.num_frames()
            print("frame = %d/%d" % (self.frame_index, self.world.num_frames()))
            if hasattr(self.world, "set_frame"):
                self.world.set_frame(self.frame_index)
        elif key == '[':
            self.frame_index = (self.frame_index - 1) % self.world.num_frames()
            print("frame = %d/%d" % (self.frame_index, self.world.num_frames()))
            if hasattr(self.world, "set_frame"):
                self.world.set_frame(self.frame_index)
        elif key == 'c':
            self.capture()

    def mouseFunc(self, button, state, x, y):
        if state == 0:  # Mouse pressed
            self.mouseLastPos = np.array([x, y])
        elif state == 1:
            self.mouseLastPos = None

    def motionFunc(self, x, y):
        dx = x - self.mouseLastPos[0]
        dy = y - self.mouseLastPos[1]
        modifiers = GLUT.glutGetModifiers()
        tb = self.scene.tb
        if modifiers == GLUT.GLUT_ACTIVE_SHIFT:
            tb.zoom_to(dx, -dy)
        elif modifiers == GLUT.GLUT_ACTIVE_CTRL:
            tb.trans_to(dx, -dy)
        else:
            tb.drag_to(x, y, dx, -dy)
        self.mouseLastPos = np.array([x, y])

    def idle(self):
        if self.world is None:
            return

        #if self.count == self.num_steps: self.is_simulating = False



        if self.is_simulating:
            self.count += 1
            self.world.step()
            print "did a step"
            self.world.check_collision()

            time_orig = time()



            print "appending time", time() - time_orig

            self.force_application_count += 1



        elif self.is_animating:
            self.frame_index = (self.frame_index + 1) % self.world.num_frames()
            if hasattr(self.world, "set_frame"):
                self.world.set_frame(self.frame_index)

    def renderTimer(self, timer):
        GLUT.glutPostRedisplay()
        GLUT.glutTimerFunc(20, self.renderTimer, 1)

    def capture(self, ):
        print("capture! index = %d" % self.capture_index)
        from PIL import Image
        GL.glPixelStorei(GL.GL_PACK_ALIGNMENT, 1)
        w, h = 1280, 720
        data = GL.glReadPixels(0, 0, w, h, GL.GL_RGBA, GL.GL_UNSIGNED_BYTE)
        img = Image.fromstring("RGBA", (w, h), data)
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        filename = "./data/captures/capture%04d.png" % self.capture_index
        img.save(filename, 'png')
        self.capture_index += 1

    def run_sim_with_window(self):
        print("\n")
        print("space bar: simulation on/off")
        print("' ': run/stop simulation")
        print("'a': run/stop animation")
        print("'[' and ']': play one frame backward and forward")

        # Init glut
        GLUT.glutInit(())
        GLUT.glutInitDisplayMode(GLUT.GLUT_RGBA |
                                 GLUT.GLUT_DOUBLE |
                                 GLUT.GLUT_MULTISAMPLE |
                                 GLUT.GLUT_ALPHA |
                                 GLUT.GLUT_DEPTH)
        GLUT.glutInitWindowSize(*self.window_size)
        GLUT.glutInitWindowPosition(0, 0)
        self.window = GLUT.glutCreateWindow(self.title)

        # Init functions
        # glutFullScreen()
        GLUT.glutDisplayFunc(self.drawGL)
        GLUT.glutIdleFunc(self.idle)
        GLUT.glutReshapeFunc(self.resizeGL)
        GLUT.glutKeyboardFunc(self.keyPressed)
        GLUT.glutMouseFunc(self.mouseFunc)
        GLUT.glutMotionFunc(self.motionFunc)
        GLUT.glutTimerFunc(25, self.renderTimer, 1)
        self.initGL(*self.window_size)

        # Run
        GLUT.glutMainLoop()


    def run_simulation(self, num_steps = 100):
        self.num_steps = num_steps
        #pydart.gui.viewer.launch(world)

        default_camera = None
        if default_camera is not None:
            self.scene.set_camera(default_camera)
        self.run_sim_with_window()


if __name__ == '__main__':
    dss = DartSkelSim()
    dss.run_simulation()
