#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, time
import pybullet as p
import numpy as np
from image_getter import ImageGetter

import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == "__main__":
    ##########################################################
    " Visualization by using rviz"
    import rospy, tf
    from point_cloud_publisher import PointCloudPublisher

    import subprocess, time
    roscore = subprocess.Popen('roscore')
    time.sleep(2)
    rviz = subprocess.Popen('rviz')
    time.sleep(2)

    print("Initializing node... ")
    rospy.init_node('bullet_point_cloud')
    pcl_node = PointCloudPublisher()
    #pcl_node.run_thread()

    br = tf.TransformBroadcaster()
    ##########################################################

    model_dir = os.getcwd() # Hack
    pcid = p.connect(p.GUI)

    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=pcid)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0, physicsClientId=pcid)
    p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=90, cameraPitch=-15,
                                 cameraTargetPosition=[0.5, -0.35, 0.2], physicsClientId=pcid)

    p.resetSimulation(physicsClientId=pcid)
    p.setGravity(0, 0, -9.81, physicsClientId=pcid)
    p.setPhysicsEngineParameter(numSolverIterations=1000, physicsClientId=pcid)
    time_step = 1./200.
    p.setTimeStep(time_step, physicsClientId=pcid)

    p.loadURDF(model_dir + '/models/table/table.urdf', [0.5,0.0,-0.33],
               useFixedBase=True, physicsClientId=pcid)
    object_id = p.loadURDF(model_dir + '/models/object_models/urdf/bowl.urdf',
                       useFixedBase=False, physicsClientId=pcid)

    p.resetBasePositionAndOrientation(object_id,[0.5,0.6,-0.32],[0,0,0,1], physicsClientId=pcid)

    robot = p.loadURDF(model_dir + '/models/boris/boris.urdf',
                       useFixedBase=True, physicsClientId=pcid)

    home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0]

    active_joints_id = []
    gui_slider_id = []
    cnt = 0

    robot_joint_names =[ 'left_arm_0_joint', 'left_arm_1_joint', 'left_arm_2_joint',
                        'left_arm_3_joint', 'left_arm_4_joint', 'left_arm_5_joint',
                        'left_arm_6_joint', 'left_hand_synergy_joint']
    mimic_joint_names = []
    hand_joint_names = []
    hand_abd_joint_names = []

    mimic_joint_ids = []
    hand_joint_ids = []
    hand_abd_joint_ids = []
    for i in range (p.getNumJoints(robot, physicsClientId=pcid)):
        if p.JOINT_FIXED is not p.getJointInfo(robot, i, physicsClientId=pcid)[2] :
            name = p.getJointInfo(robot, i)[1].decode("utf-8")
            # print "Joint name [%d]: %s"%(i,name)
            if "mimic" in name:
                mimic_joint_names.append(name)
                mimic_joint_ids.append(i)
            elif "hand" in name and not "synergy" in name and (("abd" in name and "thumb" in name) or (not "abd" in name)):
                hand_joint_names.append(name)
                hand_joint_ids.append(i)
            elif "abd" in name:
                print "Joint: ", name
                hand_abd_joint_names.append(name)
                hand_abd_joint_ids.append(i)

            elif name in robot_joint_names:
                gui_slider_id.append(p.addUserDebugParameter(str(name) + " (" + str(i) + ")",
                                                            p.getJointInfo(robot, i)[8],
                                                            p.getJointInfo(robot, i)[9],
                                                            home_joints[cnt],
                                                            physicsClientId=pcid))
                active_joints_id.append(i)
                cnt += 1

    print "Mimic joints: ", mimic_joint_names
    print "Hand joints: ", hand_joint_names
    image_getter = ImageGetter(pcid)
    image_getter.getImage()

    loop_count = 0
    while True:
        for i in gui_slider_id:
            p.resetJointState(robot, active_joints_id[i], p.readUserDebugParameter(i),
                              physicsClientId=pcid)
            
            if i == gui_slider_id[-1]:
                for hid in hand_joint_ids:
                    p.resetJointState(robot, hid, p.readUserDebugParameter(i),
                                    physicsClientId=pcid)
                for hid in hand_abd_joint_ids:
                    p.resetJointState(robot, hid, 0.0,
                                    physicsClientId=pcid)


                n_joints = len(hand_joint_ids)
                vels = [0.0 for n in range(n_joints)]
                forces = np.ones(n_joints) * 6.0
                positionGains = np.ones(n_joints) * 3.0
                velocityGains = np.ones(n_joints) * 0.15
                cmd = [p.readUserDebugParameter(i)]*n_joints
                p.setJointMotorControlArray(robot, hand_joint_ids, controlMode=p.POSITION_CONTROL,
                                            targetPositions=cmd, targetVelocities=vels,
                                            forces=forces,
                                            positionGains=positionGains,
                                            velocityGains=velocityGains,
                                            physicsClientId=pcid)

                n_joints = len(hand_abd_joint_ids)
                vels = [0.0 for n in range(n_joints)]
                forces = np.ones(n_joints) * 6.0
                positionGains = np.ones(n_joints) * 3.0
                velocityGains = np.ones(n_joints) * 0.15
                cmd = [0.0]*n_joints
                p.setJointMotorControlArray(robot, hand_abd_joint_ids, controlMode=p.POSITION_CONTROL,
                                            targetPositions=cmd, targetVelocities=vels,
                                            forces=forces,
                                            positionGains=positionGains,
                                            velocityGains=velocityGains,
                                            physicsClientId=pcid)

                
                    #joint_states = p.getJointStates(robot, mimic_joint_ids, physicsClientId=pcid)
                #angles = [ s[0] for s in joint_states]
                #print angles
        if loop_count%20 == 0:
            

            image_getter.getImage()
            
            xyz_rgb = image_getter.createPointCloud()

            ##########################################################
            # " Visualization by using rviz"
            t0 = time.time()
            pcl_node.update(xyz_rgb)
            print "Cloud processing time: ", (time.time()-t0,)
            pcl_node.publish()
            br.sendTransform(image_getter.camera_pos,
                             image_getter.camera_quat,
                             rospy.Time.now(),
                             "bullet_camera", "world")
            ##########################################################
            
        p.stepSimulation(physicsClientId=pcid)
        time.sleep(time_step)
        loop_count += 1
