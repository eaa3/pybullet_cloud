#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, time
import pybullet as p
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
    pcl_node.run_thread()

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
    p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=pcid)
    time_step = 1./200.
    p.setTimeStep(time_step, physicsClientId=pcid)

    p.loadURDF(model_dir + '/models/table/table.urdf', [0.5, -0.4, -0.075],
               useFixedBase=True, physicsClientId=pcid)

    robot = p.loadURDF(model_dir + '/models/kinova/kinova.urdf',
                       useFixedBase=True, physicsClientId=pcid)

    home_joints = [4.8046852, 2.92482, 1.002, 4.2031852, 1.4458, 1.3233, 0.0 ,0.0, 0.0]

    active_joints_id = []
    gui_slider_id = []
    cnt = 0
    for i in range (p.getNumJoints(robot, physicsClientId=pcid)):
        if p.JOINT_FIXED is not p.getJointInfo(robot, i, physicsClientId=pcid)[2] :
            name = p.getJointInfo(robot, i)[1].decode("utf-8")
            gui_slider_id.append(p.addUserDebugParameter(str(name) + " (" + str(i) + ")",
                                                         p.getJointInfo(robot, i)[8],
                                                         p.getJointInfo(robot, i)[9],
                                                         home_joints[cnt],
                                                         physicsClientId=pcid))
            active_joints_id.append(i)
            cnt += 1

    image_getter = ImageGetter(pcid)
    image_getter.getImage()

    loop_count = 0
    while True:
        for i in gui_slider_id:
            p.resetJointState(robot, active_joints_id[i], p.readUserDebugParameter(i),
                              physicsClientId=pcid)

        if loop_count%10 == 0:
            image_getter.getImage()
            xyz_rgb = image_getter.createPointCloud()

            ##########################################################
            " Visualization by using rviz"
            pcl_node.update(xyz_rgb)
            br.sendTransform(image_getter.camera_pos,
                             image_getter.camera_quat,
                             rospy.Time.now(),
                             "bullet_camera", "world")
            ##########################################################

        p.stepSimulation(physicsClientId=pcid)
        time.sleep(time_step)
        loop_count += 1
