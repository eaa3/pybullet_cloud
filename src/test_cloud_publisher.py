#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, time
import pybullet as p
from image_getter import ImageGetter
import atexit
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == "__main__":
    ##########################################################
    " Visualization by using rviz"
    import rospy, tf
    from point_cloud_publisher import PointCloudPublisher

    import subprocess, time
    # roscore = subprocess.Popen('roscore', shell=True)
    # atexit.register(roscore.kill)
    time.sleep(2)
    rviz = subprocess.Popen('rviz', shell=True)
    atexit.register(rviz.kill)
    time.sleep(2)

    
    
    

    print("Initializing node... ")
    rospy.init_node('bullet_point_cloud')
    pcl_node = PointCloudPublisher()
    #pcl_node.run_thread()

    br = tf.TransformBroadcaster()
    ##########################################################

    model_dir = os.getcwd() # Hack
    pcid = p.connect(p.GUI)
    if pcid < 0:
        pcid = pb.connect(pb.UDP, "127.0.0.1")
    # pcid = pb.connect(pb.UDP, "127.0.0.1")
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=pcid)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0, physicsClientId=pcid)
    p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=90, cameraPitch=-15,
                                 cameraTargetPosition=[0.5, -0.35, 0.2], physicsClientId=pcid)

    p.resetSimulation(physicsClientId=pcid)
    p.setGravity(0, 0, -9.81, physicsClientId=pcid)
    p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=pcid)
    time_step = 1./200.
    # p.setTimeStep(time_step, physicsClientId=pcid)
    p.setRealTimeSimulation(1)
    p.setTimeStep(0.01)

    p.loadURDF(model_dir + '/models/table/table.urdf', [0.5,0.0,-0.355],
               useFixedBase=True, physicsClientId=pcid)


    # object_id = p.loadURDF(model_dir + '/models/object_models/cylinder.urdf',
    #                    useFixedBase=False, physicsClientId=pcid)
    # object_id = p.loadURDF(model_dir + '/models/object_models/urdf/bowl.urdf',
    #                    useFixedBase=False, physicsClientId=pcid)

    object_id = p.loadURDF(model_dir + '/models/object_models/cube_small.urdf',
                       useFixedBase=False, physicsClientId=pcid)
    p.resetBasePositionAndOrientation(object_id,[0.5,0.6,-0.32],[0,0,0,1], physicsClientId=pcid)

    image_getter = ImageGetter(pcid)
    image_getter.getImage()

    loop_count = 0

    # p.setRealTimeSimulation(1)
    # p.setTimeStep(0.01)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        # print loop_count
        if loop_count%1 == 0:


            image_getter.getImage()
            xyz_rgb = image_getter.createPointCloud()

            ##########################################################
            " Visualization by using rviz"
            pcl_node.update(xyz_rgb)
            pcl_node.publish()
            br.sendTransform(image_getter.camera_pos,
                             image_getter.camera_quat,
                             rospy.Time.now(),
                             "bullet_camera", "world")
            ##########################################################

        #p.stepSimulation(physicsClientId=pcid)
        rate.sleep()
        loop_count += 1
