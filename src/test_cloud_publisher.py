#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os, time, subprocess
import pybullet as p
from image_getter import ImageGetter
import atexit
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL)
from kbhit import KBHit
from std_msgs.msg import Int32, String


class BulletSimulation(object):

    def __init__(self, time_step = 0.01):

        self.time_step = time_step

        self.pcid = p.connect(p.DIRECT)
        if self.pcid < 0:
            self.pcid = p.connect(pb.UDP, "127.0.0.1")
        # pcid = p.connect(pb.UDP, "127.0.0.1")

        self.object_id = None

        model_dir = os.getcwd() # Hack
        self.object_path = model_dir + '/models/object_models/cube_small.urdf'
        self.table_path = model_dir + '/models/table/table.urdf'

    def reset(self, object_path = None):
        
        if object_path is not None:
            self.object_path = object_path

        

        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=self.pcid)
        p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0, physicsClientId=self.pcid)
        p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=90, cameraPitch=-15,
                                    cameraTargetPosition=[0.5, -0.35, 0.2], physicsClientId=self.pcid)

        p.resetSimulation(physicsClientId=self.pcid)
        p.setGravity(0, 0, -9.81, physicsClientId=self.pcid)
        # p.setPhysicsEngineParameter(numSolverIterations=150, physicsClientId=self.pcid)
        
        p.setTimeStep(self.time_step, physicsClientId=self.pcid)
        p.setRealTimeSimulation(1)

        p.loadURDF(self.table_path, [0.5,0.0,-0.355],
                useFixedBase=True, physicsClientId=self.pcid)


        # self.object_id = p.loadURDF(model_dir + '/models/object_models/cylinder.urdf',
        #                    useFixedBase=False, physicsClientId=pcid)
        # self.object_id = p.loadURDF(model_dir + '/models/object_models/urdf/bowl.urdf',
        #                    useFixedBase=False, physicsClientId=pcid)

        self.object_id = p.loadURDF(self.object_path,
                        useFixedBase=False, physicsClientId=self.pcid)
        p.resetBasePositionAndOrientation(self.object_id,[0.5,0.6,-0.29],[0,0,0,1], physicsClientId=self.pcid)


    def step(self):
        
        p.stepSimulation(physicsClientId=self.pcid)

    
    def get_id(self):
        return self.pcid



if __name__ == "__main__":
    ##########################################################
    " Visualization by using rviz"
    import rospy, tf
    from point_cloud_publisher import PointCloudPublisher


    # roscore = subprocess.Popen('roscore', shell=True)
    # atexit.register(roscore.kill)
    # time.sleep(2)
    # rviz = subprocess.Popen('rviz', shell=True)
    # atexit.register(rviz.kill)
    # time.sleep(2)

    
    keyboard = KBHit()
    

    print("Initializing node... ")
    rospy.init_node('bullet_point_cloud')
    pcl_node = PointCloudPublisher()
    #pcl_node.run_thread()

    br = tf.TransformBroadcaster()
    ##########################################################

    simulation = BulletSimulation(0.01)
    simulation.reset()

    image_getter = ImageGetter(simulation.get_id())
    image_getter.getImage()

    def next_camera_callback(msg):
        idx = int(msg.data)
        rospy.loginfo("Next camera: %d"%(idx,))
        image_getter.setCameraPose(idx)

    def next_object_callback(msg):
        object_path = str(msg.data)
        rospy.loginfo("Next object path: %s"%(object_path,))

        simulation.reset(object_path)
        image_getter.setCameraPose(0)

    next_camera_sub = rospy.Subscriber("sim/next_camera/", Int32, next_camera_callback, queue_size=None)
    next_object_sub = rospy.Subscriber("sim/next_object/", String, next_object_callback, queue_size=None)

    rate = rospy.Rate(30)

    rospy.loginfo("Running...")
    while not rospy.is_shutdown():
        # print loop_count

        t0 = time.time()
        image_getter.getImage()
        xyz_rgb = image_getter.createPointCloud()
        ##########################################################
        #" Visualization by using rviz"
        stamp = rospy.Time.now()
        pcl_node.update(xyz_rgb)
        pcl_node.publish(stamp)
        br.sendTransform(image_getter.camera_pos,
                             image_getter.camera_quat,
                             stamp,
                             "bullet_camera", "world")
        ##########################################################

        rospy.logdebug("Cloud creation time: %f"%(time.time()-t0))

        if keyboard.is_pressed('n'):
            image_getter.nextCameraPose()
        elif keyboard.is_pressed('b'):
            image_getter.previousCameraPose()
        elif keyboard.is_pressed('r'):
            simulation.reset()

        simulation.step()
        rate.sleep()


