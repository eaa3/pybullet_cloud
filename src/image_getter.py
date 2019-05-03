#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import pybullet as p
import tf.transformations as tt
import matplotlib.pyplot as plt
from scipy.stats import norm
import time

def addDebugFrame(pos, quat, length=0.2, lw=2, duration=0.2, pcid=0):
    rot_mat = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3).transpose()
    toX = pos + length * rot_mat[0, :]
    toY = pos + length * rot_mat[1, :]
    toZ = pos + length * rot_mat[2, :]
    item_id = [0, 0, 0]
    item_id[0] = p.addUserDebugLine(pos, toX, [1, 0, 0], lw, duration, physicsClientId=pcid)
    item_id[1] = p.addUserDebugLine(pos, toY, [0, 1, 0], lw, duration, physicsClientId=pcid)
    item_id[2] = p.addUserDebugLine(pos, toZ, [0, 0, 1], lw, duration, physicsClientId=pcid)
    return item_id

class CameraPose(object):

    def __init__(self, position, rpy, distance):
        self.position = position
        self.rpy = rpy
        self.distance = distance

def normal_random_integer(samples = 1, mean = 0, std = 1):
    
    return norm.ppf(np.random.random(samples), loc=mean, scale=std).astype(int)

class ImageGetter(object):
    def __init__(self, pcid):
        self._id = pcid

        self.camera_poses = []
        self.camera_poses.append(CameraPose([0.3, 0.0, 0.5],[0.0, -30.0, 90.0],1.5))
        self.camera_poses.append(CameraPose([0.50,0.35,0.20],[0.0, 4.60, 168.0],1.5))
        self.camera_poses.append(CameraPose([1.0,-0.02,-1.31],[0.0, -46.6, 214.0],1.9))
        self.camera_poses.append(CameraPose([0.6,0.62,-0.55],[0.0, -39.0, 320.0],1.5))

        self.camera_idx = 0

        camera_pose = self.camera_poses[self.camera_idx]

        self.camTargetPos = camera_pose.position#[0.3, 0.0, 0.5]
        self.roll = camera_pose.rpy[0]
        self.pitch = camera_pose.rpy[1]
        self.yaw = camera_pose.rpy[2]
        
        self.camDistance = camera_pose.distance

        self.pixelWidth = 640
        self.pixelHeight = 640
        self.aspect = self.pixelWidth / self.pixelHeight

        self.nearPlane = 0.01
        self.farPlane = 5.0
        self.fov = 57.0
        # Primesense carmine 1.09 field of view
        #horizontal FOV of 57.5 degrees and vertical FOV of 45 degrees.

        self.xyz_rgb = np.zeros((self.pixelWidth * self.pixelHeight, 6))



        self.projectionMatrix = p.computeProjectionMatrixFOV(self.fov, self.aspect,
                                                             self.nearPlane, self.farPlane,
                                                             physicsClientId=self._id)
        self.viewMatrix = None

        self.setCameraPose()

    def setCameraPose(self, idx = 0):
        assert idx < len(self.camera_poses)

        self.camera_idx = idx

        camera_pose = self.camera_poses[self.camera_idx]

        self.camTargetPos = camera_pose.position
        self.roll = camera_pose.rpy[0]
        self.pitch = camera_pose.rpy[1]
        self.yaw = camera_pose.rpy[2]
        self.camDistance = camera_pose.distance
        
        self.viewMatrix = p.computeViewMatrixFromYawPitchRoll(self.camTargetPos, self.camDistance,
                                                              yaw=self.yaw, pitch=self.pitch, roll=self.roll,
                                                              upAxisIndex=2, physicsClientId=self._id)


        p.resetDebugVisualizerCamera(cameraDistance=self.camDistance,
                                    cameraYaw=self.yaw, cameraPitch=self.pitch,
                                    cameraTargetPosition = self.camTargetPos
                                    )

        self.setCameraParams()

    def nextCameraPose(self):
        idx = (self.camera_idx+1)%len(self.camera_poses)

        self.setCameraPose(idx)

    def previousCameraPose(self):
        idx = (self.camera_idx-1)
        idx = (len(self.camera_poses)-1) if idx < 0 else idx

        self.setCameraPose(idx)

    def setCameraParams(self):
        # https://blog.noctua-software.com/opencv-opengl-projection-matrix.html
        opengl_proj_mat = np.array(self.projectionMatrix).reshape([4,4])
        opengl_view_mat = np.array(self.viewMatrix).reshape([4,4])

        self.fx = opengl_proj_mat[0,0] / 2.0 * self.pixelWidth
        self.fy = -opengl_proj_mat[1,1] / 2.0 * self.pixelHeight
        self.cx = (1.0 - opengl_proj_mat[2,0])/2.0*self.pixelWidth
        self.cy = (1.0 + opengl_proj_mat[2,1])/2.0*self.pixelHeight

        self.extrinsic_params = np.linalg.inv(opengl_view_mat).T
        self.camera_pos = self.extrinsic_params[0:3, 3]
        self.camera_quat = tt.quaternion_from_matrix(self.extrinsic_params)

        addDebugFrame(self.camera_pos, self.camera_quat, 0.2, 2, 10.0, self._id)

    def getImage(self):
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0)
        Renderer = p.ER_BULLET_HARDWARE_OPENGL #p.ER_TINY_RENDERER
        img_arr = p.getCameraImage(self.pixelWidth, self.pixelHeight,
                                   self.viewMatrix, self.projectionMatrix,
                                   renderer=Renderer, shadow=-1, physicsClientId=self._id)

        self.rgb = img_arr[2] # RGBA data
        self.dep = img_arr[3] # depth data
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    def corruptImage(self, depth_image):
        # zˆ(u, v) = z(u + N (0, σ2p), v + N (0, σ2p)) + N (0, σ2d)
        # Ed John's paper
        # Corrupt image with noise
        rows, cols = depth_image.shape

        std_dev = 0.001
        std_p = 1
        std_v = 1
        rintc = normal_random_integer(rows*cols,0,std_p).reshape(rows,cols)
        rintr = normal_random_integer(rows*cols,0,std_p).reshape(rows,cols)

        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=False)
        c_new = np.minimum(np.maximum(rintc + c,0),rows-1)
        r_new = np.minimum(np.maximum(rintr + r,0),cols-1)

        depth_image[c,r] = depth_image[c_new,r_new] # zˆ(u, v) = z(u + N (0, σ2p), v + N (0, σ2p))
        
        noise = np.random.randn(rows, cols)*std_dev
        noise_cutoff = 0.005
        noise[ noise < -noise_cutoff ] = -noise_cutoff
        noise[ noise > noise_cutoff ] = noise_cutoff
        depth_image = depth_image + noise # zˆ(u, v) = zˆ(u, v) + N (0, σ2d)


        # mask = (np.random.uniform(0,1,size=depth_image.shape) > 0.95).astype(np.bool)
        # # Randomly set 5% of the depth pixels to 0
        # depth_image[mask] = 0

        return depth_image

    def createPointCloud(self):
        xyz_rgb = self.xyz_rgb

        depth_image = np.array(self.dep).T
        depth_image = self.farPlane* self.nearPlane / (self.farPlane - (self.farPlane - self.nearPlane) * depth_image)
        # i = 0

        rows, cols = depth_image.shape



        t0 = time.time()
        
        depth_image = self.corruptImage(depth_image)

        # plt.imshow(depth_image)
        # plt.show()


        c, r = np.meshgrid(np.arange(cols), np.arange(rows), sparse=True)
        valid = (depth_image > 0) #
        z = np.where(valid, -depth_image , np.nan)
        x = np.where(valid, -z * (r - self.cx) / self.fx, 0)
        y = np.where(valid, -z * (c - self.cy) / self.fy, 0)
        # rgb = np.flipud(np.rot90(self.rgb))#[:, :,:3] #np.where(valid, self.rgb[r, c,:], [0,0,0])
        #
        # points = np.dstack((x, y, z,rgb))
        points = np.dstack((x, y, z))
        point_list = points.reshape(-1,3)
        # point_list = points.reshape(-1,7)
        # print "RGB shape: ", rgb.shape
        # points = np.vstack([x,y,z])
        # print "Point shape: ", points.shape
        # points = np.vstack([x,y,z,rgb])#np.dstack((x, y, z, rgb))
        # print points.shape

        # colors = np.where(valid, rgb[col, row, :3], 0) 
        # points = np.dstack((x, y, z))
        # for row in range(self.pixelWidth):
        #     for col in range(self.pixelHeight):
        #         z = depth_image[row, col]
        #         xyz_rgb[i, 0] = (row - self.cx) / self.fx * z
        #         xyz_rgb[i, 1] = (col - self.cy) / self.fy * z
        #         xyz_rgb[i, 2] = -z
        #         xyz_rgb[i, 3:6] = self.rgb[col, row, :3]
        #         i = i + 1
        # print "Got it!"
        print("Time elapsed %.3f"%(time.time()-t0,))
        return point_list
