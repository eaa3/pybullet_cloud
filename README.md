### Dependencies:
* pybullet
* rospy and tf for rviz visualization (can be removed)

### How to test
```
$ cd pybullet_cloud
$ source set_env.bash
$ python src/test.py
```

### Visualize in rviz
Add "PointCloud2" with topic name "/sim_cloud"