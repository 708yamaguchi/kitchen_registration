# kitchen registration

This package is designed to help PR2 estimate its own position in the kitchen with high accuracy

## Usage

### Create workspace

```bash
mkdir ~/kitchen_ws/src
cd ~/kitchen_ws/src
git clone https://github.com/708yamaguchi/kitchen_registration.git
rosdep install --from-paths . --ignore-src -y -r
cd ..
catkin build kitchen_registration
```

### Create Point Cloud Data (.pcd) file in the kitchen

```bash
roslaunch kitchen_registration pointcloud_to_pcd.launch
```

- When a pcd containing more than 60000 points is generated, stop the program with Ctrl-c.
  ```
  [ INFO] [1645802003.046638548]: Received 61668 data points in frame base_link with the following fields: x y z index
  ```

- Change the name of the generated file to `base_link.pcd`.
  - This is because the frame_id of the pointcloud that PointCloudDatabaseServer publishes depends on the file name.
  - https://jsk-docs.readthedocs.io/projects/jsk_recognition/en/latest/jsk_pcl_ros/nodes/pointcloud_database_server.html

  ```bash
  roscd kitchen_registration/data
  mv sample_pcd_xxx.pcd base_link.pcd
  ```

### ICP registration

```bash
# Terminal 1
roslaunch kitchen_registration icp_registration.launch
# Terminal 2
rostopic echo /icp_registration/icp_result
```

### TODO

- Download the desired base_link.pcd during `catkin build`by using `jsk_data`
- Calculate position of PR2 based on the result of the icp registration
- Evaluate the accuracy of positioning, including whether this package can be applied to object grasping.
