# euclidean_cluster
 
This is a ROS package fro 3D lidar Point cloud segmentation. It is recommended that you use this package with another `plane_fit_ground_filter`.

### Requirements

* ROS
* PCL 1.7
* jsk_rvize_plugins
```
$ sudo apt-get install ros-kinetic-jsk-rviz-plugins
```

### Run

There are two launch file, the `euclidean_cluster.launch` only run the node of `euclidean_cluster`.

if you use the `plane_fit_ground_filter` package, you can use `euclidean_cluster_output.launch`. You need to modify the `euclidean_cluster_output.launch` file of the line 53 to you own path.


### Example

![Screenshot 2019-06-21 11:25:40.png](https://i.loli.net/2019/06/21/5d0c4f403b96c42631.png)

