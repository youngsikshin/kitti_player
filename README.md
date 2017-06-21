# kitti_player

This rpository has source code for publishing kitti odometry datasets to ros topic.

## Usage

Running with default parameters

```
rosrun kitti_player kitti_player
```

You can change parameters(e.g. data_path, left_topic, left_image_pub ... ) at runtime
For example, 

```
rosrun kitti_player kitti_player _data_path:="/var/data/kitti/dataset"
```

Roslaunch is also available. A sample of the default launch file is in the 'launch' directory.

```
roslaunch kitti_player kitti_player kitti_player.launch
```

kitti_player needs a directory tree like the following: 

└── dataset
│ └── poses
│ └ 00.txt
│ └ ...
│ └ 10.txt
│
├── sequences
│ └── 00
│ │ └── image_0
│ │ └── image_1
│ │ └── image_2
│ │ └── image_3
│ │ └── velodyne
│ │ └── times.txt
│ │ └── calib.txt
│ └── 01 (Subdirectory has same structure as above)
│ └── ...
│ └── 21

## TODO

* Publish GT posesr
