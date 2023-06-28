#read_pcd
读取一个目录下的所有bin或者pcd文件，对其进行地面分割，并发布成topic

点云分割使用的方法是`3D_Ground_Segmentation`,https://github.com/chrise96/3D_Ground_Segmentation

##How to use
```bash
catkin build read_pcd
roslaunch read_pcd readPCD.launch
```