# 概要  
RoboCup@Homeリーグ**StoringGroceries**用  

# 実行方法  

```
  $ rosrun tm_storing_groceries sg_start.sh  
  $ roslaunch realsense_camera r200_nodelet_rgbd.launch  
  $ roslaunch darknet_ros darknet_ros_gdb.launch  
  $ rosrun tm_storing_groceries Navigation.py  
  $ rosrun tm_storing_groceries Storing_Groceries.py  
```

# その他  
`Navigation.pyとStoring_Groceries.py`は毎回立ち上げ直してください  
