xterm -geometry 80x5+0+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup minimal.launch" &
sleep 7s
xterm -geometry 80x5+0+120 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_bringup 3dsensor.launch" &
sleep 7s
xterm -geometry 80x5+600+0 -e "/opt/ros/kinetic/bin/roslaunch turtlebot_navigation gmapping_demo.launch" &
sleep 7s
#xterm -geometry 80x5+600+220 -e "/opt/ros/kinetic/bin/rosrun tm_storing_groceries Navigation.py" &
#sleep 2s
#xterm -geometry 80x5+600+320 -e "/opt/ros/kinetic/bin/rosrun tm_storing_groceries PDF_Generator.py" &
#sleep 2s
xterm -geometry 80x5+0+320 -e "/opt/ros/kinetic/bin/rosrun e_object_recognizer object_recognizer.py" &
sleep 3s
xterm -geometry 80x5+0+420 -e "/opt/ros/kinetic/bin/rosrun e_grasping_position_detector e_grasping_position_detector"&
sleep 3s
xterm -geometry 80x5+0+520 -e "/opt/ros/kinetic/bin/rosrun e_manipulation manipulation.py" &
sleep 3s
