roslaunch jackal_gazebo empty_world.launch confi:=front_laser &

roslaunch jackal_navigation odom_navigation_demo.launch &

roslaunch jackal_viz view_robot.launch config:=navigation &