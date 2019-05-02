echo "Removing old gazebo_sim folder"
rm -rf gazebo_sim
echo "Downloading new from git"
svn export http://github.com/vicidel/lora_drone/trunk/gazebo_sim
echo "Removing old catkin_ws/src/vic_package folder"
rm -rf catkin_ws/src/vic_package
echo "Copying new from gazebo_sim folder"
cp -R gazebo_sim/vic_package catkin_ws/src/vic_package
echo "DONE!"
