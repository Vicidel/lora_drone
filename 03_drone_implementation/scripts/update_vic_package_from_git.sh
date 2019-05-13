cd ~/catkin_ws/src
echo "Removing old vic_package folder"
rm -rf vic_package
echo "Copying new from git"
svn export http://github.com/vicidel/lora_drone/trunk/03_drone_implementation/vic_package
echo "DONE!"