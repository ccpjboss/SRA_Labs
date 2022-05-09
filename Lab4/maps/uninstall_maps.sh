
# uninstall SRA 2022 maps (v01)

echo "---uninstalling SRA 2022 maps---"

# remove 3D maps
rm -rf ~/.gazebo/models/csquare
rm -rf ~/.gazebo/models/fmap
rm -rf ~/.gazebo/models/scmap
rm -rf ~/.gazebo/models/stmap
rm -rf ~/.gazebo/models/umap

# remove 3D objects
rm -rf ~/.gazebo/models/sra_sign

# remove launch files
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_csquare.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_fmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_scmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_stmap.launch
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/launch/tbot3_umap.launch

# remove world files
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_csquare.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_fmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_scmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_stmap.world
rm -v /opt/ros/melodic/share/turtlebot3_gazebo/worlds/tbot3_umap.world

# remove run scripts
rm -v ~/start-gazebo-csquare.sh
rm -v ~/start-gazebo-fmap.sh
rm -v ~/start-gazebo-scmap.sh
rm -v ~/start-gazebo-stmap.sh
rm -v ~/start-gazebo-umap.sh

echo "---all maps removed---"


