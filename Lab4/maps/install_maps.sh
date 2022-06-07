
#
# install SRA 2022 maps script (v02)
#
# versions:  
# v01 - initial version (basic install)
# v02 - update w/ absolute paths + extra maps + extra objects


# get directory basename 
fpath=`readlink -f "$0"`
base=`dirname "$fpath"`

# script name (install_maps.sh)
# echo $0
# file path (including filename) 
# echo $fpath
# absolute path location 
# echo $base
# testing ... 
#ls "$base"/gzb_models
#exit 0


echo "---installing SRA 2022 maps---"

# update file ownersip
chown $USER -R "$base"/gzb_models

# update file permissions (read, write to user and group)
chmod ug+rw -R "$base"/gzb_models

# update file permissions (execute on scripts)
chmod ug+x "$base"/gzb_models/start-gazebo-*.sh


echo "---copying 3D maps---"

# copy 3D maps
cp -rv "$base"/gzb_models/bxmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/csquare ~/.gazebo/models/
cp -rv "$base"/gzb_models/fmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/lcmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/mvmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/scmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/stmap ~/.gazebo/models/
cp -rv "$base"/gzb_models/umap ~/.gazebo/models/

# copy 3D objects
cp -r "$base"/gzb_models/omodels/sra_sign ~/.gazebo/models/
cp -r "$base"/gzb_models/omodels/box ~/.gazebo/models/

echo "---copying gazebo & ros config files---"

# copy launch files
cp -v "$base"/gzb_models/*.launch /opt/ros/melodic/share/turtlebot3_gazebo/launch/

# copy world files
cp -v "$base"/gzb_models/*.world /opt/ros/melodic/share/turtlebot3_gazebo/worlds/


echo "---copying run scripts----"
cp -v "$base"/gzb_models/start-gazebo-*.sh ~/

echo "---done---"


