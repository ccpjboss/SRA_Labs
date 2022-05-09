
# install SRA 2022 maps script (v01)

echo "---installing SRA 2022 maps---"

# update file ownersip
chown $USER -R gzb_models

# update file permissions (read, write to user and group)
chmod ug+rw -R gzb_models

# update file permissions (execute on scripts)
chmod ug+x gzb_models/start-gazebo-*.sh


echo "---copying 3D maps---"

# copy 3D maps
cp -rv gzb_models/csquare ~/.gazebo/models/
cp -rv gzb_models/fmap ~/.gazebo/models/
cp -rv gzb_models/scmap ~/.gazebo/models/
cp -rv gzb_models/stmap ~/.gazebo/models/
cp -rv gzb_models/umap ~/.gazebo/models/

# copy 3D objects
cp -r gzb_models/omodels/sra_sign ~/.gazebo/models/

echo "---copying gazebo & ros config files---"

# copy launch files
cp -v gzb_models/*.launch /opt/ros/melodic/share/turtlebot3_gazebo/launch/

# copy world files
cp -v gzb_models/*.world /opt/ros/melodic/share/turtlebot3_gazebo/worlds/


echo "---copying run scripts----"
cp -v gzb_models/start-gazebo-*.sh ~/

echo "---done---"


