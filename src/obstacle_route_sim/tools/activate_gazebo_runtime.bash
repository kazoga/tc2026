# source tools/activate_gazebo_runtime.bash <deb 展開ディレクトリ>
# ROS 本体とワークスペースの setup.bash を先に読み込む。
terrain_runtime_root=$(realpath "${1:?deb 展開ディレクトリを指定してください}")
terrain_ros_prefix="$terrain_runtime_root/opt/ros/${ROS_DISTRO:-jazzy}"
export PATH="$terrain_runtime_root/usr/bin:$terrain_ros_prefix/opt/gz_tools_vendor/bin:$PATH"
export AMENT_PREFIX_PATH="$terrain_ros_prefix:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$terrain_ros_prefix/lib/python3.12/site-packages:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$terrain_runtime_root/usr/lib/x86_64-linux-gnu:$terrain_ros_prefix/lib:${LD_LIBRARY_PATH:-}"
export GZ_CONFIG_PATH=""
for terrain_vendor in "$terrain_ros_prefix"/opt/*; do
  export LD_LIBRARY_PATH="$terrain_vendor/lib:$LD_LIBRARY_PATH"
  export GZ_CONFIG_PATH="$terrain_vendor/share/gz:$GZ_CONFIG_PATH"
done
export RUBYLIB="$terrain_runtime_root/usr/lib/ruby/3.2.0:$terrain_runtime_root/usr/lib/x86_64-linux-gnu/ruby/3.2.0"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$terrain_ros_prefix/opt/gz_sim_vendor/lib/gz-sim-8/plugins"
export GZ_SIM_SERVER_CONFIG_PATH="$terrain_ros_prefix/opt/gz_sim_vendor/share/gz/gz-sim8/server.config"
export GZ_SIM_PHYSICS_ENGINE_PATH="$terrain_ros_prefix/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins"
export GZ_RENDERING_PLUGIN_PATH="$terrain_ros_prefix/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins"
export GZ_RENDERING_RESOURCE_PATH="$terrain_ros_prefix/opt/gz_rendering_vendor/share/gz/gz-rendering8"
export OGRE2_RESOURCE_PATH="$terrain_ros_prefix/opt/gz_ogre_next_vendor/lib/OGRE-Next"
unset terrain_vendor terrain_ros_prefix terrain_runtime_root
