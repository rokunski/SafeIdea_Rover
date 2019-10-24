#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

namespace gazebo{

class WorldPluginRover : public WorldPlugin{

  public: WorldPluginRover() : WorldPlugin(){
  }

  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    //make sure the ROS node for Gazebo has already been initialized
    if(!ros::isInitialized()){
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin.");
    return;
    }

    ROS_INFO("Mars Loaded");

  }
};

GZ_REGISTER_WORLD_PLUGIN(WorldPluginRover)
}
