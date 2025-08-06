// Updated for ROS 2 Jazzy and Gazebo Harmonic (gz-sim8)
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/World.hh>
#include <gz/physics/Shape.hh>
#include <gz/plugin/Loader.hh>
#include <gz/physics/RequestEngine.hh>
#include <gz/physics/ConstructEmpty.hh>
#include <gz/physics/FindFeatures.hh>
//#include <gz/physics/RayShape.hh>
#include <gz/sim/rendering/SceneManager.hh>
#include <gz/rendering/RenderEngine.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include "gz/sim/rendering/Events.hh"
#include "gz/sim/Events.hh"
#include "gz/sim/rendering/RenderUtil.hh"

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/World.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
//#include <gz/msgs/laser_scan.hh>
#include <gz/msgs/laserscan.pb.h>

#include <rclcpp/rclcpp.hpp>
#include <gazebo_map_creator_interface/srv/map_request.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <octomap/octomap.h>

using namespace gz;
using namespace gz::sim;
using namespace sim;
using namespace systems;

using pcl::PointCloud;
using pcl::PointXYZ;

namespace gazebo_map_creator
{
class GazeboMapCreator
  : public System,
    public ISystemConfigure
{
public:
  virtual void Configure(
    const Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &/*_sdf*/,
    EntityComponentManager & _ecm,
    EventManager &/*_eventMgr*/)
  {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);

    this->node_ = std::make_shared<rclcpp::Node>("gazebo_map_creator_node");

    this->map_service_ = node_->create_service<gazebo_map_creator_interface::srv::MapRequest>(
      "/world/save_map",
      std::bind(&GazeboMapCreator::OnMapCreate, this, std::placeholders::_1, std::placeholders::_2,  std::ref(_ecm)));

    gz_node_.Subscribe("/world/default/model/virtual_lidar/link/lidar_link/sensor/ray_sensor/scan", &GazeboMapCreator::OnLidarData, this);

    RCLCPP_INFO(this->node_->get_logger(), "Map Creator Plugin loaded and service advertised.");
  }

  void OnMapCreate(
    const std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Request> req,
    std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Response> res,
    EntityComponentManager &_ecm)
  {
    RCLCPP_INFO(node_->get_logger(), "Map request received: resolution = %f", req->resolution);

    // Area calculations
    float size_x = req->upperleft.x - req->lowerright.x;
    float size_y = req->lowerright.y - req->upperleft.y;
    float size_z = req->upperleft.z - req->lowerright.z;

    if (size_x <= 0 || size_y >= 0 || size_z <= 0)
    {
      RCLCPP_ERROR(node_->get_logger(), "Invalid coordinates");
      res->success = false;
      return;
    }

    int num_points_x = static_cast<int>(std::abs(size_x) / req->resolution) + 1;
    int num_points_y = static_cast<int>(std::abs(size_y) / req->resolution) + 1;
    int num_points_z = req->skip_vertical_scan ? 2 : static_cast<int>(std::abs(size_z) / req->resolution) + 1;

    float step_x = size_x / num_points_x;
    float step_y = size_y / num_points_y;
    float step_z = req->skip_vertical_scan ? size_z : size_z / num_points_z;

    pcl::PointCloud<pcl::PointXYZ> cloud;



    //auto rayQuery = gz::physics::RayQuery();

    for (int x = 0; x < num_points_x; ++x)
    {
      double cur_x = req->lowerright.x + x * step_x;
      for (int y = 0; y < num_points_y; ++y)
      {
        double cur_y = req->upperleft.y + y * step_y;
        for (int z = 0; z < num_points_z; ++z)
        {
          double cur_z = req->lowerright.z + z * step_z;
          gz::math::Vector3d start(cur_x, cur_y, cur_z);
          gz::math::Vector3d end = start + gz::math::Vector3d(0, 0, step_z);

          //auto result = rayQuery.CastRay(_ecm, start, end);

          //if (result)
          {
            cloud.push_back(pcl::PointXYZ(cur_x, cur_y, cur_z));
          }
        }
      }
    }

    if (!req->filename.empty() && cloud.size() > 0)
    {
      pcl::io::savePCDFileASCII(req->filename + ".pcd", cloud);

      octomap::OcTree octree(req->resolution);
      for (const auto &p : cloud.points)
        octree.updateNode(octomap::point3d(p.x, p.y, p.z), true);

      octree.updateInnerOccupancy();
      octree.writeBinary(req->filename + ".bt");

      RCLCPP_INFO(node_->get_logger(), "Output saved: %s[.pcd, .bt]", req->filename.c_str());
    }
   
    res->success = true;
  }

  void OnLidarData(const gz::msgs::LaserScan &msg)
  {
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    latest_lidar_points_.clear();
    double angle = msg.angle_min();
    for (int i = 0; i < msg.ranges_size(); ++i)
    {
      double r = msg.ranges(i);
      if (r < msg.range_max())
      {
        double x = r * cos(angle);
        double y = r * sin(angle);
        latest_lidar_points_.emplace_back(x, y, 0);
      }
      angle += msg.angle_step();
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Service<gazebo_map_creator_interface::srv::MapRequest>::SharedPtr map_service_;
  gz::transport::Node gz_node_;
  std::vector<gz::math::Vector3d> latest_lidar_points_;
  std::mutex lidar_mutex_;
};
}  // namespace gazebo_map_creator

 // Register plugin
GZ_ADD_PLUGIN(gazebo_map_creator::GazeboMapCreator,
                    gz::sim::System,
                    gazebo_map_creator::GazeboMapCreator::ISystemConfigure)
