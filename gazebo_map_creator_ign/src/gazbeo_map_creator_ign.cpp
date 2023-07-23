// Copyright (c) 2023.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fstream>
#include <iostream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

#ifdef GZ_HEADERS

  #include <gz/sim/System.hh>
  #include <gz/plugin/Register.hh>
  #include <gz/sim/components/Name.hh>
  #include <gz/sim/components/ParentEntity.hh>
  #include <gz/sim/World.hh>
  #include <gz/sim/Model.hh>
  #include <gz/sim/components/Physics.hh>
  #include <gz/sim/rendering/SceneManager.hh>
  #include <gz/rendering/RenderEngine.hh>
  #include <gz/rendering/RenderingIface.hh>
  #include <gz/rendering/Scene.hh>
  #include "gz/sim/rendering/Events.hh"
  #include "gz/sim/Events.hh"
  #include "gz/sim/rendering/RenderUtil.hh"
  namespace sim = gz::sim;
  namespace render = gz::rendering;

#else

  #include <ignition/gazebo/System.hh>
  #include <ignition/plugin/Register.hh>
  #include <ignition/gazebo/Model.hh>
  #include <ignition/gazebo/components/Physics.hh>
  #include <ignition/gazebo/components.hh>
  #include <ignition/gazebo/components/Name.hh>
  #include <ignition/gazebo/components/ParentEntity.hh>
  #include <ignition/gazebo/World.hh>
  #include <ignition/transport/AdvertiseOptions.hh>
  #include <ignition/transport/Node.hh>
  #include <ignition/msgs.hh>
  #include <ignition/gazebo/Util.hh>
  #include <ignition/math/Vector3.hh>
  #include <ignition/rendering/RenderEngine.hh>
  #include <ignition/rendering/RenderingIface.hh>

  namespace sim = ignition::gazebo;
  namespace render = ignition::rendering;

#endif

#include <pluginlib/class_loader.hpp>
#include <gazebo_map_creator_interface/srv/map_request.hpp>
#include <boost/gil.hpp>
#include <boost/gil/io/dynamic_io_new.hpp>
#include <boost/gil/extension/io/png/old.hpp>
#include <boost/shared_ptr.hpp>
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
#include <octomap/octomap.h>

class GazeboMapCreator :
  public sim::System,
  public sim::ISystemConfigure
{

public: void Configure(const sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& /*_sdf*/,
                         sim::EntityComponentManager& /*_ecm*/, sim::EventManager& _eventMgr)
  {
      (void) _entity;

      pEventMgr_ = &_eventMgr;
      if (!rclcpp::ok())
      {
          rclcpp::init(0, nullptr);
      }

      // Though spin is not needed for service but still we will enable node spin.
      node_ = rclcpp::Node::make_shared("map_generator");
      RCLCPP_INFO(node_->get_logger(), "Node created");
      this->executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      this->executor_->add_node(this->node_);
      this->stop_ = false;
      auto spin = [this]()
        {
          while (rclcpp::ok() && !this->stop_) {
            this->executor_->spin_once();
          }
        };
      this->thread_executor_spin_ = std::thread(spin);

      map_service_ = node_->create_service<gazebo_map_creator_interface::srv::MapRequest>(
      "/world/save_map",
      std::bind(
        &GazeboMapCreator::OnMapCreate, this,
        std::placeholders::_1, std::placeholders::_2));

      //sim::World world = sim::World(_entity);
      //world_ = sim::components::Physics(world);

  }

  public: void OnMapCreate(const std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Request> _req,
                           std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Response> _res)
  {
    (void)_req;
    (void)_res;

    // Your map creation code here
    RCLCPP_INFO(node_->get_logger(), "Map Create Service Called");

    // TODO:  Below code not working.
    // Gazebo Classic plugin creates a RayShape and then finds out any intersection between two points.
    // We want to do the same in IGN but RayShape is not available here.
    //

    // Get the scene
    render::ScenePtr scene = render::sceneFromFirstRenderEngine();
    if (!scene)
    {
      ignwarn << "No active rendering scene, can't create ray query yet\n";
      return;
    }

    #ifdef GZ_HEADERS
      // Create the ray query
      render::RayQueryPtr rayQuery_ = scene->CreateRayQuery();
      if (!rayQuery_)
      {
        ignerr << "Failed to create ray query\n";
        return;
      }
    #endif

    // Possible solution but currently not working.
    /*
    gz::math::Vector3d startPoint(0, 0, 0);
    gz::math::Vector3d endPoint(1, 1, 1);
    rayQuery_->SetOrigin(startPoint);
    rayQuery_->SetDirection(endPoint);
    gz::rendering::RayQueryResult result = rayQuery_->ClosestPoint();
    */

    // TODO: Use Gazebo classic approach here to create 2D/3D maps.

  }

private:
  //private: std::shared_ptr<transport::Node> node;
  rclcpp::Node::SharedPtr node_;

  sim::EventManager *pEventMgr_ ;
  rclcpp::Service<gazebo_map_creator_interface::srv::MapRequest>::SharedPtr  map_service_;

  std::thread thread_executor_spin_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  bool stop_{false};

};

#ifdef GZ_HEADERS

  GZ_ADD_PLUGIN(
    GazeboMapCreator,
    sim::System,
   GazeboMapCreator::ISystemConfigure)
  GZ_ADD_PLUGIN_ALIAS(
    GazeboMapCreator,
    "GazeboMapCreator")

#else

  IGNITION_ADD_PLUGIN(GazeboMapCreator,
                      sim::System,
                      sim::ISystemConfigure)
  IGNITION_ADD_PLUGIN_ALIAS(GazeboMapCreator, "IgnitionMapCreator")

#endif


