// Copyright (c) 2023 arshadlab.
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
#include <boost/gil.hpp>
#include <boost/gil/io/dynamic_io_new.hpp>
#include <boost/gil/extension/io/png/old.hpp>
#include <boost/shared_ptr.hpp>
#include <ignition/math/Vector3.hh>
#include <gazebo_ros/node.hpp>
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include <gazebo_map_creator_interface/srv/map_request.hpp>

namespace gazebo
{

class GazeboMapCreator : public gazebo::SystemPlugin
{
  gazebo_ros::Node::SharedPtr ros_node_;

  rclcpp::Service<gazebo_map_creator_interface::srv::MapRequest>::SharedPtr  map_service_;
  
  physics::WorldPtr world_;
  /// Connection to world update event, called at every iteration
  gazebo::event::ConnectionPtr world_update_event_;

  /// To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  public: void Load(int argc, char ** argv)
  {
    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);      
    } 
    
    ros_node_ = gazebo_ros::Node::Get();
    // Get a callback when a world is created
    this->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
      std::bind(&GazeboMapCreator::OnWorldCreated, this, std::placeholders::_1));
 
  }

  public: void OnWorldCreated(const std::string & _world_name)
  {
      world_created_event_.reset();
      world_ = gazebo::physics::get_world(_world_name);

      map_service_ = ros_node_->create_service<gazebo_map_creator_interface::srv::MapRequest>(
      "/world/save_map",
      std::bind(
        &GazeboMapCreator::MapCreate, this,
        std::placeholders::_1, std::placeholders::_2));
  }
  
  public: void MapCreate(const std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Request> _req,
                      std::shared_ptr<gazebo_map_creator_interface::srv::MapRequest::Response>  _res )
  {
   
    RCLCPP_INFO(ros_node_->get_logger(), "Received message");
    std::cout << "Creating collision map with corners at (" <<
      _req->upperleft.x << ", " << _req->upperleft.y << "), (" <<
      _req->upperright.x << ", " << _req->upperright.y << "), (" <<
      _req->lowerright.x << ", " << _req->lowerright.y << "), (" <<
      _req->lowerleft.x << ", " << _req->lowerleft.y <<
        ") with collision projected vertical in Z from " << _req->ground_height  << " to " << _req->height << "\nResolution = " << _req->resolution << " m\n" <<
        "Occupied spaces will be filled with: " << _req->threshold <<
        std::endl;

   
    double dX_vertical = _req->upperleft.x - _req->lowerleft.x;
    double dY_vertical = _req->upperleft.y - _req->lowerleft.y;
    double mag_vertical =
      sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical);
    dX_vertical = _req->resolution * dX_vertical / mag_vertical;
    dY_vertical = _req->resolution * dY_vertical / mag_vertical;

    double dX_horizontal = _req->upperright.x - _req->upperleft.x;
    double dY_horizontal = _req->upperright.y - _req->upperleft.y;
    double mag_horizontal =
      sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
    dX_horizontal = _req->resolution * dX_horizontal / mag_horizontal;
    dY_horizontal = _req->resolution * dY_horizontal / mag_horizontal;

    int count_vertical = mag_vertical / _req->resolution;
    int count_horizontal = mag_horizontal / _req->resolution;

    if (count_vertical == 0 || count_horizontal == 0)
    {
      std::cout << "Image has a zero dimensions, check coordinates"
                << std::endl;
      _res->success = false;
      return;
    }
    
    boost::gil::gray8_pixel_t fill(255 - _req->threshold);
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(count_vertical, count_horizontal);

    gazebo::physics::PhysicsEnginePtr engine = world_->Physics();
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray =
      boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
        engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing model and checking collisions" << std::endl;
    boost::gil::fill_pixels(image._view, blank);
    std::cout << "Vertical " << count_vertical << ",  Horizontal " << count_horizontal 
                << ", dX_vertical " <<  dX_vertical << ", dY_vertical " <<  dY_vertical
                << std::endl;
    
    double x, y, dist;
    double distance_to_check = _req->distance;
    std::string entityName;

    ignition::math::Vector3d start, end;
    start.Z(_req->ground_height);
    end.Z(_req->height);
 
    for (int i = 0; i < count_vertical; ++i)
    {
      if (i % 200 == 0) {
            std::cout << "Percent complete: " << i * 100.0 / count_vertical
                << std::endl;
      }
      x = i * dX_vertical + _req->lowerleft.x;
      y = i * dY_vertical + _req->lowerleft.y;
      for (int j = 0; j < count_horizontal; ++j)
      {
        
        x += dX_horizontal;
        y += dY_horizontal;

        start.X(x);
        start.Y(y);
        
        end.X(x);
        end.Y(y);
        end.Z(_req->height);

        ray->SetPoints(start, end);      

        ray->GetIntersection(dist, entityName);
        if (!entityName.empty())
        { 
          image._view(i,j) = fill;
        }
        
        double dx=0.0;
        double dy=0.0;
        
        for (const auto& direction : {std::make_pair(-distance_to_check, 0.0), std::make_pair(0.0, distance_to_check), std::make_pair(distance_to_check, 0.0), std::make_pair(0.0, -distance_to_check)}) {
            dx = direction.first;
            dy = direction.second;
            end.X(x+dx);
            end.Y(y+dy);
            end.Z(_req->ground_height);

            ray->SetPoints(start, end);
            ray->GetIntersection(dist, entityName);
            if (!entityName.empty())
            {          
              image._view(i,j) = fill;
            }            
        } 
      }
    }

    std::cout << "Completed calculations, writing to image" << std::endl;
    if (!_req->filename.empty())
    {
      boost::gil::gray8_view_t view = image._view;

      // Write to png 
      boost::gil::png_write_view(_req->filename+".png", view); 
      // Write to pgm (pnm p2)
      pgm_write_view(_req->filename, view);
    }

    std::unordered_map<std::string, std::string> yaml_dict;

    yaml_dict["image"] = _req->filename + ".pgm";
    yaml_dict["mode"] = "trinary";
    yaml_dict["resolution"] = std::to_string(_req->resolution);
    yaml_dict["origin"] =  "[" + std::to_string(_req->lowerright.x) + std::string(", ") + std::to_string(_req->lowerright.y) + std::string(", 0.0]");
    yaml_dict["negate"] = "0";
    yaml_dict["occupied_thresh"] = "0.95";  // hardcoding these values since we absolutely know occupied or free
    yaml_dict["free_thresh"] = "0.90";

    std::ofstream outputFile(_req->filename + ".yaml");
    if (outputFile.is_open()) {
        for (const auto& pair : yaml_dict) {
            outputFile << pair.first << ": " << pair.second << std::endl;
        }
        outputFile.close();
    } else {
        std::cout << "Unable to open yaml file for writing." << std::endl;
    }

    std::cout << "Output location: " << _req->filename + "[.pgm, .yaml, .png]" << std::endl;
    
    _res->success = true;
  }

  public: void pgm_write_view(const std::string& filename, boost::gil::gray8_view_t& view)
  {
    // Write image to pgm file
    std::cout << "running" << std::endl;
    int h = view.height();
    int w = view.width();

    std::ofstream ofs;
    ofs.open(filename+".pgm");
    ofs << "P2" << '\n';          // grayscale
    ofs << w << ' ' << h << '\n'; // width and height
    ofs << 255 <<  '\n';          // max value
    for (int y = 0; y < h; ++y){
      for (int x = 0; x < w; ++x){
        // std::cout << (int)view(x, y)[0];
        ofs << (int)view(x, y)[0] << ' ';
      }
      ofs << '\n';
    }
    ofs.close();
  }
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(GazeboMapCreator)
}
