/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

/*Fast triangulation of unordered point clouds*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

/*voxel*/
#include <pcl/filters/voxel_grid.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*
for graph
type edge_t is inside here
*/
#include <global_planner/a_star_on_graph.h>
#include <set>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*TF listener, although it is included in sensor.h*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*srv msg for make plan*/
#include "dddmr_sys_core/action/get_plan.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace global_planner
{

class Global_Planner : public rclcpp::Node {
    public:
      Global_Planner(const std::string& name);
      ~Global_Planner();

      void initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d);
   
    private:

      bool is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> handle) const
      {
        return handle != nullptr && handle->is_active();
      }

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);

      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);
      
      std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> current_handle_;

      rclcpp_action::Server<dddmr_sys_core::action::GetPlan>::SharedPtr action_server_global_planner_;

      rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
      rclcpp::CallbackGroup::SharedPtr action_server_group_;
      
      rclcpp::Clock::SharedPtr clock_;

      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      std::shared_ptr<A_Star_on_Graph> a_star_planner_;
      
      std::string global_frame_;
      std::string robot_frame_;
      bool graph_ready_;
      bool has_initialized_;
      
      bool use_adaptive_connection_;
      int adaptive_connection_number_;
      double radius_of_ground_connection_;
      int boundary_reject_threshold_;
      
      /*Original point cloud*/
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up_;
      /*Original kdtree*/
      pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree_original_; 
      
      /*
      Graph class:
      static: similar to static layer in costmap_2d
      dynamic: similar to obstacle layer in costmap_2d, this class require frequently clearing, therefore, keep it small
      */
      perception_3d::StaticGraph static_graph_ = perception_3d::StaticGraph();

      rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_ground_;
      rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
      
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_static_graph_;
      rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_weighted_pc_;

      /*cb*/
      bool use_static_layer_in_perception_3d_;
      rclcpp::TimerBase::SharedPtr perception_3d_check_timer_;
      void checkPerception3DThread();
      void cbMapcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
      void cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal);
      

      /*Func*/
      void radiusSearchConnection();

      bool getStartGoalID(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal, 
                          unsigned int& start_id, unsigned int& goal_id);

      void pubStaticGraph();
      void getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path);
      void pubWeight();
      void makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle);

    protected:
      std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
      std::shared_ptr<tf2_ros::TransformListener> tfl_;
};

}
