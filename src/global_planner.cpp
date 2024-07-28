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
#include <global_planner/global_planner.h>

using namespace std::chrono_literals;

namespace global_planner
{

Global_Planner::Global_Planner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse Global_Planner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Global_Planner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Global_Planner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Global_Planner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void Global_Planner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){

  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  
  declare_parameter("use_static_layer_in_perception_3d", rclcpp::ParameterValue(true));
  this->get_parameter("use_static_layer_in_perception_3d", use_static_layer_in_perception_3d_);
  RCLCPP_INFO(this->get_logger(), "use_static_layer_in_perception_3d: %d", use_static_layer_in_perception_3d_);

  declare_parameter("radius_of_ground_connection", rclcpp::ParameterValue(1.0));
  this->get_parameter("radius_of_ground_connection", radius_of_ground_connection_);
  RCLCPP_INFO(this->get_logger(), "radius_of_ground_connection: %.2f", radius_of_ground_connection_);

  declare_parameter("use_adaptive_connection", rclcpp::ParameterValue(true));
  this->get_parameter("use_adaptive_connection", use_adaptive_connection_);
  RCLCPP_INFO(this->get_logger(), "use_adaptive_connection: %d", use_adaptive_connection_);  
  
  declare_parameter("adaptive_connection_number", rclcpp::ParameterValue(20));
  this->get_parameter("adaptive_connection_number", adaptive_connection_number_);
  RCLCPP_INFO(this->get_logger(), "adaptive_connection_number: %d", adaptive_connection_number_);  

  declare_parameter("boundary_reject_threshold", rclcpp::ParameterValue(15));
  this->get_parameter("boundary_reject_threshold", boundary_reject_threshold_);
  RCLCPP_INFO(this->get_logger(), "boundary_reject_threshold: %d", boundary_reject_threshold_);    
  
  declare_parameter("turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("turning_weight", turning_weight_);
  RCLCPP_INFO(this->get_logger(), "turning_weight: %.2f", turning_weight_);    
  
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  if(use_static_layer_in_perception_3d_){
    perception_3d_check_timer_ = this->create_wall_timer(100ms, std::bind(&Global_Planner::checkPerception3DThread, this), action_server_group_);
  }
  else{
    sub_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "mapground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
        std::bind(&Global_Planner::cbMapcloud, this, std::placeholders::_1), sub_options);
  }


  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&Global_Planner::cbClickedPoint, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&Global_Planner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Global_Planner::handle_cancel, this, std::placeholders::_1),
    std::bind(&Global_Planner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

Global_Planner::~Global_Planner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  action_server_global_planner_.reset();
}

void Global_Planner::checkPerception3DThread(){
  
  if(graph_ready_)
    return;
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  pc_original_z_up_ = perception_3d_ros_->getSharedDataPtr()->pcl_ground_;
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  kdtree_original_ = perception_3d_ros_->getSharedDataPtr()->kdtree_ground_;

  RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d, start to generate connection.");
  radiusSearchConnection();
  
}

void Global_Planner::cbMapcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  graph_ready_ = false;
  pc_original_z_up_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pc_original_z_up_);

  if(perception_3d_ros_->getGlobalUtils()->getGblFrame().compare(msg->header.frame_id) != 0)
    RCLCPP_ERROR(this->get_logger(), "The global frame is not consistent with topics and perception setting. Make sure to add perception_3d map/robot frame setting.");

  global_frame_ = msg->header.frame_id;
  kdtree_original_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  kdtree_original_->setInputCloud(pc_original_z_up_);
  RCLCPP_INFO(this->get_logger(), "Got ground , start to generate Kd-tree in global planner.");
  radiusSearchConnection();

}

void Global_Planner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  geometry_msgs::msg::PoseStamped start, goal;

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
  }

  unsigned int start_id, goal_id;
  std::vector<unsigned int> path;
  nav_msgs::msg::Path ros_path;

  if(getStartGoalID(start, goal, start_id, goal_id)){
    a_star_planner_->getPath(start_id, goal_id,path);    
  }


  if(path.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
  }
  else{
    getROSPath(path, ros_path);
    pub_path_->publish(ros_path);
  }

}

void Global_Planner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = pc_original_z_up_->points[path_id[it]].x;
    pst.pose.position.y = pc_original_z_up_->points[path_id[it]].y;
    pst.pose.position.z = pc_original_z_up_->points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1){
      next_pst.pose.position.x = pc_original_z_up_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pc_original_z_up_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pc_original_z_up_->points[path_id[it+1]].z;
    }


    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();


    pst.pose.orientation.x = q.getX();
    pst.pose.orientation.y = q.getY();
    pst.pose.orientation.z = q.getZ();
    pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    if(it<path_id.size()-1){
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      for(double step=0.05;step<0.99;step+=0.05){

        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      ros_path.poses.push_back(pst);
    }
    
  }
}

bool Global_Planner::getStartGoalID(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                                    unsigned int& start_id, unsigned int& goal_id){

  //@Get goal ID
  std::vector<int> pointIdxRadiusSearch_goal;
  std::vector<float> pointRadiusSquaredDistance_goal;
  pcl::PointXYZ pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;

  //@Compute nearest pc as goal
  //@TODO: add an edge between goal and nearest pc
  
  if(kdtree_original_->radiusSearch (pcl_goal, 0.5, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal)<1){
    RCLCPP_WARN(this->get_logger(), "Goal is not found.");
    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    bool second_search = false;
    if(graph_ready_){
      for(double z=goal.pose.position.z; z>-10;z-=0.1){
        pointIdxRadiusSearch_goal.clear();
        pointRadiusSquaredDistance_goal.clear();
        pcl_goal.z = z;
        if(kdtree_original_->radiusSearch(pcl_goal, 0.3, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal,0)>0)
        {
          second_search = true;
          break;
        }
      }
      if(!second_search)
        return false;
    }
    else{
      return false;
    }
    return false;
  }

  RCLCPP_WARN(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
    goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, pointIdxRadiusSearch_goal[0], 
    pc_original_z_up_->points[pointIdxRadiusSearch_goal[0]].x, pc_original_z_up_->points[pointIdxRadiusSearch_goal[0]].y, pc_original_z_up_->points[pointIdxRadiusSearch_goal[0]].z);

  //--------------------------------------------------------------------------------------
  //@Get start ID
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZ pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;

  if(kdtree_original_->radiusSearch (pcl_start, 0.5, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start)<1){
    RCLCPP_WARN(this->get_logger(), "Start is not found.");
    return false;
  }

  RCLCPP_WARN(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
    start.pose.position.x, start.pose.position.y, start.pose.position.z, pointIdxRadiusSearch_start[0], 
    pc_original_z_up_->points[pointIdxRadiusSearch_start[0]].x, pc_original_z_up_->points[pointIdxRadiusSearch_start[0]].y, pc_original_z_up_->points[pointIdxRadiusSearch_start[0]].z);

  start_id = pointIdxRadiusSearch_start[0];

  goal_id = pointIdxRadiusSearch_goal[0];

  return true;

}

void Global_Planner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){

  unsigned int start_id, goal_id;
  std::vector<unsigned int> path;
  nav_msgs::msg::Path ros_path;

  //@ WE dont rely on the start from service call, global plan is always start at baselink using tf
  geometry_msgs::msg::TransformStamped transformStamped;
  geometry_msgs::msg::PoseStamped start;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
  }

  const auto goal = goal_handle->get_goal();

  if(getStartGoalID(start, goal->goal, start_id, goal_id)){
    a_star_planner_->getPath(start_id, goal_id, path);    
  }
  
  auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();

  if(path.empty()){
    result->path = ros_path;
    goal_handle->abort(result);
    RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
  }
  else{
    getROSPath(path, ros_path);
    ros_path.poses.push_back(goal->goal);
    pub_path_->publish(ros_path);
    result->path = ros_path;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
  }
  
}

void Global_Planner::radiusSearchConnection(){
  
  //@Calculate node weight

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();
  static_graph->clear();
  unsigned int index_cnt = 0;
  for(auto it = pc_original_z_up_->points.begin();it!=pc_original_z_up_->points.end();it++){

    pcl::PointXYZ pcl_node;
    pcl_node = (*it);

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      kdtree_original_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{

      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      kdtree_original_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        kdtree_original_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr nn_pc (new pcl::PointCloud<pcl::PointXYZ>);
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      
      //chekc relative z value for the edge, because we need to eliminate stair and wheel chair passage issue
      edge_t a_edge;
      auto node = index_cnt;
      a_edge.first = (*it);
      double z_diff = fabs(pc_original_z_up_->points[node].z - pc_original_z_up_->points[a_edge.first].z);
      if(z_diff >=0.1){
        RCLCPP_DEBUG(this->get_logger(), "Connection between %u and %u is %.2f",node, a_edge.first, z_diff);
        continue;
      }
      //@Create an edge
      a_edge.second = sqrt(pcl::geometry::squaredDistance(pc_original_z_up_->points[node], pc_original_z_up_->points[a_edge.first]));
      static_graph_.insertNode(node, a_edge);

      //@Push bach the points for plane equation later
      nn_pc->push_back(pc_original_z_up_->points[(*it)]);
    }

    float weight = 1.0;
    float max_radius = 1.0; //this value is suggested to be 1.0 meters, because if it is too large, the narrow passage will be miscalculated

    //@ consider this scenario to be boundary of ground
    if(nn_pc->points.size()<5){
      //This node is orphan, set high weight to it
      weight = 1000;
    }
    else{
      //@Use RANSAC to get normal
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.05); 

      seg.setInputCloud (nn_pc);
      seg.segment (*inliers, *coefficients);
      
      //@We get planar equation (coefficients), we now calculate z by iterate x,y value
      //@We use polar coordinate to generate points (i.e.: iterate theta with given radius)
      //@We search multiple rings
      int reject_threshold = 0;
      for(float ring_radius=max_radius; ring_radius>0; ring_radius-=0.5){
        for(float d_theta=-3.1415926; d_theta<=3.1415926; d_theta+=0.174){ //per 10 deg
          pcl::PointXYZ pcl_ring;
          pcl_ring.x = pcl_node.x + ring_radius*sin(d_theta);
          pcl_ring.y = pcl_node.y + ring_radius*cos(d_theta);
          pcl_ring.z = (-coefficients->values[3]-coefficients->values[0]*pcl_ring.x-coefficients->values[1]*pcl_ring.y)/coefficients->values[2];
          if(isinf(pcl_ring.z)){
            pcl_ring.z = 0.0;
            //RCLCPP_INFO(this->get_logger(), "%.2f,%.2f,%.2f", pcl_ring.x, pcl_ring.y, pcl_ring.z);
          }
          if(std::isnan(pcl_ring.z))
            continue;
          //@Search around the ring
          std::vector<int> pointIdxRadiusSearch_ring;
          std::vector<float> pointRadiusSquaredDistance_ring;
          if(kdtree_original_->radiusSearch (pcl_ring, 0.3, pointIdxRadiusSearch_ring, pointRadiusSquaredDistance_ring)<1) //0.3 is related to resolutio, looks good for 0.5 m voxel
            reject_threshold++;
        }
      }

      if(reject_threshold>boundary_reject_threshold_)
        weight = reject_threshold*0.5; //0.5 is to be justified, because it is related to voxel size
    }
    
    static_graph_.insertWeight(index_cnt, weight);//make the value of weight to 1 for non-weighted A*
    index_cnt++;
  }
  pubStaticGraph();

  RCLCPP_INFO(this->get_logger(), "Static graph is generated with size: %lu", static_graph_.getSize());
   
  if(!has_initialized_){
    has_initialized_ = true;
    a_star_planner_ = std::make_shared<A_Star_on_Graph>(pc_original_z_up_, static_graph_, perception_3d_ros_);
    a_star_planner_->setupTurningWeight(turning_weight_);
  }
  else{
    a_star_planner_->updateGraph(pc_original_z_up_, static_graph_);
  }

  pubWeight();

  RCLCPP_INFO(this->get_logger(), "Publish weighted pc.");
  graph_ready_ = true;
}


void Global_Planner::pubWeight(){

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();
  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);

  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){


    pcl::PointXYZI ipt;

    ipt.x = pc_original_z_up_->points[(*it).first].x;
    ipt.y = pc_original_z_up_->points[(*it).first].y;
    ipt.z = pc_original_z_up_->points[(*it).first].z;
    ipt.intensity = static_graph_.getNodeWeight((*it).first);    
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void Global_Planner::pubStaticGraph(){
 
  //@edge visualization 
  //@This is just for visulization, therefore reduce edges to let rviz less lag
  
  std::set<std::pair<unsigned int, unsigned int>> duplicate_check;
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = global_frame_;
  markerEdge.header.stamp = clock_->now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "edges";
  markerEdge.id = 3;
  markerEdge.scale.x = 0.03;
  markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
  markerEdge.color.a = 0.2;

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();

  int cnt = 0;
  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){
    geometry_msgs::msg::Point p;
    p.x = pc_original_z_up_->points[(*it).first].x;
    p.y = pc_original_z_up_->points[(*it).first].y;
    p.z = pc_original_z_up_->points[(*it).first].z;
    for(auto it_set = (*it).second.begin();it_set != (*it).second.end();it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = (*it).first;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = (*it).first;
      if( !duplicate_check.insert(edge_marker).second )
      {   
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {   
        continue;
      }
      markerEdge.points.push_back(p);
      p.x = pc_original_z_up_->points[(*it_set).first].x;
      p.y = pc_original_z_up_->points[(*it_set).first].y;
      p.z = pc_original_z_up_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

}