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
  
  perception_3d_check_timer_ = this->create_wall_timer(100ms, std::bind(&Global_Planner::checkPerception3DThread, this), action_server_group_);
  
  /*
    sub_ground_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "mapground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
        std::bind(&Global_Planner::cbMapcloud, this, std::placeholders::_1), sub_options);
  */

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
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

void Global_Planner::checkPerception3DThread(){
  
  if(graph_ready_)
    return;
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  pcl_ground_ = perception_3d_ros_->getSharedDataPtr()->pcl_ground_;
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  kdtree_ground_ = perception_3d_ros_->getSharedDataPtr()->kdtree_ground_;
  kdtree_map_ = perception_3d_ros_->getSharedDataPtr()->kdtree_map_;
  pcl_map_ = perception_3d_ros_->getSharedDataPtr()->pcl_map_;
  static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_;
  RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d, start to generate connection.");
  getStaticGraphFromPerception3D();
  
}

void Global_Planner::cbMapcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  graph_ready_ = false;
  pcl_ground_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *pcl_ground_);

  if(perception_3d_ros_->getGlobalUtils()->getGblFrame().compare(msg->header.frame_id) != 0)
    RCLCPP_ERROR(this->get_logger(), "The global frame is not consistent with topics and perception setting. Make sure to add perception_3d map/robot frame setting.");

  global_frame_ = msg->header.frame_id;
  kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZ>());
  kdtree_ground_->setInputCloud(pcl_ground_);
  RCLCPP_INFO(this->get_logger(), "Got ground, start to generate Kd-tree in global planner.");
  getStaticGraphFromPerception3D();

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
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  if(getStartGoalID(start, goal, start_id, goal_id)){
    a_star_planner_->getPath(start_id, goal_id,path);    
  }


  if(path.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    getROSPath(path, ros_path);
    pub_path_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
  }

}

void Global_Planner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(auto it=1;it<path_id.size()-1;it++){

    geometry_msgs::msg::PoseStamped next_pst;
    next_pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    next_pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    next_pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    double vx,vy,vz;
    vx = next_pst.pose.position.x - current_pst.pose.position.x;
    vy = next_pst.pose.position.y - current_pst.pose.position.y;
    vz = next_pst.pose.position.z - current_pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    current_pst.pose.orientation.x = q.getX();
    current_pst.pose.orientation.y = q.getY();
    current_pst.pose.orientation.z = q.getZ();
    current_pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    for(double step=0.05;step<0.99;step+=0.05){
      pcl::PointXYZ pst_inter_polate_pc;
      pst_inter_polate_pc.x = current_pst.pose.position.x + vx*step;
      pst_inter_polate_pc.y = current_pst.pose.position.y + vy*step;
      pst_inter_polate_pc.z = current_pst.pose.position.z + vz*step;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree_map_->radiusSearch(pst_inter_polate_pc, perception_3d_ros_->getGlobalUtils()->getInscribedRadius(), pointIdxRadiusSearch, pointRadiusSquaredDistance)>1){
        //@ no line of sight, keep this pose
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      if(kdtree_ground_->radiusSearch(pst_inter_polate_pc, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<2){
        //@ not on the ground
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      double dx = vx*step;
      double dy = vy*step;
      double dr = sqrt(dx*dx+dy*dy);
      double dz = fabs(vz*step);
      if(dr>0.5 && atan2(dz,dr)>0.035){
        //@ z jump
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
      if(dr>20.0){
        //@ longer than 10 meter
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
    }
  }
  smoothed_path_id.push_back(path_id[path_id.size()-1]);
}

void Global_Planner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1){
      next_pst.pose.position.x = pcl_ground_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pcl_ground_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pcl_ground_->points[path_id[it+1]].z;
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
  
  if(kdtree_ground_->radiusSearch (pcl_goal, 0.5, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal)<1){
    RCLCPP_WARN(this->get_logger(), "Goal is not found.");
    RCLCPP_WARN(this->get_logger(), "Using vertical search to find a goal on the ground.");
    bool second_search = false;
    if(graph_ready_){
      for(double z=goal.pose.position.z; z>-10;z-=0.1){
        pointIdxRadiusSearch_goal.clear();
        pointRadiusSquaredDistance_goal.clear();
        pcl_goal.z = z;
        if(kdtree_ground_->radiusSearch(pcl_goal, 0.3, pointIdxRadiusSearch_goal, pointRadiusSquaredDistance_goal,0)>0)
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
    pcl_ground_->points[pointIdxRadiusSearch_goal[0]].x, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].y, pcl_ground_->points[pointIdxRadiusSearch_goal[0]].z);

  //--------------------------------------------------------------------------------------
  //@Get start ID
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZ pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;

  if(kdtree_ground_->radiusSearch (pcl_start, 0.5, pointIdxRadiusSearch_start, pointRadiusSquaredDistance_start)<1){
    RCLCPP_WARN(this->get_logger(), "Start is not found.");
    return false;
  }

  RCLCPP_WARN(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
    start.pose.position.x, start.pose.position.y, start.pose.position.z, pointIdxRadiusSearch_start[0], 
    pcl_ground_->points[pointIdxRadiusSearch_start[0]].x, pcl_ground_->points[pointIdxRadiusSearch_start[0]].y, pcl_ground_->points[pointIdxRadiusSearch_start[0]].z);

  start_id = pointIdxRadiusSearch_start[0];

  goal_id = pointIdxRadiusSearch_goal[0];

  return true;

}

void Global_Planner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){

  unsigned int start_id, goal_id;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
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
    //postSmoothPath(path, smoothed_path);
    getROSPath(path, ros_path);
    ros_path.poses.push_back(goal->goal);
    pub_path_->publish(ros_path);
    result->path = ros_path;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
  }
  
}

void Global_Planner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();
  pubStaticGraph();

  RCLCPP_INFO(this->get_logger(), "Static graph is generated with size: %lu", static_graph_.getSize());
   
  if(!has_initialized_){
    has_initialized_ = true;
    a_star_planner_ = std::make_shared<A_Star_on_Graph>(pcl_ground_, static_graph_, perception_3d_ros_);
    a_star_planner_->setupTurningWeight(turning_weight_);
  }
  else{
    a_star_planner_->updateGraph(pcl_ground_, static_graph_);
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

    ipt.x = pcl_ground_->points[(*it).first].x;
    ipt.y = pcl_ground_->points[(*it).first].y;
    ipt.z = pcl_ground_->points[(*it).first].z;
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
    p.x = pcl_ground_->points[(*it).first].x;
    p.y = pcl_ground_->points[(*it).first].y;
    p.z = pcl_ground_->points[(*it).first].z;
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
      p.x = pcl_ground_->points[(*it_set).first].x;
      p.y = pcl_ground_->points[(*it_set).first].y;
      p.z = pcl_ground_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

}