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
#include <global_planner/a_star_on_graph.h>

AstarList::AstarList(perception_3d::StaticGraph& static_graph){
  static_graph_ = static_graph;
}

void AstarList::setGraph(perception_3d::StaticGraph& static_graph){
  static_graph_ = static_graph;
}

void AstarList::Initial(){
  as_list_.clear();
  graph_t* tmp_graph_t; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  tmp_graph_t = static_graph_.getGraphPtr();
  for(auto it=(*tmp_graph_t).begin();it!=(*tmp_graph_t).end();it++){
    Node_t new_node = {.self_index=0, .g=0, .h=0, .f=0, .parent_index=0, .is_closed=false, .is_opened=false};
    as_list_[(*it).first] = new_node;
  }
  f_priority_set_.clear();
}

Node_t AstarList::getNode(unsigned int node_index){

  return as_list_[node_index];
}

float AstarList::getGVal(Node_t& a_node){
  return as_list_[a_node.self_index].g;
}

void AstarList::closeNode(Node_t& a_node){
  as_list_[a_node.self_index].is_closed = true;
}

void AstarList::updateNode(Node_t& a_node){
  as_list_[a_node.self_index] = a_node;
  f_p_ afp;
  afp.first = a_node.f; //made minimum f to be top so we can pop it
  afp.second = a_node.self_index;
  f_priority_set_.insert(afp);
  //ROS_DEBUG("Add node ---> %u with g: %f, h: %f, f: %f",a_node.self_index, a_node.g, a_node.h, a_node.f);
}

Node_t AstarList::getNode_wi_MinimumF(){
  auto first_it = f_priority_set_.begin();
  Node_t m_node = as_list_[(*first_it).second];
  if(!m_node.is_closed){
    f_priority_set_.erase(first_it);
    return m_node;
  }
  
  //Because we updateNode node even when new g value is smaller than that in openlist
  //We will have duplicate f value in the f_priority_set_
  int concern_cnt = 0;
  while(m_node.is_closed && !f_priority_set_.empty()){
    concern_cnt++;
    f_priority_set_.erase(first_it);
    first_it = f_priority_set_.begin();
    m_node = as_list_[(*first_it).second];
  }
  return m_node;
}

bool AstarList::isClosed(unsigned int node_index){
  return as_list_[node_index].is_closed;
}

bool AstarList::isOpened(unsigned int node_index){
  return as_list_[node_index].is_opened;
}

bool AstarList::isFrontierEmpty(){
  return f_priority_set_.empty();
}

//@----------------------------------------------------------------------------------------

A_Star_on_Graph::A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph, 
                                  std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros){
  static_graph_ = static_graph;
  perception_ros_ = perception_ros;

  pc_original_z_up_ = pc_original_z_up;
  ASLS_ = new AstarList(static_graph_);
}

A_Star_on_Graph::~A_Star_on_Graph(){
  if(ASLS_)
    delete ASLS_;
}

void A_Star_on_Graph::updateGraph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph){
  
  
  static_graph_ = static_graph;
  pc_original_z_up_ = pc_original_z_up;
  ASLS_->setGraph(static_graph_);
}

void A_Star_on_Graph::getPath(
  unsigned int start, unsigned int goal,
  std::vector<unsigned int>& path){

  //ROS_DEBUG("Start: %u, Goal: %u", start, goal);

  /*
  Create the first node which is start and add into frontier
  */

  pcl::PointXYZ pcl_goal = pc_original_z_up_->points[goal];
  pcl::PointXYZ pcl_start = pc_original_z_up_->points[start];
  float f = sqrt(pcl::geometry::squaredDistance(pcl_start, pcl_goal));
  Node_t current_node = {.self_index=start, .g=0, .h=0, .f=f, .parent_index=start, .is_closed=false, .is_opened=true};

  ASLS_->Initial();
  ASLS_->updateNode(current_node);
  
  double inscribed_radius = perception_ros_->getGlobalUtils()->getInscribedRadius();
  double inflation_descending_rate = perception_ros_->getGlobalUtils()->getInflationDescendingRate();
  double max_obstacle_distance = perception_ros_->getGlobalUtils()->getMaxObstacleDistance();

  while(!ASLS_->isFrontierEmpty()){ 
    /*Pop minimum F, we leverage prior queue, so we dont need to loop frontier everytime*/
    current_node = ASLS_->getNode_wi_MinimumF();
    //ROS_DEBUG("Expand node: %u", current_node.self_index);
    /*Get successors*/
    auto successors = static_graph_.getEdge(current_node.self_index);
    for(auto it = successors.begin(); it!=successors.end(); it++){

      //@ dGraphValue is the distance to lethal
      double dGraphValue = perception_ros_->get_min_dGraphValue((*it).first);

      /*This is for lethal*/
      if(dGraphValue<inscribed_radius){
        //ROS_DEBUG("%.2f,%.2f,%.2f, v: %.2f",pc_original_z_up_->points[(*it).first].x,pc_original_z_up_->points[(*it).first].y,pc_original_z_up_->points[(*it).first].z, dGraphValue);
        continue;
      }
      
      double factor = exp(-1.0 * inflation_descending_rate * (dGraphValue - inscribed_radius));

      float new_g = current_node.g + (*it).second * static_graph_.getNodeWeight((*it).first) + factor * 1.0;
      pcl::PointXYZ pcl_current = pc_original_z_up_->points[(*it).first];
      float new_h = sqrt(pcl::geometry::squaredDistance(pcl_current, pcl_goal));
      float new_f = new_g + new_h;

      Node_t new_node = {.self_index=((*it).first), .g=new_g, .h=new_h, .f=new_f, .parent_index=current_node.self_index, .is_closed=false, .is_opened=true};

      /*Check is in closed list*/
      if(ASLS_->isClosed((*it).first))
        continue;
      /*Check is in opened list*/
      else if(ASLS_->isOpened((*it).first)){
        if(ASLS_->getGVal(new_node)>new_g){
          ASLS_->updateNode(new_node);          
        }
      }
      /*addNode*/
      else{
        ASLS_->updateNode(new_node);
      }
        
      
    }

    /*Close this node*/
    ASLS_->closeNode(current_node);

    /*If goal is in closed list, we are done*/
    if(ASLS_->isClosed(goal)){
      //ROS_DEBUG("Found path");
      Node_t trace_back = ASLS_->getNode(goal);
      while(trace_back.self_index!=trace_back.parent_index){
        path.push_back(trace_back.self_index);
        trace_back = ASLS_->getNode(trace_back.parent_index);
      }
      path.push_back(trace_back.self_index);//Push start point
      std::reverse(path.begin(),path.end()); 
      break;
    }

    /*Check if*/
  }

}
