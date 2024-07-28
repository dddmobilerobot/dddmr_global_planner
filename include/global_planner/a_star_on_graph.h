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

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 

/*For pcl::PointXYZ*/
#include <pcl/common/geometry.h>
#include <math.h>

/*
for graph
type edge_t is defined here
type graph_t is defined here
*/

/*For perception*/
#include <perception_3d/perception_3d_ros.h>

typedef struct {
  unsigned int self_index;
  float g, h, f;
  unsigned int parent_index; 
  bool is_closed, is_opened;
} Node_t;

typedef std::pair<double, unsigned int> f_p_;

class AstarList{
  public:
    AstarList(perception_3d::StaticGraph& static_graph);

    void Initial();
    void setGraph(perception_3d::StaticGraph& static_graph);
    void updateNode(Node_t& a_node);
    void closeNode(Node_t& a_node);
    float getGVal(Node_t& a_node);
    Node_t getNode_wi_MinimumF();
    Node_t getNode(unsigned int node_index);
    bool isClosed(unsigned int node_index);
    bool isOpened(unsigned int node_index);
    bool isFrontierEmpty();
  private:

    /*Static graph is for path planning and list*/
    perception_3d::StaticGraph static_graph_;
    
    /*
    We handle closed list and accessory by unordered::map to get time complexity of O(1)
    Key: node id
    content: Node
    */
    std::unordered_map<unsigned int, Node_t> as_list_;

    /*
    We push back a pair <f,node_index> by the priority, hence ,the time complexity of checking minimum f is O(1)
    */
    //std::priority_queue<f_p_> f_priority_queue_;
    std::set<f_p_> f_priority_set_;
};

class A_Star_on_Graph{

    public:
      A_Star_on_Graph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
        perception_3d::StaticGraph& static_graph,
        std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros);
      
      ~A_Star_on_Graph();
      
      void updateGraph(pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up, 
                                  perception_3d::StaticGraph& static_graph);

      void getPath( unsigned int start, unsigned int goal, std::vector<unsigned int>& path);
      
      void setupTurningWeight(double m_weight){turning_weight_ = m_weight;}
    private:
      pcl::PointCloud<pcl::PointXYZ>::Ptr pc_original_z_up_;

      perception_3d::StaticGraph static_graph_; 
      /*Provide dynamic graph for obstacle avoidance*/
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_ros_;
      
      /*Create the list*/
      AstarList* ASLS_;

      //@ turning weight of the node
      double turning_weight_;

      double getThetaFromParent2Expanding(pcl::PointXYZ m_pcl_current_parent, pcl::PointXYZ m_pcl_current, pcl::PointXYZ m_pcl_expanding);
};

