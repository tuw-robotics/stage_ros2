#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

using std::placeholders::_1;

StageNode::Vehicle::Camera::Camera(Stg::ModelCamera *m, std::shared_ptr<Vehicle> &v, StageNode *n) 
: model(m), vehicle(v), node(n)
{};
void StageNode::Vehicle::Camera::init(){

}
void StageNode::Vehicle::Camera::publish_msg(){
    
}
void StageNode::Vehicle::Camera::publish_tf(){
    
}