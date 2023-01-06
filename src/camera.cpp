#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

using std::placeholders::_1;

StageNode::Vehicle::Camera::Camera(Stg::ModelCamera *m) 
: model(m)
{};
void StageNode::Vehicle::Camera::init(){

}
void StageNode::Vehicle::Camera::publish(){
    
}