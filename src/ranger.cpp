#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <memory>
#include <filesystem>

using std::placeholders::_1;

StageNode::Vehicle::Ranger::Ranger(Stg::ModelRanger *m) 
: model(m)
{};
void StageNode::Vehicle::Ranger::init(){

}
void StageNode::Vehicle::Ranger::publish(){

}