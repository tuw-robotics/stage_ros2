#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

    bool gui = true;
    bool use_model_names = false;
    for(int i=0;i<(argc-1);i++)
    {
        if(!strcmp(argv[i], "-g"))
            gui = false;
        if(!strcmp(argv[i], "-u"))
            use_model_names = true;
    }
  auto node = std::make_shared<StageNode>(rclcpp::NodeOptions());
  node->init(argc-1,argv, gui, argv[argc-1], use_model_names);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
