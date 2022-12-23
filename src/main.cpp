#include <memory>
#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int 
main(int argc, char** argv)
{
    if( argc < 2 )
    {
        puts(USAGE);
        exit(-1);
    }

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

    StageNode sn(argc-1,argv,gui,argv[argc-1], use_model_names);

    if(sn.SubscribeModels() != 0)
        exit(-1);

    std::thread t = std::thread([&sn](){rclcpp::spin(sn.n_);});

    sn.world->Start();

    Stg::World::Run();

    return 0;
}