#include "Svar.h"

int publish_exception(sv::Svar config){
    std::string topic   = config.arg<std::string>("topic","/exception","exception topic name with type std_msgs/msg/String to "
                                                                     "control important clip save from memory");
    std::string exception = config.arg<std::string>("exception","unkown","exception name, will occur in bag file name");

    if(config.get("help",false))
        return config.help();

    svar["init"](std::vector<std::string>({"publish_condition"}));

    sv::Svar node=svar["Node"]("publish_condition");
    sv::Svar pub =node.call("create_publisher",topic,svar["QoS"](1),"std_msgs/msg/String");

    pub.call("publish",sv::Svar({{"data",exception}}));

    svar["spinSome"](node);

    usleep(1000000);

    return 0;
}

REGISTER_SVAR_MODULE(publish_exception){
    svar["apps"]["publish_exception"]={publish_exception,"publish exception"};
}
