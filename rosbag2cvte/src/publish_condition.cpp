#include "Svar.h"

int publish_condition(sv::Svar config){
    std::string topic   = config.arg<std::string>("topic","/condition","condition topic name with type std_msgs/msg/Bool "
                                                                       "to control whether idle or not");
    bool        on = config.arg<bool>("on",false,"Default publish false, -on to publish true");

    if(config.get("help",false))
        return config.help();

    svar["init"](std::vector<std::string>({"record_condition"}));

    sv::Svar node=svar["Node"]("record_condition");
    sv::Svar pub =node.call("create_publisher",topic,svar["QoS"](1),"std_msgs/msg/Bool");

    pub.call("publish",sv::Svar({{"data",on}}));

    svar["spinSome"](node);

    usleep(1000000);

    return 0;
}

REGISTER_SVAR_MODULE(publish_condition){
    svar["apps"]["publish_condition"]={publish_condition,"publish condition"};
}
