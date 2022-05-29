#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>

#include "Svar.h"
#include "register.h"

using namespace sv;
using namespace rclcpp;

enum SubscriptionFormat{
    SerializedFormat = 0 ,
    AnyMessageFormat = 1 ,
    JSONFormat = 2 ,
    CBORFormat = 3
};

struct Supports{
    const rosidl_message_type_support_t* cpp_support;
    const rosidl_message_type_support_t* icl_support;
};

class AnyMessage{
public:
    AnyMessage(const Supports* support)
        : support(support){
        auto intro_ts_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(support->cpp_support->data);
        msg_mem = malloc(intro_ts_members->size_of_);
        intro_ts_members->init_function(msg_mem,rosidl_runtime_cpp::MessageInitialization());
    }

    ~AnyMessage(){
        auto intro_ts_members = static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(support->cpp_support->data);
        intro_ts_members->fini_function(msg_mem);
        free(msg_mem);
    }

    void serialize_message(rclcpp::SerializedMessage * message){
        SerializationBase serializer(support->icl_support);
        serializer.serialize_message(msg_mem,message);
    }

    void deserialize_message(rclcpp::SerializedMessage * message){
        SerializationBase serializer(support->icl_support);
        serializer.deserialize_message(message,msg_mem);
    }

    sv::SvarBuffer serialize(){
        SerializationBase serializer(support->icl_support);
        std::shared_ptr<rclcpp::SerializedMessage> message = std::make_shared<rclcpp::SerializedMessage>();
        serializer.serialize_message(msg_mem,message.get());

        return sv::SvarBuffer(message->get_rcl_serialized_message().buffer,message->size(),message);
    }

    void deserialize(sv::SvarBuffer buf){
        std::shared_ptr<rclcpp::SerializedMessage> message = std::make_shared<rclcpp::SerializedMessage>();
        rcl_serialized_message_t& serialized = message->get_rcl_serialized_message();
        rcl_serialized_message_t tmp= serialized;
        serialized.buffer = buf.ptr<uint8_t>();
        serialized.buffer_length = serialized.buffer_capacity = buf.size();
        deserialize_message(message.get());
        serialized = tmp;
    }

    void from_svar(const Svar &v){
        set_value(support->cpp_support,msg_mem,v);
    }

    sv::Svar to_cbor(){
        sv::Svar ret;
        get_value(support->cpp_support,msg_mem,ret,true);
        return ret;
    }

    sv::Svar to_json(){
        sv::Svar ret;
        get_value(support->cpp_support,msg_mem,ret,false);
        return ret;
    }

    const Supports* support;
    void* msg_mem;
};

class AnyPublisher : public rclcpp::PublisherBase
{
public:
  explicit AnyPublisher(rclcpp::Node * node, std::string topic, const Supports* type,
                        const rcl_publisher_options_t & publisher_options)
  : rclcpp::PublisherBase(node->get_node_base_interface().get(), topic, *type->icl_support, publisher_options),
    support(type)
    {    }

    void publish(sv::Svar msg){
        if(msg.is<SerializedMessage>())
        {
            rcl_serialized_message_t rcl_msg = msg.as<SerializedMessage>().get_rcl_serialized_message();
            auto status = rcl_publish_serialized_message(publisher_handle_.get(), &rcl_msg , nullptr);
            if (RCL_RET_OK != status) {
              rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
            }
        }
        else if(msg.is<SvarBuffer>()){
            SvarBuffer& buf=msg.as<SvarBuffer>();
            std::shared_ptr<rclcpp::SerializedMessage> message = std::make_shared<rclcpp::SerializedMessage>();
            rcl_serialized_message_t& serialized = message->get_rcl_serialized_message();
            rcl_serialized_message_t tmp= serialized;
            serialized.buffer = buf.ptr<uint8_t>();
            serialized.buffer_length = serialized.buffer_capacity = buf.size();
            auto status = rcl_publish_serialized_message(publisher_handle_.get(), &serialized , nullptr);
            if (RCL_RET_OK != status) {
              rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
            }
            serialized = tmp;
        }
        else if(msg.is<AnyMessage>()){
            rclcpp::SerializedMessage message;
            msg.as<AnyMessage>().serialize_message(&message);

            rcl_serialized_message_t rcl_msg = message.get_rcl_serialized_message();
            auto status = rcl_publish_serialized_message(publisher_handle_.get(), &rcl_msg , nullptr);
            if (RCL_RET_OK != status) {
              rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
            }
        }
        else if(msg.isObject()){
            AnyMessage rawmsg(support);
            rawmsg.from_svar(msg);

            rclcpp::SerializedMessage message;
            rawmsg.serialize_message(&message);

            rcl_serialized_message_t rcl_msg = message.get_rcl_serialized_message();
            auto status = rcl_publish_serialized_message(publisher_handle_.get(), &rcl_msg , nullptr);
            if (RCL_RET_OK != status) {
              rclcpp::exceptions::throw_from_rcl_error(status, "failed to publish serialized message");
            }
        }
    }

    int  sub_count(){
        return get_subscription_count();
    }

    std::string topic_name(){
        return get_topic_name();
    }

    const Supports* support;
};

class AnySubscription : public rclcpp::SubscriptionBase
{
public:
  explicit AnySubscription(rclcpp::Node * node, std::string topic, const rcl_subscription_options_t & subscription_options,
                            const Supports* type_support,
                            sv::Svar cbk, int format = CBORFormat)
  : rclcpp::SubscriptionBase(node->get_node_base_interface().get(),
                             *type_support->icl_support, topic, subscription_options, true),
                            support(type_support), cbk(cbk), format(format) {
  }

    int  pub_count(){
        return get_publisher_count();
    }

    std::string topic_name(){
        return get_topic_name();
    }

  std::shared_ptr<void> create_message() override {
      return nullptr;
  }

  std::shared_ptr<rclcpp::SerializedMessage>
  create_serialized_message() override {return std::make_shared<rclcpp::SerializedMessage>();}

  void handle_message(std::shared_ptr<void>& msg, const rclcpp::MessageInfo&) override {
      std::shared_ptr<SerializedMessage> serialized = std::static_pointer_cast<SerializedMessage>(msg);
      switch (format) {
      case SerializedFormat:
          cbk(serialized);
          break;
      case AnyMessageFormat:{
          auto anymsg = std::make_shared<AnyMessage>(support);
          anymsg->deserialize_message(serialized.get());
          cbk(anymsg);
      }
          break;
      case CBORFormat:{
          AnyMessage anymsg(support);
          anymsg.deserialize_message(serialized.get());
          cbk(anymsg.to_cbor());
      }
          break;
      case JSONFormat:{
          AnyMessage anymsg(support);
          anymsg.deserialize_message(serialized.get());
          cbk(anymsg.to_json());
      }
          break;
      default:
          break;
      }
  }

  void handle_loaned_message(void *, const rclcpp::MessageInfo &) override {}
  void return_message(std::shared_ptr<void> &) override {}
  void return_serialized_message(std::shared_ptr<rclcpp::SerializedMessage> &) override {}

  const Supports* support;
  sv::Svar cbk;
  int  format;
};

class AnyNode{
public:
    AnyNode(std::string name)
        : node(std::make_shared<Node>(name)){

    }

    std::shared_ptr<AnyPublisher> create_publisher(const std::string &topic_name, const QoS &qos,std::string type_name){
        const Supports* support = get_support_t(get_type_by_topic(type_name,topic_name));

        rcl_publisher_options_t options = rcl_publisher_get_default_options();
        options.qos = qos.get_rmw_qos_profile();

        auto pub = std::make_shared<AnyPublisher>(node.get(), topic_name, support, options);

        using rclcpp::node_interfaces::get_node_topics_interface;
        auto node_topics = get_node_topics_interface(node);
        node_topics->add_publisher(pub, nullptr);

        return pub;
    }

    std::shared_ptr<AnySubscription> create_subscription(std::string topic_name, QoS qos, Svar cbk,
                                                         std::string type_name, int format = CBORFormat){
        const Supports* support = get_support_t(get_type_by_topic(type_name,topic_name));

        rcl_subscription_options_t options = rcl_subscription_get_default_options();
        options.qos = qos.get_rmw_qos_profile();

        auto sub= std::make_shared<AnySubscription>(node.get(), topic_name, options, support, cbk, format);

        using rclcpp::node_interfaces::get_node_topics_interface;
        auto node_topics = get_node_topics_interface(node);
        node_topics->add_subscription(sub,nullptr);

        return sub;
    }

    static const Supports* get_support_t(std::string type){
        static std::map<std::string,std::shared_ptr<rcpputils::SharedLibrary>> libraries; // WARNING: Important not to release
        static std::map<std::string,std::shared_ptr<Supports>> supports; // WARNING: Important not to release

        if(supports.count(type))
            return supports[type].get();

        auto library = rosbag2_cpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
        libraries[type] = library;

        std::shared_ptr<Supports> support= std::make_shared<Supports>();
        support->icl_support = rosbag2_cpp::get_typesupport_handle(type, "rosidl_typesupport_cpp", library);
        support->cpp_support = get_message_typesupport_handle(support->icl_support, "rosidl_typesupport_introspection_cpp");
        supports[type] = support;

        return support.get();
    }

    std::string get_type_by_topic(std::string type, std::string topic){
        if(!type.empty()) return type;

        std::vector<std::string> types;
        for(int i=0;i<100;i++){
            auto topic_names_and_types = node->get_topic_names_and_types();
            types = topic_names_and_types[topic];
            if(types.empty()&& i==99)
                throw SvarExeption(topic + " can not find type.");
            usleep(10000); // 10ms
        }
        return types.front();
    }

    Node::SharedPtr node;
};

Svar load_class(std::string type_name){
    const Supports* support = AnyNode::get_support_t(type_name);
    return Svar();
}

int test_anyros(Svar config){
    std::string topic = config.arg<std::string>("topic","/hello_ros2","topic name");
    std::string type  = config.arg<std::string>("type","std_msgs/msg/Int32","type name");
    int        queue  = config.arg<int>("queue",10,"queue size");
    int        format = config.arg<int>("format",2,"0:serialized, 1: anymessage, 2: json, 3: cbor");

    svar["init"](std::vector<std::string>({"test_anyros"}));

    AnyNode node("test_anyros");

    auto pub = node.create_publisher(topic,queue,type);
    auto sub = node.create_subscription(topic,queue,[](sv::Svar msg){
            std::cout<<"received:"<<msg<<std::endl;
     },type,format);


    Rate rate(1);
    for(int i=0;i<1000 && ok(); i++){
        pub->publish(Svar::object());
        spin_some(node.node);
        rate.sleep();
    }

    return 0;
}


REGISTER_SVAR_MODULE(rclcpp){
    svar["CBORFormat"] = (int)CBORFormat;
    svar["JSONFormat"] = (int)JSONFormat;
    svar["SerializedFormat"] = (int)SerializedFormat;
    svar["AnyMessageFormat"] = (int)AnyMessageFormat;

    svar.def("init",[](int argc,char** argv){
        return rclcpp::init(argc,argv);
    });

    svar["init"].overload([](std::vector<std::string> args){
        std::vector<char*> argv;
        for(std::string& arg:args) argv.push_back(&arg[0]);
        rclcpp::init(args.size(),argv.data());
    });
//    },arg("args") = std::vector<std::string>({})); FIXME : why kwargs lead to crash?

    svar["spin"]= [](AnyNode& node){rclcpp::spin(node.node);};
    svar["spinSome"] = [](AnyNode& node){rclcpp::spin_some(node.node);};
    svar["ok"]= [](){return rclcpp::ok();};
    svar["get_support_t"] = &AnyNode::get_support_t;

    Class<QoS>("QoS")
            .construct<int>()
            .def("keep_all",&QoS::keep_all)
            .def("reliable",&QoS::reliable)
            .def("best_effort",&QoS::best_effort)
            .def("durability_volatile",&QoS::durability_volatile);

    Class<Rate>("Rate")
         .construct<double>()
         .def("sleep",&Rate::sleep);

    Class<AnyMessage>("Message")
            .construct<const Supports*>()
            .def("serialize_message",&AnyMessage::serialize_message)
            .def("deserialize_message",&AnyMessage::deserialize_message)
            .def("serialize",&AnyMessage::serialize)
            .def("deserialize",&AnyMessage::deserialize)
            .def("from_svar",&AnyMessage::from_svar)
            .def("to_cbor",&AnyMessage::to_cbor)
            .def("to_json",&AnyMessage::to_json)
            .def("dict",&AnyMessage::to_json);

    Class<AnySubscription>("Subscription")
            .def("pub_count",&AnySubscription::pub_count)
            .def("topic_name",&AnySubscription::topic_name);

    Class<AnyPublisher>("Publisher")
            .def("publish",&AnyPublisher::publish)
            .def("sub_count",&AnyPublisher::sub_count)
            .def("topic_name",&AnyPublisher::topic_name);

    Class<AnyNode>("Node")
            .construct<std::string>()
            .def_static("get_support_t",&AnyNode::get_support_t)
            .def("get_type_by_topic",&AnyNode::get_type_by_topic)
            .def("create_publisher",&AnyNode::create_publisher,"typename"_a="")
            .def("create_subscription",&AnyNode::create_subscription,"typename"_a="","format"_a=int(CBORFormat));

    Class<SerializedMessage>("SerializedMessage")
            .def("__buffer__",[](std::shared_ptr<SerializedMessage> msg){
        auto m = msg->get_rcl_serialized_message();
        return sv::SvarBuffer(m.buffer,m.buffer_length,msg);
    });

    svar["apps"]["test_anyros"]={test_anyros,"test any ros"};

}

EXPORT_SVAR_INSTANCE



