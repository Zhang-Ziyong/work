#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "filesystem.hpp"
#include "Messenger.h"
#include "Glog.h"
#include "package.h"

using namespace ghc::filesystem;

class Player{
public:
    Player(sv::Svar config)
        : config(config){
      node = svar["Node"]("rosbag2cvte");

      sub_bag_header  = messenger.subscribe("bag_header",0,&Player::cbk_bag_header,this);
      sub_msgs = messenger.subscribe("message_pack",0,&Player::cbk_message_pack,this);
    }

    void spin(){
        svar["spin"](node);

        pubs.clear();// destroy
    }

    void cbk_bag_header(const sv::Svar& info){
        auto pub_queue_size = config.get("pub_queue_size",10);

        sv::Svar qos = svar["QoS"](pub_queue_size);
        if(config.get("best_effort",false))
            qos.call("best_effort");

        for(auto topic:info["topics"]){
            auto pub = node.call("create_publisher",topic["topic"],qos,topic["type"]);
            pubs.push_back(pub);
        }
    }

    void cbk_message_pack(const Package& package){
        pubs[package.hd.topic_id].call("publish",package.payload);
    }

    sv::Svar                 config,node,sub_bag_header,sub_msgs;
    std::vector<sv::Svar>    pubs;
};

class Reader{
public:
    Reader(sv::Svar config)
        : config(config){
        play_speed = config.get("play_speed",1.);
        std::string bag   = config.get<std::string>("bag","");

        if(path(bag).extension() == ".bagcvte")
        {
            bag_files.push_back(bag);
        }
        else
            for(auto fileit:directory_iterator(bag))
            {
                path path_ = fileit.path();
                if(path_.extension() != ".bagcvte") continue;
                bag_files.push_back(path_.string());
            }

        std::sort(bag_files.begin(),bag_files.end());

        LOG(INFO)<<"bag_files:"<<sv::Svar(bag_files);

        pub = messenger.advertise<Package>("message_pack",0);
        read_thread_ = std::thread([this](){read_thread();});
    }

    ~Reader(){
        read_thread_.join();
    }

    void prepare_bag_header(const Package& bag_header){
        std::string str (bag_header.payload.ptr(),bag_header.payload.size());

        sv::Svar info = sv::Svar::parse_json(str);
        std::string compress_method = info.get<std::string>("compression","none");
        if(compress_method != "none")
            decompress = svar[compress_method + "_decompress"];
        messenger.publish("bag_header",info);

        sv::Svar topics = info["topics"];
        std::vector<std::string> desire_topics = config["topics"].castAs<std::vector<std::string>>();

        if(desire_topics.size() == 0){
            topics_mask.resize(topics.size(),true);
            return;
        }

        std::set<std::string> desire_topic_set;

        for (std::string topic : desire_topics)
            desire_topic_set.insert(topic);

        topics_mask.clear();
        for(sv::Svar topic: topics){
            int count = desire_topic_set.count(topic["topic"].as<std::string>());
            if(count)
                topics_mask.push_back(false);
            else
                topics_mask.push_back(true);
        }
    }

    void decompress_package(Package& package){
        if(! decompress.isFunction()) return;

        package.payload = decompress(package.payload).as<sv::SvarBuffer>();
        package.hd.payload_size = package.payload.size();

        if(package.hd.payload_size == 0)
            LOG(ERROR) <<"package is empty!";
    }

    void read_thread(){
        from = config.get<double>("from",-1.) * 1e9;
        to   = config.get<double>("to",33205538041) * 1e9;

        for(auto bag_file: bag_files)
            read_file(bag_file);

        LOG(INFO)<<"play is done.";
        exit(0);
    }

    void read_file(std::string bag_file){
        std::ifstream ifs(bag_file, std::ios::in|std::ios::binary);
        if(!ifs.is_open())
            LOG(FATAL) <<"Unable to open file " << bag_file;

        Package package;
        if(!read_package(package, ifs))
            LOG(FATAL) <<"Unable to obtain header info";
        prepare_bag_header(package);

        if(!seek_package(package, ifs, from))
            return;

        start_play_time = timestamp();
        first_bag_time  = package.hd.timestamp_nano;

        double factor = 1e-9 / play_speed;
        bool not_realtime_warning=true;

        LOG(INFO)<<"Start play "<<bag_file;

        double last_echo_time = -1;
        int    count = 0;
        while (rclcpp::ok() && read_package(package,ifs)) {
            if(package.hd.timestamp_nano > to){
                LOG(INFO) << "Time reached, play done. Now: "<<package.hd.timestamp_nano<<", to: "<<to;
                break;
            }

            if(!topics_mask[package.hd.topic_id]) continue;

            decompress_package(package);

            double should_sleep = (package.hd.timestamp_nano - first_bag_time) * factor - (timestamp() - start_play_time);

            if(should_sleep < -2 & not_realtime_warning){
                not_realtime_warning = false;
                LOG(WARNING) << "Play speed not realtime!";
            }
            else if(should_sleep > 0.01)
                std::this_thread::sleep_for(std::chrono::nanoseconds(int64_t(should_sleep*1e9)));

            pub.publish(package);
            ++count;

            auto now_time = package.hd.timestamp_nano * 1e-9;
            if(now_time > last_echo_time + 0.1){
                printf("\rtime: %.3f, count: %d", now_time, count);
                std::cout.flush();
                last_echo_time = now_time;
            }
        }

        std::cout<<"\r";
    }

    bool read_package(Package& package, std::ifstream& in){
        in.read((char*)&package.hd,sizeof(package.hd));
        package.payload = sv::SvarBuffer(package.hd.payload_size);
        in.read(package.payload.ptr(),package.hd.payload_size);
        return in.good();
    }

    bool seek_package(Package& package, std::ifstream& in, int64_t from){
        while(in.good()){
            in.read((char*)&package.hd,sizeof(package.hd));
            if(package.hd.timestamp_nano >= from)
                break;

            in.seekg(package.hd.payload_size, std::ios::cur);
        }

        if(!in.good()) return false;

        in.seekg(- sizeof(package.hd), std::ios::cur);

        return in.good();
    }

    double timestamp(){
        return clock.now().seconds();
    }

    sv::Svar         config;
    GSLAM::Publisher pub;
    double           play_speed, start_play_time=-1;
    int64_t          first_bag_time=-1, from, to;
    sv::Svar         decompress;
    std::vector<std::string> bag_files;
    std::thread      read_thread_;
    rclcpp::Clock    clock;

    std::vector<bool> topics_mask;
};

int play_main(sv::Svar config){
    std::string bag   = config.arg<std::string>("bag","","The bag file/folder to play");
    double play_speed = config.arg<double>("play_speed",1.,"play speed");
    double from = config.arg<double>("from",-1.,"the begin time");
    double to   = config.arg<double>("to",4804302841,"the end time"); // magic number
    int    pub_queue_size = config.arg("pub_queue_size",10,"default queue size for publishers");
    bool        best_effort = config.arg<bool>("best_effort",true,"use best_effort policy or not");
    std::string topics_str  = config.arg<std::string>("skip","","topics to ignore, please use ',' to seprate");

    std::vector<std::string> topics;
    while(!topics_str.empty()){
        int idx = topics_str.find(',');
        if(idx == std::string::npos){
            topics.push_back(topics_str);
            break;
        }

        topics.push_back(topics_str.substr(0,idx));
        topics_str = topics_str.substr(idx+1);
    }

    config["topics"] = topics;

    if(config.get("help",false))
        return config.help();

    if(bag == ""){
        LOG(ERROR) << "Please input bag file path to play.";
        return -1;
    }

    LOG(INFO)<<"Openning "<<bag;

    svar["init"](sv::Svar({"rosbag2"}));

    Player   player(config);
    Reader   reader(config);

    player.spin();

    return 0;
}

REGISTER_SVAR_MODULE(play){
    svar["apps"]["play"]={play_main,"functional like ros2 bag play"};
}
