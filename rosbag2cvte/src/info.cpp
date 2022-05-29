#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "filesystem.hpp"
#include "Messenger.h"
#include "Glog.h"
#include "package.h"

using namespace ghc::filesystem;

sv::Svar info_file(std::string file, bool full){
    std::ifstream in(file, std::ios::in|std::ios::binary);
    Package package;
    in.read((char*)&package.hd,sizeof(package.hd));
    std::string str;
    str.resize(package.hd.payload_size);
    in.read((char*)str.data(),str.size());

    sv::Svar info = sv::Svar::parse_json(str);
    sv::Svar topics = info["topics"];
    if(!full) return info;

    int64_t min_t=std::numeric_limits<int64_t>::max(),max_t=-1;
    std::vector<int> counts(topics.size(),0);

    while(in.good()){
        in.read((char*)&package.hd,sizeof(package.hd));
        in.seekg(package.hd.payload_size, std::ios::cur);
        min_t = std::min(package.hd.timestamp_nano, min_t);
        max_t = std::max(package.hd.timestamp_nano,max_t);
        counts[package.hd.topic_id]++;
    }

    for(int i=0;i<counts.size();i++)
        topics[i]["count"] = counts[i];

    info["start"] = min_t * 1e-9;
    info["end"] = max_t * 1e-9;
    info["duration"] = info["end"] - info["start"];

    return info;
}

int info_main(sv::Svar config){
    std::string bag   = config.arg<std::string>("bag","","The bag file/folder to play");
    bool        full  = config.arg<bool>("full",false,"full info or not");

    if(config.get("help",false))
        return config.help();

    if(bag.empty())
        LOG(FATAL) << "Please input bag file path to play.";
    sv::Svar info;

    if(path(bag).extension() == ".bagcvte")
    {
        info = info_file(bag,full);
        std::cerr<< info <<std::endl;
        return 0;
    }

    std::vector<std::string> paths;

    for(auto fileit:directory_iterator(bag))
    {
        path path_ = fileit.path();
        if(path_.extension() != ".bagcvte") continue;

        paths.push_back(path_.string());
    }

    std::sort(paths.begin(),paths.end());

    for(auto path_:paths){
        LOG(INFO)<<"Load "<<path_;
        sv::Svar clip_info = info_file(path_,full);
        if(info.isUndefined()){
            info=clip_info;
            continue;
        }

        if(!full) continue;

        sv::Svar topics_dst = info["topics"];
        sv::Svar topics_src = clip_info["topics"];
        for(int i = 0; i < topics_dst.size(); i++){
            topics_dst[i]["count"] = topics_dst[i]["count"] + topics_src[i]["count"];
        }

        info["end"] = clip_info["end"];
    }

    if(full)
        info["duration"] = info["end"] - info["start"];

    std::cerr<< info << std::endl;
    return 0;
}

REGISTER_SVAR_MODULE(info){
    svar["apps"]["info"]={info_main,"functional like ros2 bag info"};
}
