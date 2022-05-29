#include "Svar.h"
#include "Messenger.h"
#include "Glog.h"
#include "rclcpp/clock.hpp"
#include "package.h"
#include "filesystem.hpp"

class Recorder{
public:
    Recorder(sv::Svar config)
        :config(config){
      topics = config["topics"].castAs<std::vector<std::string>>();
      int sub_queue_size = config.get("sub_queue_size",10);
      int write_queue_size = config.get("write_queue_size",1000);
      sv::Svar resamples = config["resamples"];

      pub = messenger.advertise<sv::Svar>("msg_queue",write_queue_size);
      node = svar["Node"]("rosbag2cvte");
      usleep(10000);// FIXME: sometimes get_type_by_topic fails, this helps or not?

      for(int i = 0;i< topics.size(); i++){
          auto type=node.call("get_type_by_topic","",topics[i]);
          types.push_back(type.as<std::string>());
          desire_freq.push_back(resamples.get(topics[i],10000.));
      }

      write_info();

      counts.resize(topics.size(),0);

      sv::Svar qos = svar["QoS"](sub_queue_size);
      if(config.get("best_effort",false))
          qos.call("best_effort");

      for(int i = 0;i< topics.size(); i++){
          subs.push_back(node.call("create_subscription",topics[i],qos,[this,i](sv::SvarBuffer msg){
              write_msg(i, msg);
          },types[i],svar["SerializedFormat"]));

          LOG(INFO)<< "Subscribed to " << topics[i];
      }

      std::string condition_name = config.get<std::string>("condition","");
      if(!condition_name.empty())
          subs.push_back(node.call("create_subscription",condition_name,qos,[this](sv::Svar msg){
              cbk_condition(msg);
          },"std_msgs/msg/Bool",svar["CBORFormat"]));

      std::string exception_name   = config.get<std::string>("exception","");
      if(!exception_name.empty())
          subs.push_back(node.call("create_subscription",exception_name,qos,[this](sv::Svar msg){
              messenger.publish("exception", msg["data"].castAs<std::string>());
          },"std_msgs/msg/String",svar["CBORFormat"]));

      std::cout<<"All topics are subscribed."<<std::endl;
    }

    void spin(){
        svar["spin"](node);

        subs.clear();// destroy subs

        // prepare info and write
        sv::Svar info;
        std::vector<sv::Svar> topic_list;
        for(int i=0;i<topics.size();i++){
            sv::Svar t;
            t["topic"]=topics[i];
            t["type"] =types[i];
            t["count"]=counts[i];
            topic_list.push_back(t);
        }
        info["topics"] = topic_list;

        std::ofstream ofs("info.json");
        ofs << info.dump_json();
    }

    void write_info(){
        // prepare info and write
        auto compress_m   = config.get<std::string>("compression","none");
        sv::Svar info;
        std::vector<sv::Svar> topic_list;
        for(int i=0;i<topics.size();i++){
            sv::Svar t;
            t["topic"]=topics[i];
            t["type"]=types[i];
            topic_list.push_back(t);
        }
        info["topics"] = topic_list;
        info["compression"] = compress_m;
        std::string json = info.dump_json();

        Package package;
        package.hd.timestamp_nano = timestamp();
        package.payload = sv::SvarBuffer(json.data(),json.size()).clone();
        package.hd.payload_size = package.payload.size();
        package.hd.topic_id = -1;

        start_time = package.hd.timestamp_nano;

        messenger.publish("bag_header",package);
    }

    void write_msg(int topic_id, sv::SvarBuffer msg){
        if(!condition) return;
        auto now_time = timestamp();

        if( counts[topic_id] > desire_freq[topic_id] * (now_time - start_time) * 1e-9) return;

        Package package;
        package.hd.timestamp_nano = now_time;
        package.payload = msg;
        package.hd.payload_size = package.payload.size();
        package.hd.topic_id = topic_id;
        ++counts[topic_id];
        pub.publish(package);
    }

    void cbk_condition(sv::Svar msg){
        auto cond = msg["data"].castAs<bool>();
        if(cond == condition) return;

        condition = cond;

        if(!cond)
        {
            pause_time = timestamp();
        }
        else{
            start_time += timestamp() - pause_time; // prevent resample probelm
        }

        LOG(INFO)<<"Record is "<<std::string(condition?"on":"off");
    }

    int64_t timestamp(){
        return clock.now().nanoseconds();
    }

    rclcpp::Clock            clock;
    sv::Svar                 node,config;
    std::vector<std::string> topics,types;
    std::vector<sv::Svar>    subs;
    std::vector<int>         counts;
    std::vector<double>      desire_freq;
    int64_t                  start_time, pause_time;
    GSLAM::Publisher         pub;
    bool                     condition = true;
};

/// 0 --- begin --- end --- capacity : size = end - begin
/// 0 --- end   --- begin --- capacity : size = capacity + (end - begin)
/// 0 --- begin == end --capacity: size = 0 or capacity, so this should not happen!
class RingBuffer{
public:
    RingBuffer(size_t capacity)
        : bytes(capacity), begin(0), end(0){}

    size_t size(){
        ssize_t size = end - begin;

        if(size < 0){
            size += bytes.size();
        }

        return size;
    }

    size_t left(){
        // should reserve one byte to prevent begin == end when full
        return bytes.size() - size() - 1;
    }

    /// return the droped package when full
    Package push_back(Package in){
        Package ret;

        size_t reserve_size = in.hd.payload_size + sizeof(ret.hd);
        if(left() < reserve_size)
            ret = pop_front();

        size_t  end1 = end + reserve_size;

        if(end1 < bytes.size())
        {
            memcpy(&bytes[end], (const char*)&in.hd,  sizeof(in.hd));
            end += sizeof(in.hd);
            memcpy(&bytes[end], in.payload.ptr(), in.hd.payload_size);
            end += in.hd.payload_size;
            return in;
        }

        // across
        std::vector<char> tmp_bytes(in.hd.payload_size,0);

        memcpy(tmp_bytes.data(), (const char*)&in.hd,  sizeof(in.hd));
        memcpy(tmp_bytes.data() + sizeof(in.hd), in.payload.ptr(), in.hd.payload_size);

        int second_pack = end1 % bytes.size();
        int first_pack = reserve_size - second_pack;

        memcpy(&bytes[end], tmp_bytes.data(),  first_pack);
        memcpy(bytes.data(), tmp_bytes.data() + sizeof(in.hd), in.hd.payload_size);
        end = second_pack;

        return ret;
    }

    /// no throw, should not call when it empty
    Package pop_front(){
        ssize_t size = end - begin;
        if(size < 0){
            size += bytes.size();
        }

        Package ret;
        if(size == 0)
            return ret;

        read(&ret.hd,sizeof(ret.hd));

        if(ret.hd.payload_size <= 0)
            return ret;

        if(ret.hd.payload_size > left())
            throw sv::SvarExeption("left not enough!");

        ret.payload = sv::SvarBuffer(ret.hd.payload_size);

        read(ret.payload.ptr(),ret.hd.payload_size);
        return ret;
    }

    size_t read(void* dst, int sz){
        if(left() < sz)
            throw sv::SvarExeption("left not enough, please check.");

        if(begin + sz < bytes.size()){
            memcpy(dst, &bytes[begin], sz);
        }
        else{
            std::vector<char> tmp(sz,0);
            int second_pack = (begin + sz) % bytes.size();
            int first_pack = sz - second_pack;
            memcpy(tmp.data(), &bytes[begin], first_pack);
            memcpy(tmp.data() + first_pack, bytes.data(), second_pack);
            memcpy(dst,tmp.data(),sz);
        }
        begin = (begin + sz) % bytes.size();
        return sz;
    }

    void write(void* src, int sz){

    }

    void dump(const Package& bag_header, std::string bag_file){
        std::ofstream ofs(bag_file,std::ios_base::binary);

        // header
        ofs.write((const char*)&bag_header.hd,sizeof(bag_header.hd));
        ofs.write(bag_header.payload.ptr(),bag_header.payload.size());

        // cache
        if(end >= begin){
            ofs.write(&bytes[begin], size());
        }
        else{
            ofs.write(&bytes[begin], size() - end);
            ofs.write(bytes.data(), end);
        }
    }

    void clear(){
        begin = end = 0;
    }

private:
    std::vector<char> bytes;
    size_t begin,end;
};


int test_ringbuffer(sv::Svar config){
    RingBuffer buffer(1000);

    for(int i=0;i<1000;i++)
    {
        Package pack;
        pack.payload = sv::SvarBuffer(&i,sizeof(i)).clone();
        pack.hd.payload_size = pack.payload.size();
        Package ret = buffer.push_back(pack);
        if(ret.hd.payload_size){
            LOG(INFO)<<*ret.payload.ptr<int>()<<" poped, size: "<<buffer.size()
                    <<",left:"<<buffer.left();
        }
    }
    return 0;
}

class Writer{
public:
    Writer(sv::Svar config){
        max_duration = config.get<int>("max_duration",std::numeric_limits<int>::max());
        max_bytes    = config.get<int>("max_size_mb",1024)*1024*1024;
        max_file_num = config.get<int>("max_file_num",10);
        folder       = config.get<std::string>("output",Writer::time_str());
        auto compress_m   = config.get<std::string>("compression","none");

        if(!compress_m.empty()){
            compress = svar[compress_m+"_compress"];
            if(compress.isFunction())
                LOG(INFO)<<"Found compression plugin "<<compress_m;
        }

        sub_bag_header = messenger.subscribe("bag_header", 0, &Writer::write_bag_header,this);
        sub_msgs = messenger.subscribe("msg_queue",0,&Writer::write_msg,this);
        std::string exception_name   = config.get<std::string>("exception","");

        if(!exception_name.empty()){
            ringbuf = std::make_shared<RingBuffer>(max_bytes);
            sub_exception = messenger.subscribe("exception",1,&Writer::cbk_exception,this);
        }
    }

    void prepare(){
        using namespace ghc::filesystem;
        using namespace std;

        if(!exists(folder))
            create_directory(folder);
        else{
            std::vector<std::string> files;
            for(auto fileit:directory_iterator(folder))
            {
                path path_ = fileit.path();
                if(path_.extension() != ".bagcvte") continue;

                files.push_back(path_.filename());
            }
            std::sort(files.begin(),files.end());
            file_names.insert(file_names.end(),files.begin(),files.end());
        }

        open_newbag();
    }

    void open_newbag(){
        if(file_names.size() >= max_file_num){
            ghc::filesystem::remove(folder+"/"+file_names.front());
            LOG(INFO) << "Remove old bag: "<< file_names.front();
            file_names.pop_front();
        }

        std::string new_bag = time_str() + ".bagcvte";
        if(ofs.is_open())
            ofs.close();
        ofs.open(folder + "/" + new_bag,std::ios_base::binary);

        if(bag_header.hd.payload_size){
            ofs.write((const char*)&bag_header.hd,sizeof(bag_header.hd));
            ofs.write(bag_header.payload.ptr(),bag_header.payload.size());
        }

        file_names.push_back(new_bag);

        LOG(INFO)<<"Created new bag: "<< new_bag;
        cur_bytes = 0;
    }

    void write_bag_header(const Package& pack){
        prepare();

        LOG(INFO)<<"package header: "<<std::string(pack.payload.ptr(),pack.payload.size());
        bag_header = pack;
        if(ringbuf) return;

        ofs.write((const char*)&pack.hd,sizeof(pack.hd));
        ofs.write(pack.payload.ptr(),pack.payload.size());
    }

    void write_msg(const Package& pack0){
        Package pack=pack0;
        if(compress.isFunction()){
            pack.payload = compress(pack.payload).as<sv::SvarBuffer>();
            pack.hd.payload_size = pack.payload.size();
            if(pack.hd.payload_size == 0)
                LOG(ERROR) <<"Payload is empty!";
        }

        if(ringbuf){
            std::unique_lock<std::mutex> lock(ringbuf_mutex);
            ringbuf->push_back(pack);
        }
        else{
            ofs.write((const char*)&pack.hd,sizeof(pack.hd));
            ofs.write(pack.payload.ptr(),pack.payload.size());
            cur_bytes += sizeof(pack.hd) + pack.hd.payload_size;

            if(cur_bytes >= max_bytes)
                open_newbag();
        }
    }

    int64_t timestamp(){
        return clock.now().nanoseconds();
    }

    void cbk_exception(const std::string& e){
        // we should dump all to a bagcvte file now
        std::string bagfile=folder+"/"+time_str()+e+".bagcvte";
        LOG(INFO)<< "Received exception:" << e
                << ", saving record to file " << bagfile;

        std::unique_lock<std::mutex> lock(ringbuf_mutex);
        ringbuf->dump(bag_header, bagfile);
        ringbuf->clear();
    }

    static std::string time_str(){
        time_t now = time(0);
        tm *ltm = localtime(&now);
        char str[256];

        sprintf(str,"%04d%02d%02d_%02d%02d%02d",1900 + ltm->tm_year,ltm->tm_mon + 1,ltm->tm_mday,
                ltm->tm_hour,ltm->tm_min,ltm->tm_sec);

        return str;
    }

    int64_t        max_duration, max_bytes, max_file_num, cur_bytes ;
    sv::Svar       compress, sub_bag_header, sub_msgs, sub_exception;
    Package        bag_header;
    std::shared_ptr<RingBuffer>     ringbuf;
    std::mutex                      ringbuf_mutex;
    std::ofstream  ofs;
    std::list<std::string> file_names;
    std::string            folder;
    rclcpp::Clock  clock;
};

int record_main(sv::Svar config){
    std::string topics_str  = config.arg<std::string>("topics","","topics to record, please use ',' to seprate");
    std::string resample    = config.arg<std::string>("resample","","resample control with format: topic:freq, eg: topic1:0.5,topic2:20");
    std::string output      = config.arg<std::string>("output",Writer::time_str(),"target folder to save everything");
    std::string compression = config.arg<std::string>("compression","none","compression options: none, zstd, lz4");
    std::string condition   = config.arg<std::string>("condition","/condition","condition topic name with type std_msgs/msg/Bool "
                                                                     "to control whether idle or not");
    std::string exception   = config.arg<std::string>("exception","","exception topic name with type std_msgs/msg/String to "
                                                                     "control important clip save from memory");
    int         sub_queue_size = config.arg("sub_queue_size",10,"default queue size for subscribers");
    int         write_queue_size = config.arg("write_queue_size",1000,"default write queue size for writer");
    int         max_duration = config.arg<int>("max_duration",std::numeric_limits<int>::max(),
                                               "max duration for a file part in seconds");
    int         max_size_mb  = config.arg<int>("max_size_mb",1024,"max size for a file to split in MB");
    int         max_file_num = config.arg<int>("max_file_num",10,"max bag file number for flash to prevent too much");
    bool        best_effort = config.arg<bool>("best_effort",true,"use best_effort policy or not");

    if(config.get("help",false)) return config.help();

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

    if(topics.empty())
        LOG(FATAL)<<"Please input topic names to record.";

    config["topics"] = topics;

    if(resample.size()){
        sv::Svar resamples=sv::Svar::parse_json("{"+resample+"}");
        LOG(INFO)<<"resamples:"<<resamples;
        config["resamples"] = resamples;
    }

    std::vector<std::string> args={"rosbag2"};
    svar["init"](args);

    Writer   writer(config);
    Recorder recorder(config);

    recorder.spin();

    return 0;
}


REGISTER_SVAR_MODULE(record){
    svar["apps"]["record"]={record_main,"record functional like ros2 bag record"};
    svar["apps"]["test_ringbuffer"]={test_ringbuffer,"test ring buffer"};
}
