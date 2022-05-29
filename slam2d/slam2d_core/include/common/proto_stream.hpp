#ifndef _PROTO_STREAM_H_
#define _PROTO_STREAM_H_

#include <fstream>
#include <string>
#include <boost/iostreams/device/back_inserter.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include "google/protobuf/message.h"
namespace slam2d_core {
namespace common {

inline void fastGzipString(const std::string &uncompressed,
                           std::string *compressed) {
  boost::iostreams::filtering_ostream out;
  out.push(
      boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
  out.push(boost::iostreams::back_inserter(*compressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char *>(uncompressed.data()),
                          uncompressed.size());
}

inline void fastGunzipString(const std::string &compressed,
                             std::string *decompressed) {
  boost::iostreams::filtering_ostream out;
  out.push(boost::iostreams::gzip_decompressor());
  out.push(boost::iostreams::back_inserter(*decompressed));
  boost::iostreams::write(out,
                          reinterpret_cast<const char *>(compressed.data()),
                          compressed.size());
}

class ProtoStreamWriter {
 public:
  explicit ProtoStreamWriter(const std::string &filename);
  ~ProtoStreamWriter() = default;

  ProtoStreamWriter(const ProtoStreamWriter &) = delete;
  ProtoStreamWriter &operator=(const ProtoStreamWriter &) = delete;

  void writeProto(const google::protobuf::Message &proto);
  bool close();

 private:
  void write(const std::string &uncompressed_data);

  std::ofstream out_;
};

class ProtoStreamReader {
 public:
  explicit ProtoStreamReader(const std::string &filename);
  ~ProtoStreamReader() = default;

  ProtoStreamReader(const ProtoStreamReader &) = delete;
  ProtoStreamReader &operator=(const ProtoStreamReader &) = delete;

  bool readProto(google::protobuf::Message *proto);
  bool eof() const;

 private:
  bool read(std::string *decompressed_data);

  std::ifstream in_;
};

}  // namespace common
}  // namespace slam2d_core

#endif