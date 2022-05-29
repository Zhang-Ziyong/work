#include "common/proto_stream.hpp"
#include "glog/logging.h"

namespace slam2d_core {
namespace common {

namespace {

// First eight bytes to identify our proto stream format.
const uint64_t kMagic = 0x7b1d1f7b5bf501db;

void writeSizeAsLittleEndian(uint64_t size, std::ostream *out) {
  for (int i = 0; i != 8; ++i) {
    out->put(size & 0xff);
    size >>= 8;
  }
}

bool readSizeAsLittleEndian(std::istream *in, uint64_t *size) {
  *size = 0;
  for (int i = 0; i != 8; ++i) {
    *size >>= 8;
    *size += static_cast<uint64_t>(in->get()) << 56;
  }
  return !in->fail();
}

}  // namespace

ProtoStreamWriter::ProtoStreamWriter(const std::string &filename)
    : out_(filename, std::ios::out | std::ios::binary) {
  writeSizeAsLittleEndian(kMagic, &out_);
}

void ProtoStreamWriter::write(const std::string &uncompressed_data) {
  std::string compressed_data;
  fastGzipString(uncompressed_data, &compressed_data);
  writeSizeAsLittleEndian(compressed_data.size(), &out_);
  out_.write(compressed_data.data(), compressed_data.size());
}

void ProtoStreamWriter::writeProto(const google::protobuf::Message &proto) {
  std::string uncompressed_data;
  proto.SerializeToString(&uncompressed_data);
  write(uncompressed_data);
}

bool ProtoStreamWriter::close() {
  out_.close();
  return !out_.fail();
}

ProtoStreamReader::ProtoStreamReader(const std::string &filename)
    : in_(filename, std::ios::in | std::ios::binary) {
  uint64_t magic;
  if (!readSizeAsLittleEndian(&in_, &magic) || magic != kMagic) {
    in_.setstate(std::ios::failbit);
  }
  CHECK(in_.good()) << "Failed to open proto stream '" << filename << "'.";
}

bool ProtoStreamReader::read(std::string *decompressed_data) {
  uint64_t compressed_size;
  if (!readSizeAsLittleEndian(&in_, &compressed_size)) {
    return false;
  }
  std::string compressed_data(compressed_size, '\0');
  if (!in_.read(&compressed_data.front(), compressed_size)) {
    return false;
  }
  fastGunzipString(compressed_data, decompressed_data);
  return true;
}

bool ProtoStreamReader::readProto(google::protobuf::Message *proto) {
  std::string decompressed_data;
  return read(&decompressed_data) && proto->ParseFromString(decompressed_data);
}

bool ProtoStreamReader::eof() const {
  return in_.eof();
}

}  // namespace common
}  // namespace slam2d_core