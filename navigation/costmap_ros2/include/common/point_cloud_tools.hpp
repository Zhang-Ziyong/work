#ifndef POINT_CLOUD_TOOLS_HPP_
#define POINT_CLOUD_TOOLS_HPP_

#include <cstdint>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "costmap_utils.hpp"
/*!
 * \Enum to type mapping.
 */
template <int>
struct pointFieldTypeAsType {};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT8> {
  typedef int8_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT8> {
  typedef uint8_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT16> {
  typedef int16_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT16> {
  typedef uint16_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT32> {
  typedef int32_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT32> {
  typedef uint32_t type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::FLOAT32> {
  typedef float type;
};
template <>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::FLOAT64> {
  typedef double type;
};

/*!
 * \Type to enum mapping.
 */
template <typename T>
struct typeAsPointFieldType {};
template <>
struct typeAsPointFieldType<int8_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::INT8;
};
template <>
struct typeAsPointFieldType<uint8_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::UINT8;
};
template <>
struct typeAsPointFieldType<int16_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::INT16;
};
template <>
struct typeAsPointFieldType<uint16_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::UINT16;
};
template <>
struct typeAsPointFieldType<int32_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::INT32;
};
template <>
struct typeAsPointFieldType<uint32_t> {
  static const uint8_t value = sensor_msgs::msg::PointField::UINT32;
};
template <>
struct typeAsPointFieldType<float> {
  static const uint8_t value = sensor_msgs::msg::PointField::FLOAT32;
};
template <>
struct typeAsPointFieldType<double> {
  static const uint8_t value = sensor_msgs::msg::PointField::FLOAT64;
};

/*!
 * \Converts a value at the given pointer position, interpreted as the datatype
 *  specified by the given template argument point_field_type, to the given
 *  template type T and returns it.
 * \param data_ptr            pointer into the point cloud 2 buffer
 * \tparam point_field_type   sensor_msgs::PointField datatype value
 * \tparam T                  return type
 */
template <int point_field_type, typename T>
inline T readPointCloud2BufferValue(const unsigned char *data_ptr) {
  typedef typename pointFieldTypeAsType<point_field_type>::type type;
  return static_cast<T>(*(reinterpret_cast<type const *>(data_ptr)));
}

/*!
 * \Converts a value at the given pointer position interpreted as the datatype
 *  specified by the given datatype parameter to the given template type and
 * returns it.
 * \param data_ptr    pointer into the point cloud 2 buffer
 * \param datatype    sensor_msgs::PointField datatype value
 * \tparam T          return type
 */
template <typename T>
inline T readPointCloud2BufferValue(const unsigned char *data_ptr,
                                    const unsigned char datatype) {
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT8, T>(
          data_ptr);
    case sensor_msgs::msg::PointField::UINT8:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT8, T>(
          data_ptr);
    case sensor_msgs::msg::PointField::INT16:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT16, T>(
          data_ptr);
    case sensor_msgs::msg::PointField::UINT16:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT16,
                                        T>(data_ptr);
    case sensor_msgs::msg::PointField::INT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT32, T>(
          data_ptr);
    case sensor_msgs::msg::PointField::UINT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT32,
                                        T>(data_ptr);
    case sensor_msgs::msg::PointField::FLOAT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT32,
                                        T>(data_ptr);
    case sensor_msgs::msg::PointField::FLOAT64:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT64,
                                        T>(data_ptr);
  }
  // This should never be reached, but return statement added to avoid compiler
  // warning. (#84)
  return T();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get the index of a specified field (i.e., dimension/channel)
 * \param points the the point cloud message
 * \param field_name the string defining the field name
 */
static inline int getPointCloud2FieldIndex(
    const sensor_msgs::msg::PointCloud2 &cloud, const std::string &field_name) {
  // Get the index we need
  for (size_t d = 0; d < cloud.fields.size(); ++d) {
    if (cloud.fields[d].name == field_name) {
      return static_cast<int>(d);
    }
  }
  return -1;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Convert a sensor_msgs::msg::PointCloud message to a
 * sensor_msgs::msg::PointCloud2 message.
 * \param input the message in the sensor_msgs::msg::PointCloud format
 * \param output the resultant message in the sensor_msgs::msg::PointCloud2
 * format
 */
static inline bool convertPointCloudToPointCloud2(
    const sensor_msgs::msg::PointCloud &input,
    sensor_msgs::msg::PointCloud2 &output) {
  output.header = input.header;
  output.width = static_cast<uint32_t>(input.points.size());
  output.height = 1;
  output.fields.resize(3 + input.channels.size());
  // Convert x/y/z to fields
  output.fields[0].name = "x";
  output.fields[1].name = "y";
  output.fields[2].name = "z";
  int offset = 0;
  // All offsets are *4, as all field data types are float32
  for (size_t d = 0; d < output.fields.size(); ++d, offset += 4) {
    output.fields[d].offset = offset;
    output.fields[d].datatype = sensor_msgs::msg::PointField::FLOAT32;
    output.fields[d].count = 1;
  }
  output.point_step = offset;
  output.row_step = output.point_step * output.width;
  // Convert the remaining of the channels to fields
  for (size_t d = 0; d < input.channels.size(); ++d) {
    output.fields[3 + d].name = input.channels[d].name;
  }
  output.data.resize(input.points.size() * output.point_step);
  output.is_bigendian = false;  // @todo ?
  output.is_dense = false;

  // Copy the data points
  for (size_t cp = 0; cp < input.points.size(); ++cp) {
    memcpy(&output.data[cp * output.point_step + output.fields[0].offset],
           &input.points[cp].x, sizeof(float));
    memcpy(&output.data[cp * output.point_step + output.fields[1].offset],
           &input.points[cp].y, sizeof(float));
    memcpy(&output.data[cp * output.point_step + output.fields[2].offset],
           &input.points[cp].z, sizeof(float));
    for (size_t d = 0; d < input.channels.size(); ++d) {
      if (input.channels[d].values.size() == input.points.size()) {
        memcpy(
            &output.data[cp * output.point_step + output.fields[3 + d].offset],
            &input.channels[d].values[cp], sizeof(float));
      }
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Convert a sensor_msgs::msg::PointCloud2 message to a
 * sensor_msgs::msg::PointCloud message.
 * \param input the message in the sensor_msgs::msg::PointCloud2 format
 * \param output the resultant message in the sensor_msgs::msg::PointCloud
 * format
 */
static inline bool convertPointCloud2ToPointCloud(
    const sensor_msgs::msg::PointCloud2 &input,
    sensor_msgs::msg::PointCloud &output) {
  output.header = input.header;
  output.points.resize(input.width * input.height);
  output.channels.resize(input.fields.size() - 3);
  // Get the x/y/z field offsets
  int x_idx = getPointCloud2FieldIndex(input, "x");
  int y_idx = getPointCloud2FieldIndex(input, "y");
  int z_idx = getPointCloud2FieldIndex(input, "z");
  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    return false;
  }
  int x_offset = input.fields[x_idx].offset;
  int y_offset = input.fields[y_idx].offset;
  int z_offset = input.fields[z_idx].offset;
  uint8_t x_datatype = input.fields[x_idx].datatype;
  uint8_t y_datatype = input.fields[y_idx].datatype;
  uint8_t z_datatype = input.fields[z_idx].datatype;

  // Convert the fields to channels
  int cur_c = 0;
  for (size_t d = 0; d < input.fields.size(); ++d) {
    if (static_cast<int>(input.fields[d].offset) == x_offset ||
        static_cast<int>(input.fields[d].offset) == y_offset ||
        static_cast<int>(input.fields[d].offset) == z_offset) {
      continue;
    }
    output.channels[cur_c].name = input.fields[d].name;
    output.channels[cur_c].values.resize(output.points.size());
    cur_c++;
  }

  // Copy the data points
  for (size_t cp = 0; cp < output.points.size(); ++cp) {
    // Copy x/y/z
    output.points[cp].x = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + x_offset], x_datatype);
    output.points[cp].y = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + y_offset], y_datatype);
    output.points[cp].z = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + z_offset], z_datatype);
    // Copy the rest of the data
    int cur_c = 0;
    for (size_t d = 0; d < input.fields.size(); ++d) {
      if (static_cast<int>(input.fields[d].offset) == x_offset ||
          static_cast<int>(input.fields[d].offset) == y_offset ||
          static_cast<int>(input.fields[d].offset) == z_offset) {
        continue;
      }
      output.channels[cur_c++].values[cp] = readPointCloud2BufferValue<float>(
          &input.data[cp * input.point_step + input.fields[d].offset],
          input.fields[d].datatype);
    }
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Convert a sensor_msgs::msg::PointCloud2 message to a
 * sensor_msgs::msg::PointCloud message.
 * \param input the message in the sensor_msgs::msg::PointCloud2 format
 * \param output the resultant message in the sensor_msgs::msg::PointCloud
 * format
 */
static inline bool convertPointCloud2ToCostmapCloud(
    const sensor_msgs::msg::PointCloud2 &input,
    CVTE_BABOT::CostmapPointCloud &output) {
  output.resize(input.width * input.height);
  // Get the x/y/z field offsets
  int x_idx = getPointCloud2FieldIndex(input, "x");
  int y_idx = getPointCloud2FieldIndex(input, "y");
  int z_idx = getPointCloud2FieldIndex(input, "z");
  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    return false;
  }
  int x_offset = input.fields[x_idx].offset;
  int y_offset = input.fields[y_idx].offset;
  int z_offset = input.fields[z_idx].offset;
  uint8_t x_datatype = input.fields[x_idx].datatype;
  uint8_t y_datatype = input.fields[y_idx].datatype;
  uint8_t z_datatype = input.fields[z_idx].datatype;

  // Copy the data points
  for (size_t cp = 0; cp < output.size(); ++cp) {
    // Copy x/y/z
    output[cp].d_x = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + x_offset], x_datatype);
    output[cp].d_y = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + y_offset], y_datatype);
    output[cp].d_z = readPointCloud2BufferValue<float>(
        &input.data[cp * input.point_step + z_offset], z_datatype);
  }
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Convert a sensor_msgs::msg::PointCloud2 message to a
 * sensor_msgs::msg::PointCloud message.
 * \param input the message in the sensor_msgs::msg::PointCloud2 format
 * \param output the resultant message in the sensor_msgs::msg::PointCloud
 * format
 */
static inline bool convertLaserScanToCostmapCloud(
    const sensor_msgs::msg::LaserScan &input,
    CVTE_BABOT::CostmapPointCloud &output) {
  if (input.ranges.empty()) {
    return false;
  }

  static const float epsilon = 0.0001;

  output.clear();
  output.reserve(input.ranges.size());

  double range = 0.0;
  double point_x = 0.0, point_y = 0.0;

  for (unsigned int i = 0; i < input.ranges.size(); i++) {
    range = input.ranges[i];

    // 小距离值一般是由于结构阻挡导致，此点不应该作为障碍物，并且也不该用此点来清除障碍物
    if (range < 0.01) {
      continue;
    }

    // 当距离是无效值时，很有可能是激光打到无穷远处无返回值，此时用此点来清除障碍物是合理的
    if (std::isnan(range) || (!std::isfinite(range) && range > 0)) {
      range = input.range_max - epsilon;
    }
    
    // point in laser
    float angle = input.angle_min + i * input.angle_increment;

    point_x = range * cos(angle);  //- 0.25;
    point_y = range * sin(angle);

    output.push_back({point_x, point_y, 0});
  }
  return true;
}
#endif  // POINT_CLOUD_TOOLS_HPP_