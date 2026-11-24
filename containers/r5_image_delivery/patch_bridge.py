"""Instrument the fixed release without changing conversion/publication semantics."""
from pathlib import Path
import sys
root=Path(sys.argv[1]);path=root/'src/factory.hpp';original=path.read_text()
text=original.replace('#include "factory_interface.hpp"','#include "factory_interface.hpp"\n#include "BridgeTrace.hh"',1)
anchor='    ROS_T ros_msg;\n    convert_gz_to_ros(gz_msg, ros_msg);'
assert text.count(anchor)==1
text=text.replace(anchor,'''    const auto trace_received = deliveryWall();
    const std::string trace_stream = ros_pub->get_topic_name();
    int64_t trace_stamp = 0, trace_conversion = 0, trace_publish = 0;
    if constexpr (std::is_same_v<ROS_T, sensor_msgs::msg::Image>) {
      trace_stamp = static_cast<int64_t>(gz_msg.header().stamp().sec()) * 1000000000LL + gz_msg.header().stamp().nsec();
      bridgeImageTrace(trace_stream, trace_stamp, "entry", trace_received);
    }
    try {
    ROS_T ros_msg;
    convert_gz_to_ros(gz_msg, ros_msg);
    if constexpr (std::is_same_v<ROS_T, sensor_msgs::msg::Image>) {
      trace_conversion = deliveryWall() - trace_received;
      bridgeImageTrace(trace_stream, trace_stamp, "converted", trace_received, trace_conversion);
    }''',1)
anchor='    ros_pub->publish(ros_msg);';assert text.count(anchor)==1
text=text.replace(anchor,'''    if constexpr (std::is_same_v<ROS_T, sensor_msgs::msg::Image>) {
      trace_publish = deliveryWall();
      bridgeImageTrace(trace_stream, trace_stamp, "publish_begin", trace_received, trace_conversion);
    }
    ros_pub->publish(ros_msg);
    if constexpr (std::is_same_v<ROS_T, sensor_msgs::msg::Image>) {
      bridgeImageTrace(trace_stream, trace_stamp, "publish_return", trace_received, trace_conversion, deliveryWall()-trace_publish);
    }
    } catch (...) {
      if constexpr (std::is_same_v<ROS_T, sensor_msgs::msg::Image>) {
        bridgeImageTrace(trace_stream, trace_stamp, "exception", trace_received, trace_conversion, 0, true);
      }
      throw;
    }''',1)
path.write_text(text)
