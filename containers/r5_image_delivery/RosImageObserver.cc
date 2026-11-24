#include "BoundedLog.hh"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <iomanip>
#include <sstream>
int main(int argc,char **argv){
  if(argc!=2)return 2;
  const std::string folder=argv[1];
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("r5_cpp_image_observer",rclcpp::NodeOptions().use_intra_process_comms(false));
  BoundedLog log(folder+"/cpp-images.jsonl");
  auto callback=[&](const std::string &kind,const sensor_msgs::msg::Image::ConstSharedPtr &msg,const rclcpp::MessageInfo &info){
    const int64_t wall=deliveryWall();
    const auto &meta=info.get_rmw_message_info();
    std::ostringstream gid;gid<<std::hex<<std::setfill('0');
    for(auto byte:meta.publisher_gid.data)gid<<std::setw(2)<<static_cast<unsigned>(byte);
    auto sequence=[](uint64_t n){return n==RMW_MESSAGE_INFO_SEQUENCE_NUMBER_UNSUPPORTED ? std::string("null") : std::to_string(n);};
    std::ostringstream row;
    row<<"{\"kind\":\""<<kind<<"\",\"stamp_ns\":"<<static_cast<int64_t>(msg->header.stamp.sec)*1000000000LL+msg->header.stamp.nanosec
      <<",\"received_wall_ns\":"<<wall<<",\"dds_source_timestamp_ns\":"<<meta.source_timestamp
      <<",\"dds_received_timestamp_ns\":"<<meta.received_timestamp<<",\"publisher_gid\":\""<<gid.str()
      <<"\",\"publication_sequence\":"<<sequence(meta.publication_sequence_number)
      <<",\"reception_sequence\":"<<sequence(meta.reception_sequence_number)
      <<",\"width\":"<<msg->width<<",\"height\":"<<msg->height<<",\"callback_ns\":"<<deliveryWall()-wall<<"}";
    log.put(row.str());
  };
  auto rgb=node->create_subscription<sensor_msgs::msg::Image>("/mooncar/rgb/image",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::Image::ConstSharedPtr m,const rclcpp::MessageInfo &i){callback("rgb",m,i);});
  auto depth=node->create_subscription<sensor_msgs::msg::Image>("/mooncar/depth/image",rclcpp::SensorDataQoS(),[&](sensor_msgs::msg::Image::ConstSharedPtr m,const rclcpp::MessageInfo &i){callback("depth",m,i);});
  bool ok=true;
  try{rclcpp::spin(node);}catch(const std::exception &e){std::cerr<<e.what()<<std::endl;ok=false;}
  rgb.reset();depth.reset();node.reset();
  ok=log.finish() && ok;
  if(rclcpp::ok())rclcpp::shutdown();return ok?0:1;
}
