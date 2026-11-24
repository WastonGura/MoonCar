#include <iostream>
#include <rclcpp/rclcpp.hpp>
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("mooncar_startup_io_probe");
  std::cout<<"NODE_READY"<<std::endl;
  rclcpp::spin(node);
  node.reset();rclcpp::shutdown();
  std::cout<<"NODE_CLOSED"<<std::endl;
}
