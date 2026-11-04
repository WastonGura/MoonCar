#pragma once
#include "BoundedLog.hh"
#include <cstdlib>
#include <sstream>
#include <sensor_msgs/msg/image.hpp>
inline void bridgeImageTrace(const std::string &stream,int64_t stamp,
  const std::string &stage,int64_t received,int64_t conversion=0,int64_t publication=0,bool exception=false){
  if(stream!="/mooncar/rgb/image" && stream!="/mooncar/depth/image")return;
  const char *path=std::getenv("MOONCAR_IMAGE_BRIDGE_TRACE");
  if(!path)throw std::runtime_error("Missing bridge trace output");
  static BoundedLog log(path);
  std::ostringstream row;
  row<<"{\"stream\":\""<<stream<<"\",\"stamp_ns\":"<<stamp<<",\"stage\":\""<<stage
    <<"\",\"received_wall_ns\":"<<received<<",\"wall_ns\":"<<deliveryWall()
    <<",\"conversion_ns\":"<<conversion<<",\"publication_ns\":"<<publication
    <<",\"exception\":"<<(exception?"true":"false")<<"}";
  log.put(row.str());
}
