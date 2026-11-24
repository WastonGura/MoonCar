#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  for(int i=0;i<500;++i) {
    const auto path=std::string(argv[1])+"/receipt-"+std::to_string(i)+".json";
    std::ofstream out(path,std::ios::out|std::ios::trunc);
    if(!out)return 3;
    std::cout<<path<<std::endl;
    usleep(1000);
    out<<"{\"pid\":"<<getpid()<<",\"segment_bytes\":8388608,\"participant_created\":true}\n";
    out.flush();if(!out)return 4;
    usleep(1000);
    out.close();if(!out)return 5;
  }
}
