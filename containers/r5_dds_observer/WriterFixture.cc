#include "Trace.hh"
#include <dlfcn.h>
#include <cstdlib>
#include <chrono>
#include <thread>
int main(int argc,char** argv){
    if(argc!=3)return 2;
    void* library=dlopen(argv[1],RTLD_NOW);if(!library)return 3;
    auto emit=reinterpret_cast<void(*)(MooncarDdsEvent)>(dlsym(library,"mooncar_dds_event"));
    auto flush=reinterpret_cast<void(*)()>(dlsym(library,"mooncar_dds_flush"));if(!emit||!flush)return 4;
    for(int i=0;i<std::atoi(argv[2]);++i){MooncarDdsEvent e;e.stage="fixture";e.sequence=i;emit(e);}
    flush();dlclose(library);return 0;
}
