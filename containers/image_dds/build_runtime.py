#!/usr/bin/env python3
"""Build the minimal RMW capacity patch from preserved 8.4.4 release sources."""
import argparse, hashlib, json, os, shutil, subprocess, time
from pathlib import Path

def sha(path): return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def patch(text):
    text = text.replace('#include <memory>', '#include <memory>\n#include <cstdlib>\n#include <fstream>\n#include <unistd.h>', 1)
    anchor = '        domainParticipantQos.transport().user_transports.push_back(shm_transport);'
    if text.count(anchor) != 1: raise RuntimeError('Release source anchor mismatch')
    text = text.replace(anchor, '''        if (const char * bytes = std::getenv("MOONCAR_IMAGE_DDS_TX_BYTES")) {
          if (std::string(bytes) != "8388608") {
            RMW_SET_ERROR_MSG("Unsupported MoonCar image segment capacity");
            return nullptr;
          }
          shm_transport->segment_size(8388608);
        }
''' + anchor, 1)
    anchor = '  cleanup_participant_info.cancel();'
    if text.count(anchor) != 1: raise RuntimeError('Receipt anchor mismatch')
    text = text.replace(anchor, '''  if (const char * bytes = std::getenv("MOONCAR_IMAGE_DDS_TX_BYTES")) {
    const auto & actual = participant_info->participant_->get_qos();
    std::shared_ptr<eprosima::fastdds::rtps::SharedMemTransportDescriptor> shm;
    for (const auto & descriptor : actual.transport().user_transports) {
      auto candidate = std::dynamic_pointer_cast<eprosima::fastdds::rtps::SharedMemTransportDescriptor>(descriptor);
      if (candidate) {shm = candidate; break;}
    }
    const char * receipt = std::getenv("MOONCAR_IMAGE_DDS_RECEIPT");
    if (std::string(bytes) != "8388608" || !shm || shm->segment_size() != 8388608 || !receipt) {
      RMW_SET_ERROR_MSG("MoonCar image DDS capacity was not applied");
      return nullptr;
    }
    std::ofstream out(receipt, std::ios::out | std::ios::trunc);
    out << "{\\\"pid\\\":" << getpid() << ",\\\"segment_bytes\\\":" << shm->segment_size()
        << ",\\\"port_queue_capacity\\\":" << shm->port_queue_capacity()
        << ",\\\"max_message_size\\\":" << shm->max_message_size()
        << ",\\\"user_transports\\\":" << actual.transport().user_transports.size()
        << ",\\\"builtin_transports\\\":" << (actual.transport().use_builtin_transports ? "true" : "false")
        << ",\\\"participant_created\\\":true}\\n";
    out.flush();
    if (!out.good()) {RMW_SET_ERROR_MSG("MoonCar DDS initialization receipt write failed"); return nullptr;}
  }
''' + anchor, 1)
    return text

def main():
    p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--prefix',default='/work/image-dds')
    p.add_argument('--archive',default='/source/reports/evidence/terrain-v2/r5-shm-capacity-20261008/release-source')
    a=p.parse_args();out=Path(a.output);out.mkdir(parents=True);prefix=Path(a.prefix);deadline=time.monotonic()+2700
    def run(argv):
        with (out/'build.log').open('a') as log:subprocess.run(argv,check=True,stdout=log,stderr=subprocess.STDOUT,timeout=max(.1,deadline-time.monotonic()))
    provenance={};stock=Path('/opt/ros/jazzy/lib/libfastrtps.so.2.14.6')
    if not stock.is_file():raise RuntimeError('Expected Fast DDS 2.14.6 absent')
    for label,package,commit in [('rmw-shared','rmw_fastrtps_shared_cpp','00b52c0cb5e7554797b24c98ac20532c7187c496'),('rmw-cpp','rmw_fastrtps_cpp','91f6433e1ff2292c238e42898be7542e9d2a2b2e')]:
        src=out/package;shutil.copytree(Path(a.archive)/label,src)
        found=subprocess.check_output(['git','-C',str(src),'rev-parse','HEAD'],text=True).strip()
        if found!=commit:raise RuntimeError('Release commit mismatch')
        if subprocess.check_output(['git','-C',str(src),'diff','--exit-code']):raise RuntimeError('Archived release modified')
        if label=='rmw-shared':
            file=src/'src/participant.cpp';file.write_text(patch(file.read_text()))
        diff=subprocess.check_output(['git','-C',str(src),'diff','--binary']);(out/(package+'.patch')).write_bytes(diff)
        provenance[package]=dict(commit=commit,patch_sha256=hashlib.sha256(diff).hexdigest(),source='https://github.com/ros2-gbp/rmw_fastrtps-release.git')
        build=out/(package+'-build')
        run(['cmake','-S',str(src),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(prefix),'-DCMAKE_PREFIX_PATH='+str(prefix)+';/opt/ros/jazzy','-DBUILD_TESTING=OFF'])
        run(['cmake','--build',str(build),'-j4']);run(['cmake','--install',str(build)])
    manifest=dict(format_version=1,versions={'fastdds':'2.14.6','rmw_fastrtps_shared_cpp':'8.4.4','rmw_fastrtps_cpp':'8.4.4'},
        libraries={n:sha(prefix/n) for n in ('lib/librmw_fastrtps_cpp.so','lib/librmw_fastrtps_shared_cpp.so')},
        stock_fastdds_path=str(stock),stock_fastdds_sha256=sha(stock),provenance=provenance,
        capacity_bytes=8388608,port_queue_capacity=512,build_seconds=2700-(deadline-time.monotonic()),
        builder_sha256=sha(__file__),diagnostic_instrumentation=False)
    (prefix/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest),flush=True)

if __name__=='__main__':main()
