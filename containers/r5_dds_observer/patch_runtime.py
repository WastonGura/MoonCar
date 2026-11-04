#!/usr/bin/env python3
"""Apply version-pinned metadata hooks without replacing DDS decisions/returns."""
from pathlib import Path
import json, subprocess, hashlib
root=Path('/work/dds')
header=Path('/source/containers/r5_dds_observer/Trace.hh')
def edit(package,name,changes):
    p=root/package/name;s=p.read_text();s='#include "'+str(header)+'"\n'+s
    for old,new,count in changes:
        if s.count(old)!=count:raise RuntimeError(f'Hook mismatch: {p}: {old[:90]} ({s.count(old)})')
        s=s.replace(old,new)
    p.write_text(s)
edit('fastdds','src/cpp/rtps/transport/shared_mem/SharedMemManager.hpp',[
 ('bool invalidate_if_not_processing()', 'bool invalidate_if_not_processing(Status* observed_status=nullptr)',1),
 ('            EPROSIMA_LOG_WARNING(RTPS_TRANSPORT_SHM, "Buffer is being invalidated, segment_size may be insufficient");',
  '            if(observed_status)*observed_status=s;\n            EPROSIMA_LOG_WARNING(RTPS_TRANSPORT_SHM, "Buffer is being invalidated, segment_size may be insufficient");',1),
 ('throw std::runtime_error("allocation overflow");','auto e=mooncar_buffer("allocation_failure",segment_id_,0,0);e.a=size;e.b=free_bytes_;e.c=free_buffers_.size();mooncar_dds_event(e);\n                throw std::runtime_error("allocation overflow");',1),
 ('overflows_count_++;','auto e=mooncar_buffer("allocation_exception",segment_id_,0,0);e.a=size;e.b=free_bytes_;mooncar_dds_event(e);\n                overflows_count_++;',1),
 ('(*it)->invalidate_buffer();','auto pre=(*it)->status.load(std::memory_order_relaxed);\n                        auto e=mooncar_buffer("normal_release",segment_id_,segment_->get_offset_from_address(*it),pre.validity_id);e.a=pre.enqueued_count;e.b=pre.processing_count;e.c=free_bytes_;e.d=required_data_size;mooncar_dds_event(e);\n                        (*it)->invalidate_buffer();',1),
 ('if ((*it)->invalidate_if_not_processing())','BufferNode::Status pre;\n                    bool recovered=(*it)->invalidate_if_not_processing(&pre);\n                    auto e=mooncar_buffer("capacity_recovery",segment_id_,segment_->get_offset_from_address(*it),pre.validity_id);e.a=pre.enqueued_count;e.b=pre.processing_count;e.c=recovered;e.d=required_data_size;mooncar_dds_event(e);\n                    if (recovered)',2),
 ('// Descriptor points to non-existing segment: discard','auto e=mooncar_buffer("missing_segment",buffer_descriptor.source_segment_id,buffer_descriptor.buffer_node_offset,buffer_descriptor.validity_id);mooncar_dds_event(e);\n                        // Descriptor points to non-existing segment: discard',1),
 ('                    else\n                    {\n                        if (was_cell_freed)', '                    else\n                    {\n                        if (was_cell_freed)',1),
 ('                }\n            }\n            catch (const std::exception& e)\n            {\n                if (global_port_->is_port_ok())',
  '                    auto e=mooncar_buffer(is_buffer_valid?"buffer_valid":"buffer_invalid",buffer_descriptor.source_segment_id,buffer_descriptor.buffer_node_offset,buffer_descriptor.validity_id);e.a=global_port_->port_id();mooncar_dds_event(e);\n                }\n            }\n            catch (const std::exception& e)\n            {\n                if (global_port_->is_port_ok())',1),
])
edit('fastdds','src/cpp/rtps/transport/shared_mem/SharedMemTransport.cpp',[
 ('shared_mem_segment_ = shared_mem_manager_->create_segment(configuration_.segment_size(), max_allocations);',
  'shared_mem_segment_ = shared_mem_manager_->create_segment(configuration_.segment_size(), max_allocations);\n        auto ce=mooncar_buffer("segment_initialized",shared_mem_segment_->id(),0,0);ce.a=configuration_.segment_size();ce.b=max_allocations;ce.c=configuration_.port_queue_capacity();ce.d=shared_mem_segment_->mem_size();mooncar_dds_event(ce);',1),
 ('memcpy(shared_buffer->data(), send_buffer, send_buffer_size);',
  'memcpy(shared_buffer->data(), send_buffer, send_buffer_size);\n    auto sb=std::static_pointer_cast<SharedMemManager::SharedMemBuffer>(shared_buffer);auto id=sb->segment_id();auto uuid=id.get();\n    mooncar_dds_wire("fragment_tx",send_buffer,send_buffer_size,uuid.get(),sb->node_offset(),sb->validity_id());',1),
 ('if (!find_port(remote_locator.port)->try_push(buffer, is_port_ok))',
  'bool pushed=find_port(remote_locator.port)->try_push(buffer, is_port_ok);\n            auto sb=std::static_pointer_cast<SharedMemManager::SharedMemBuffer>(buffer);auto e=mooncar_buffer(pushed?"port_push":"port_discard",sb->segment_id(),sb->node_offset(),sb->validity_id());e.a=remote_locator.port;e.b=is_port_ok;mooncar_dds_event(e);\n            if (!pushed)',1),
])
edit('fastdds','src/cpp/rtps/reader/StatelessReader.cpp',[
 ('    GUID_t writer_guid = incomingChange->writerGUID;','    GUID_t writer_guid = incomingChange->writerGUID;\n    auto trace=[&](const char* stage,CacheChange_t* ch){if(sampleSize<500000)return;MooncarDdsEvent e;e.stage=stage;mooncar_guid(e.writer,writer_guid);mooncar_guid(e.reader,m_guid);e.sequence=ch->sequenceNumber.to64long();e.fragment=fragmentStartingNum;e.fragments=fragmentsInSubmessage;e.a=sampleSize;e.b=incomingChange->getFragmentSize();mooncar_dds_event(e);};\n    trace("fragment_rx",incomingChange);',1),
 ('if (work_change != nullptr && work_change->sequenceNumber > incomingChange->sequenceNumber)\n                {', 'if (work_change != nullptr && work_change->sequenceNumber > incomingChange->sequenceNumber)\n                {\n                    trace("fragment_older",incomingChange);',1),
 ('if (will_never_be_accepted)\n                    {','trace("history_reject",incomingChange);\n                    if (will_never_be_accepted)\n                    {',1),
 ('SequenceNumber_t updated_seq = work_change->sequenceNumber;', 'trace("reassembly_replaced",work_change);\n                        SequenceNumber_t updated_seq = work_change->sequenceNumber;',1),
 ('change_completed = work_change;','trace("reassembly_complete",work_change);\n                        change_completed = work_change;',1),
 ('else if (!change_received(change_completed))\n                    {','else if (!change_received(change_completed))\n                    {\n                        trace("history_add_reject",change_completed);',1),
])
edit('rmw_fastrtps_shared_cpp','src/participant.cpp',[
 ('domainParticipantQos.transport().user_transports.push_back(shm_transport);',
  'if(const char* size=std::getenv("MOONCAR_DDS_SHM_TX_BYTES")){\n          if(std::string(size)!="8388608"){RMW_SET_ERROR_MSG("Invalid test-only SHM size");return nullptr;}\n          shm_transport->segment_size(8388608);\n        }\n        MooncarDdsEvent e;e.stage="localhost_descriptor";e.a=shm_transport->segment_size();e.b=shm_transport->port_queue_capacity();e.c=shm_transport->max_message_size();mooncar_dds_event(e);\n        domainParticipantQos.transport().user_transports.push_back(shm_transport);',1),
])
edit('rmw_fastrtps_shared_cpp','src/rmw_publish.cpp',[
 ('  TRACETOOLS_TRACEPOINT(rmw_publish, publisher, ros_message, stamp.to_ns());',
  '  if(std::strstr(publisher->topic_name,"/rgb/image")||std::strstr(publisher->topic_name,"/depth/image")){MooncarDdsEvent e;e.stage="rmw_publish";e.fragments=std::strstr(publisher->topic_name,"/rgb/image")?1:2;mooncar_guid(e.writer,info->data_writer_->guid());int32_t sec;uint32_t ns;std::memcpy(&sec,ros_message,4);std::memcpy(&ns,static_cast<const char*>(ros_message)+4,4);e.stamp_ns=int64_t(sec)*1000000000LL+ns;const auto& q=info->data_writer_->get_qos();e.a=q.reliability().kind;e.b=q.history().kind;e.c=q.history().depth;e.d=q.durability().kind;mooncar_dds_event(e);}\n  TRACETOOLS_TRACEPOINT(rmw_publish, publisher, ros_message, stamp.to_ns());',2),
])
edit('rmw_fastrtps_shared_cpp','src/rmw_take.cpp',[
 ('  while (ReturnCode_t::RETCODE_OK == info->data_reader_->take(data_values, info_seq, 1)) {',
  '  auto observed_take=[&](){auto result=info->data_reader_->take(data_values,info_seq,1);if(std::strstr(subscription->topic_name,"/rgb/image")||std::strstr(subscription->topic_name,"/depth/image")){MooncarDdsEvent e;e.stage="rmw_take";e.fragments=std::strstr(subscription->topic_name,"/rgb/image")?1:2;mooncar_guid(e.reader,info->data_reader_->guid());e.a=result();const auto& q=info->data_reader_->get_qos();e.b=q.reliability().kind;e.c=q.history().kind;e.d=q.history().depth;if(result==ReturnCode_t::RETCODE_OK&&info_seq.length()){mooncar_guid(e.writer,info_seq[0].sample_identity.writer_guid());e.sequence=info_seq[0].sample_identity.sequence_number().to64long();e.fragment=info_seq[0].valid_data;}mooncar_dds_event(e);MooncarDdsEvent qe;qe.stage="reader_qos";qe.fragments=e.fragments;mooncar_guid(qe.reader,info->data_reader_->guid());qe.a=q.reliability().kind;qe.b=q.history().kind;qe.c=q.history().depth;qe.d=q.durability().kind;mooncar_dds_event(qe);}return result;};\n  while (ReturnCode_t::RETCODE_OK == observed_take()) {',3),
])
# Record exact source state and diff; original release trees stay untouched.
provenance={}
for package in ('fastdds','rmw_fastrtps_shared_cpp','rmw_fastrtps_cpp'):
    p=root/package
    provenance[package]=dict(commit=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip(),
        version=__import__('xml.etree.ElementTree',fromlist=['ElementTree']).parse(p/'package.xml').findtext('version'))
    diff=subprocess.check_output(['git','-C',str(p),'diff','--no-ext-diff'])
    (Path('/evidence')/(package+'-observer.patch')).write_bytes(diff)
    provenance[package]['patch_sha256']=hashlib.sha256(diff).hexdigest()
Path('/evidence/source-provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
