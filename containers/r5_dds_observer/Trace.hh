#pragma once
#include <cstdint>
#include <cstring>
// Fixed-size metadata only. Formatting and I/O are confined to the writer thread.
struct MooncarDdsEvent {
    const char* stage=nullptr;
    uint8_t segment[8]{},writer[16]{},reader[16]{};
    uint64_t offset=0,validity=0,a=0,b=0,c=0,d=0;
    int64_t sequence=-1,stamp_ns=-1;
    uint32_t fragment=0,fragments=0;
};
extern "C" void mooncar_dds_event(MooncarDdsEvent) noexcept;
extern "C" void mooncar_dds_wire(const char*,const void*,uint32_t,const uint8_t*,uint64_t,uint32_t) noexcept;
extern "C" void mooncar_dds_flush() noexcept;
template<class G> inline void mooncar_guid(uint8_t* out,const G& g) {
    std::memcpy(out,g.guidPrefix.value,12);std::memcpy(out+12,g.entityId.value,4);
}
template<class I> inline MooncarDdsEvent mooncar_buffer(const char* stage,I id,uint64_t offset,uint64_t validity) {
    auto uuid=id.get();MooncarDdsEvent e;e.stage=stage;std::memcpy(e.segment,uuid.get(),8);e.offset=offset;e.validity=validity;return e;
}
template<class C> inline void mooncar_change(const char* stage,const C* change,uint64_t a=0,uint64_t b=0) {
    if(change->serializedPayload.length<500000 && change->getFragmentSize()==0)return;
    MooncarDdsEvent e;e.stage=stage;mooncar_guid(e.writer,change->writerGUID);
    e.sequence=change->sequenceNumber.to64long();e.a=a;e.b=b;mooncar_dds_event(e);
}
