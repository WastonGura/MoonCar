"""Parse retained RTPS headers without retaining or decoding image pixels."""
import struct
from collections import Counter

def rtps_messages(payload):
    if payload[:4]!=b'RTPS' or len(payload)<20:return []
    prefix=payload[8:20].hex();offset=20;rows=[]
    while offset+4<=len(payload):
        kind,flags=payload[offset],payload[offset+1];endian='<' if flags&1 else '>'
        length=struct.unpack_from(endian+'H',payload,offset+2)[0]
        body=payload[offset+4:offset+4+length] if length else payload[offset+4:]
        if kind in (0x15,0x16) and len(body)>=20:
            high,low=struct.unpack_from(endian+'iI',body,12)
            rows.append(dict(writer_guid=prefix+body[8:12].hex(),sequence=(high<<32)+low,
                kind='DATA_FRAG' if kind==0x16 else 'DATA'))
        if not length:break
        offset+=4+length
    return rows

def pcap_headers(path,image_guids):
    counts=Counter();seen=set();errors=[];packets=0
    with open(path,'rb') as stream:
        header=stream.read(24)
        if len(header)!=24:return dict(errors=['Missing pcap header'])
        endian='<' if header[:4] in (b'\xd4\xc3\xb2\xa1',b'\x4d\x3c\xb2\xa1') else '>'
        link=struct.unpack(endian+'I',header[20:24])[0]
        while True:
            record=stream.read(16)
            if not record:break
            if len(record)!=16:errors.append('Truncated pcap record');break
            seconds,useconds,size,original=struct.unpack(endian+'IIII',record)
            data=stream.read(size);packets+=1
            if len(data)!=size:errors.append('Truncated retained packet');break
            offset={1:14,113:16,276:20}.get(link)
            if offset is None:errors.append('Unsupported pcap link type');break
            ip=data[offset:]
            if not ip or ip[0]>>4!=4 or len(ip)<28 or ip[9]!=17:continue
            udp=ip[(ip[0]&15)*4:]
            for row in rtps_messages(udp[8:]):
                guid=row['writer_guid'];is_image=guid in image_guids
                counts[('image_' if is_image else 'other_')+row['kind']]+=1
                if is_image:seen.add(guid)
    return dict(packets=packets,link_type=link,counts=dict(counts),image_writer_guids=sorted(seen),
        payload_udp_confirmed=bool(seen),errors=errors,
        interpretation='Only DATA/DATA_FRAG with an image publisher GUID proves image UDP traffic.')
