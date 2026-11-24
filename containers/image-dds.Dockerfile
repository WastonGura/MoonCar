ARG BASE_IMAGE=mooncar-terrain:r5-dart
FROM ${BASE_IMAGE}
COPY containers/runtime/image-dds/ /opt/mooncar/image-dds/rmw-8.4.4/
ENV MOONCAR_IMAGE_DDS_RUNTIME=/opt/mooncar/image-dds/rmw-8.4.4
RUN python3 -c "import hashlib,json,pathlib; p=pathlib.Path('/opt/mooncar/image-dds/rmw-8.4.4'); m=json.loads((p/'manifest.json').read_text()); assert m['versions']['rmw_fastrtps_cpp']=='8.4.4'; assert m['diagnostic_instrumentation'] is False; assert all(hashlib.sha256((p/n).read_bytes()).hexdigest()==h for n,h in m['libraries'].items()); assert hashlib.sha256(pathlib.Path(m['stock_fastdds_path']).read_bytes()).hexdigest()==m['stock_fastdds_sha256']"
