# Build the verified artifact with build_dart_contact.py first; this image keeps
# the exact tested binary and does not replace system physics plugins.
ARG BASE_IMAGE=mooncar-terrain:jazzy
FROM ${BASE_IMAGE}
COPY containers/runtime/dart-7.8.0/libmooncar-physics7-dartsim.so /opt/mooncar/physics/dart-7.8.0/
COPY containers/runtime/dart-7.8.0/manifest.json /opt/mooncar/physics/dart-7.8.0/
ENV MOONCAR_DART_PLUGIN_ROOT=/opt/mooncar/physics/dart-7.8.0
RUN python3 -c "import hashlib,json,pathlib; p=pathlib.Path('/opt/mooncar/physics/dart-7.8.0'); m=json.loads((p/'manifest.json').read_text()); assert m['gz_physics_version']=='7.8.0'; assert pathlib.Path('/opt/ros/jazzy/opt/gz_physics_vendor/lib/libgz-physics7.so.7.8.0').is_file(); assert hashlib.sha256((p/'libmooncar-physics7-dartsim.so').read_bytes()).hexdigest()==m['library_sha256']"
