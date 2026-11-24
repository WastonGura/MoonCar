ARG BASE_IMAGE=mooncar-terrain:r5-image-dds-20261008
FROM ${BASE_IMAGE}
# Build context is the independent prepared runtime directory, not the repository.
COPY runtime/ /opt/mooncar/turn/r5-v2/
RUN test -f /opt/mooncar/turn/r5-v2/manifest.json && ln -s libode.so.8.0.2 /opt/mooncar/turn/r5-v2/libode.so.8
# No global LD_LIBRARY_PATH: only the checked simulator child loads this runtime.
ENV MOONCAR_TURN_RUNTIME=/opt/mooncar/turn/r5-v2
