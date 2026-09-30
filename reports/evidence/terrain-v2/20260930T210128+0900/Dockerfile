FROM mooncar-terrain:jazzy
RUN apt-get update && apt-get install -y --no-install-recommends python3-venv libmpich12 \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s /usr/lib/x86_64-linux-gnu/libmpich.so.12 /usr/local/lib/libmpi.so.12 \
    && ldconfig \
    && python3 -m venv --system-site-packages /opt/mooncar-dem \
    && /opt/mooncar-dem/bin/pip install --no-cache-dir lammps==2025.7.22.4.0
ENV PATH="/opt/mooncar-dem/bin:${PATH}" OMP_NUM_THREADS=1
WORKDIR /evidence
CMD ["python", "pile.py", "--help"]
