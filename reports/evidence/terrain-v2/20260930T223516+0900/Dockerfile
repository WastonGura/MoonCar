FROM mooncar-dem:lammps-2025.7.22.4
RUN /opt/mooncar-dem/bin/pip install --no-cache-dir mpi4py==4.1.1
ENV OMP_NUM_THREADS=1 MPI4PY_MPIABI=mpich MPI4PY_LIBMPI=/usr/local/lib/libmpi.so.12
