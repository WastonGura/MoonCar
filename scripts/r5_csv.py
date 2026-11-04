"""Parse every numeric CSV row in bounded blocks, preserving all samples."""
import csv
from itertools import islice
from pathlib import Path
import numpy as np

def numeric_blocks(path,block_rows=4096):
    with Path(path).open() as stream:
        names=next(csv.reader([stream.readline()]))
        if not names or len(set(names))!=len(names):raise ValueError('Missing or duplicate CSV header')
        dtype=np.dtype([(n,float) for n in names])
        while True:
            lines=list(islice(stream,block_rows))
            if not lines:break
            # Parse all columns even when callers retain a subset. Incomplete
            # rows must fail rather than disappear during selected-column reads.
            matrix=np.loadtxt(lines,delimiter=',',ndmin=2)
            if matrix.shape[1]!=len(names):raise ValueError('CSV row/header width mismatch')
            yield np.ascontiguousarray(matrix).view(dtype).reshape(-1)

def read_numeric_csv(path,delimiter=',',names=True,usecols=None):
    if delimiter!=',' or names is not True:raise ValueError('Expected named numeric CSV')
    chunks=[]
    for block in numeric_blocks(path):
        if usecols is not None:
            selected=np.empty(len(block),dtype=[(k,float) for k in usecols])
            for k in usecols:selected[k]=block[k]
            block=selected
        chunks.append(block)
    if not chunks:raise ValueError('Empty CSV acquisition')
    return np.concatenate(chunks)

def aligned_observations(observation,physical):
    from itertools import zip_longest
    for a,b in zip_longest(numeric_blocks(observation),numeric_blocks(physical)):
        if a is None or b is None or len(a)!=len(b):return False
        if not np.array_equal(a['iteration'],b['iteration']) or not np.array_equal(a['t'],b['t']):return False
        if not all(np.isfinite(a[k]).all() for k in a.dtype.names if not k.endswith('_cmd')):return False
    return True
