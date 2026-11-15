"""Literature-guided elevation/neighborhood prototype for controlled comparison.

Method references and project adaptations are fixed in
docs/terrain_strategy_comparison_20261115.md. Empirical height variance is not a
calibrated sensor or pose covariance. This module has no ROS or world inputs.
"""
import math

import numpy as np


class ElevationNeighborhood:
    def __init__(self, resolution=.20, fine_resolution=.05, radius=.36,
                 max_age=5., max_cells=200000):
        values=(resolution,fine_resolution,radius,max_age)
        if not all(math.isfinite(v) and v>0 for v in values):
            raise ValueError('Map dimensions and age must be positive and finite')
        ratio=resolution/fine_resolution
        if abs(ratio-round(ratio))>1e-6 or ratio<1 or radius<resolution:
            raise ValueError('Output cells must contain whole fine cells and fit the neighborhood')
        if type(max_cells) is not int or max_cells<1:
            raise ValueError('Map capacity must be a positive integer')
        self.resolution=resolution
        self.fine_resolution=fine_resolution
        self.radius=radius
        self.max_age=max_age
        self.max_cells=max_cells
        self.ratio=int(round(ratio))
        self.cells={}
        self.last_stamp=None

    def update(self, points, stamp, slope_deg=18., roughness=.04,
               step_height=.18, drop_height=.12, return_elevation=False):
        if not math.isfinite(stamp) or stamp<0 or self.last_stamp is not None and stamp<=self.last_stamp:
            raise ValueError('Observation timestamps must increase')
        points=np.asarray(points,dtype=float).reshape(-1,3)
        points=points[np.isfinite(points).all(axis=1)]
        if np.any(abs(points)>10000):
            raise ValueError('Observation coordinates exceed the map range')
        keys=np.floor(points[:,:2]/self.fine_resolution).astype(np.int64)
        unique,inverse=np.unique(keys,axis=0,return_inverse=True)
        updates=[]
        for i,key in enumerate(unique):
            heights=points[inverse==i,2]
            updates.append((tuple(map(int,key)),float(np.median(heights)),float(np.var(heights))))
        expired={k for k,v in self.cells.items() if stamp-v['stamp']>self.max_age}
        remaining=self.cells.keys()-expired
        if len(remaining|{key for key,_,_ in updates})>self.max_cells:
            raise ValueError('Elevation map capacity reached')
        for key in expired:
            del self.cells[key]
        for key,height,variance in updates:
            old=self.cells.get(key)
            if old is None:
                self.cells[key]=dict(height=height,m2=0.,count=1,within_variance=variance,stamp=stamp)
            else:
                count=old['count']+1
                delta=height-old['height']
                mean=old['height']+delta/count
                self.cells[key]=dict(height=mean,m2=old['m2']+delta*(height-mean),count=count,
                                     within_variance=old['within_variance']+(variance-old['within_variance'])/count,
                                     stamp=stamp)
        self.last_stamp=stamp
        outputs=sorted({(int(k[0])//self.ratio,int(k[1])//self.ratio) for k in unique})
        result=[]
        for key in outputs:
            cell=self.classify(key,slope_deg,roughness,step_height,drop_height)
            if cell is not None:
                result.append(cell if return_elevation else cell[:5])
        return result

    def classify(self, key, slope_max, roughness_max, step_height, drop_height):
        center=(np.asarray(key,dtype=float)+.5)*self.resolution
        fine=self.fine_resolution
        low=np.floor((center-self.radius)/fine).astype(int)
        high=np.floor((center+self.radius)/fine).astype(int)
        positions=[];variances=[];own=[]
        for ix in range(low[0],high[0]+1):
            for iy in range(low[1],high[1]+1):
                row=self.cells.get((ix,iy))
                if row is None:
                    continue
                position=(np.array([ix,iy])+.5)*fine
                if np.linalg.norm(position-center)>self.radius:
                    continue
                positions.append([position[0],position[1],row['height']])
                variances.append(row['within_variance']+row['m2']/max(1,row['count']-1))
                if (ix//self.ratio,iy//self.ratio)==key:
                    own.append((ix,iy,row))
        if len(positions)<6 or not own:
            return None
        xyz=np.asarray(positions)
        xy=xyz[:,:2].copy()
        xy-=xy.mean(axis=0)
        spread=np.linalg.eigvalsh(xy.T@xy/len(xy))
        if spread[0]<fine*fine:
            return None
        centered=xyz-xyz.mean(axis=0)
        _,vectors=np.linalg.eigh(centered.T@centered/len(centered))
        normal=vectors[:,0]
        normal*=1 if normal[2]>=0 else -1
        slope=math.degrees(math.acos(float(np.clip(normal[2],0.,1.))))
        variance=np.asarray(variances)
        residual=float(np.sqrt(np.mean((centered@normal)**2+variance*normal[2]**2)))
        reason=(1 if slope>=slope_max else 0)|(2 if residual>=roughness_max else 0)
        gradient=-normal[:2]/normal[2] if normal[2]>1e-6 else np.zeros(2)
        for ix,iy,row in own:
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                neighbor=self.cells.get((ix+dx,iy+dy))
                if neighbor is None:
                    continue
                jump=row['height']-neighbor['height']+fine*(gradient[0]*dx+gradient[1]*dy)
                reason|=4 if jump>step_height else 0
                reason|=8 if jump<-drop_height else 0
        height=float(np.mean([p[2]['height'] for p in own]))
        return float(center[0]),float(center[1]),slope,residual,reason,height
