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
        order=np.argsort(inverse,kind='stable')
        sorted_heights=points[order,2]
        starts=np.r_[0,np.cumsum(np.bincount(inverse,minlength=len(unique)))]
        updates=[]
        for i,key in enumerate(unique):
            heights=sorted_heights[starts[i]:starts[i+1]]
            updates.append((tuple(map(int,key)),float(np.median(heights)),float(np.var(heights)),
                            float(heights.min()),float(heights.max())))
        expired={k for k,v in self.cells.items() if stamp-v['stamp']>self.max_age}
        remaining=self.cells.keys()-expired
        if len(remaining|{item[0] for item in updates})>self.max_cells:
            raise ValueError('Elevation map capacity reached')
        for key in expired:
            del self.cells[key]
        for key,height,variance,low,high in updates:
            old=self.cells.get(key)
            if old is None:
                self.cells[key]=dict(height=height,m2=0.,count=1,within_variance=variance,stamp=stamp,
                                     latest_min=low,latest_max=high)
            else:
                count=old['count']+1
                delta=height-old['height']
                mean=old['height']+delta/count
                self.cells[key]=dict(height=mean,m2=old['m2']+delta*(height-mean),count=count,
                                     within_variance=old['within_variance']+(variance-old['within_variance'])/count,
                                     stamp=stamp,latest_min=low,latest_max=high)
        self.last_stamp=stamp
        outputs=sorted({(int(k[0])//self.ratio,int(k[1])//self.ratio) for k in unique})
        if not outputs:
            return []
        fine_keys=np.asarray(sorted(self.cells),dtype=np.int64)
        entries=[self.cells[tuple(k)] for k in fine_keys]
        positions=np.column_stack(((fine_keys+.5)*self.fine_resolution,
                                   [row['height'] for row in entries]))
        variances=np.array([row['within_variance']+row['m2']/max(1,row['count']-1) for row in entries])
        extents=np.array([[row['latest_min'],row['latest_max']] for row in entries])
        result=[]
        for key in outputs:
            cell=self._classify(key,fine_keys,positions,variances,extents,
                                slope_deg,roughness,step_height,drop_height)
            if cell is not None:
                result.append(cell if return_elevation else cell[:5])
        return result

    def _classify(self, key, fine_keys, positions, variances, extents,
                  slope_max, roughness_max, step_height, drop_height):
        center=(np.asarray(key,dtype=float)+.5)*self.resolution
        fine=self.fine_resolution
        local=np.sum((positions[:,:2]-center)**2,axis=1)<=self.radius**2
        own=np.all(fine_keys//self.ratio==key,axis=1)
        if np.count_nonzero(local)<6 or not np.any(own):
            return None
        xyz=positions[local]
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
        variance=variances[local]
        residual=float(np.sqrt(np.mean((centered@normal)**2+variance*normal[2]**2)))
        reason=(1 if slope>=slope_max else 0)|(2 if residual>=roughness_max else 0)
        gradient=-normal[:2]/normal[2] if normal[2]>1e-6 else np.zeros(2)
        if normal[2]>1e-6:
            plane_height=xyz[:,2].mean()+(positions[own,:2]-xyz[:,:2].mean(axis=0))@gradient
            relative_height=extents[own]-plane_height[:,None]
            reason|=4 if np.any(relative_height[:,1]>step_height) else 0
            reason|=8 if np.any(relative_height[:,0]<-drop_height) else 0
        for (ix,iy),height in zip(fine_keys[own],positions[own,2]):
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                neighbor=self.cells.get((ix+dx,iy+dy))
                if neighbor is None:
                    continue
                jump=height-neighbor['height']+fine*(gradient[0]*dx+gradient[1]*dy)
                reason|=4 if jump>step_height else 0
                reason|=8 if jump<-drop_height else 0
        height=float(np.mean(positions[own,2]))
        return float(center[0]),float(center[1]),slope,residual,reason,height
