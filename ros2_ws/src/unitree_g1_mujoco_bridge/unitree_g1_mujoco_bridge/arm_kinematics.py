"""Position/orientation IK from the robot URDF, without simulator object state."""
import math
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis,angle):
    axis = np.asarray(axis,dtype=float)
    axis /= max(1e-12,np.linalg.norm(axis))
    x,y,z = axis
    skew = np.array(((0.,-z,y),(z,0.,-x),(-y,x,0.)))
    return np.eye(3)+math.sin(angle)*skew+(1.-math.cos(angle))*(skew@skew)


def rpy_matrix(rpy):
    return rotation((0,0,1),rpy[2])@rotation((0,1,0),rpy[1])@rotation((1,0,0),rpy[0])


class ArmKinematics:
    def __init__(self,urdf,side='right',tool_offset=(0.,0.,0.)):
        self.tool_offset = np.asarray(tool_offset,dtype=float)
        root = ET.fromstring(urdf)
        by_child = {joint.find('child').get('link'):joint for joint in root.findall('joint')}
        chain,link = [],side+'_wrist_yaw_link'
        while link != 'pelvis':
            joint = by_child[link]
            origin = joint.find('origin')
            xyz = np.fromstring(origin.get('xyz','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            rpy = np.fromstring(origin.get('rpy','0 0 0'),sep=' ') if origin is not None else np.zeros(3)
            axis = joint.find('axis')
            axis = np.fromstring(axis.get('xyz','1 0 0'),sep=' ') if axis is not None else np.array((1.,0.,0.))
            limit = joint.find('limit')
            bounds = (-math.pi,math.pi) if limit is None else (float(limit.get('lower',-math.pi)),float(limit.get('upper',math.pi)))
            chain.append((joint.get('name'),joint.get('type'),xyz,rpy_matrix(rpy),axis,bounds))
            link = joint.find('parent').get('link')
        self.chain = list(reversed(chain))
        self.names = [name for name,kind,*_ in self.chain if name.startswith(side+'_') and kind != 'fixed']
        self.limits = np.array([bounds for name,kind,xyz,R,axis,bounds in self.chain if name in self.names])
        self.solve_lower = self.limits[:,0]+.02
        self.solve_upper = self.limits[:,1]-.02
        # Joint limits alone permit folding the upper arm through the torso.
        # Keep tabletop IK in an outward-shoulder workspace. Native replay of
        # 20261012T165825Z found 21 mm torso penetration on the inward branch.
        # This conservative workspace bound is not a full collision checker.
        roll = self.names.index(side+'_shoulder_roll_joint')
        if side == 'right':
            self.solve_upper[roll] = min(self.solve_upper[roll],-.10)
        else:
            self.solve_lower[roll] = max(self.solve_lower[roll],.10)

    def forward(self,joints):
        position,R = np.zeros(3),np.eye(3)
        axes,origins = [],[]
        for name,kind,xyz,offset,axis,bounds in self.chain:
            position = position+R@xyz
            R = R@offset
            if name in self.names:
                axes.append(R@axis)
                origins.append(position.copy())
            if kind in ('revolute','continuous'):
                R = R@rotation(axis,float(joints.get(name,0.)))
        position = position+R@self.tool_offset
        jacobian = np.column_stack([np.cross(axis,position-origin) for axis,origin in zip(axes,origins)])
        return position,R,jacobian,np.column_stack(axes)

    def solve(self,target,joints,orientation=None,iterations=120):
        seeds = [joints]
        # A hanging arm can drive local IK into the shoulder-roll limit.
        # Try bent-elbow seeds as well, retaining the measured waist joints.
        side = self.names[0].split('_')[0]
        for pitch in (-.8,-1.4):
            seed = dict(joints)
            seed.update({side+'_shoulder_pitch_joint':pitch,
                         side+'_shoulder_roll_joint':-.25 if side=='right' else .25,
                         side+'_shoulder_yaw_joint':0.,side+'_elbow_joint':1.4,
                         side+'_wrist_roll_joint':0.,side+'_wrist_pitch_joint':-.3,
                         side+'_wrist_yaw_joint':0.})
            seeds.append(seed)
        results = [self._solve(target,seed,orientation,iterations) for seed in seeds]
        def cost(result):
            solution,distance = result
            if orientation is None:
                return distance
            R = self.forward(dict(joints,**solution))[1]
            return distance+.12*np.linalg.norm(orientation-R)
        return min(results,key=cost)

    def _solve(self,target,joints,orientation=None,iterations=120):
        values = np.clip([joints.get(name,0.) for name in self.names],self.solve_lower,self.solve_upper)
        best = None
        for _ in range(iterations):
            state = dict(joints,**dict(zip(self.names,values)))
            position,R,J,A = self.forward(state)
            error = np.asarray(target)-position
            distance = float(np.linalg.norm(error))
            if orientation is not None:
                delta = orientation@R.T
                angular = .5*np.array((delta[2,1]-delta[1,2],delta[0,2]-delta[2,0],delta[1,0]-delta[0,1]))
                error = np.concatenate((error,.12*angular))
                J = np.vstack((J,.12*A))
            score = distance if orientation is None else distance+.12*np.linalg.norm(orientation-R)
            if best is None or score < best[0]:
                best = (score,values.copy(),distance)
            if np.linalg.norm(error) < .003:
                break
            step = J.T@np.linalg.solve(J@J.T+.0025*np.eye(len(error)),error)
            values = np.clip(values+np.clip(step,-.12,.12),self.solve_lower,self.solve_upper)
        return dict(zip(self.names,map(float,best[1]))),best[2]

    def cartesian_path(self,target,joints,orientation,max_step=.025):
        """Bound the swept tool path, not just the final joint-space endpoint.

        Retain the measured waist and continue the local IK branch. Joint-linear
        interpolation between distant endpoint solutions can bow through a
        tabletop object even when both endpoints are clear.
        """
        target = np.asarray(target,dtype=float)
        orientation = np.asarray(orientation,dtype=float)
        start,start_R,_,_ = self.forward(joints)
        if not np.all(np.isfinite(target)) or not np.all(np.isfinite(orientation)):
            raise ValueError('nonfinite Cartesian path target')
        count = max(1,int(np.ceil(np.linalg.norm(target-start)/max_step)),
                    int(np.ceil(np.linalg.norm(orientation-start_R)/.1)))
        if count>40:
            raise ValueError('Cartesian path exceeds bounded planning extent')
        state = dict(joints)
        path = []
        worst_error = 0.
        for index in range(1,count+1):
            fraction = index/count
            point = start+fraction*(target-start)
            u,s,v = np.linalg.svd((1-fraction)*start_R+fraction*orientation)
            if min(s)<1e-6:
                raise ValueError('ambiguous Cartesian orientation interpolation')
            R = u@np.diag([1.,1.,np.linalg.det(u@v)])@v
            solution,error = self._solve(point,state,R)
            if error>.004 or np.linalg.norm(self.forward(dict(state,**solution))[1]-R)>.08:
                solution,error = self.solve(point,state,R)
            if (error>.008 or np.linalg.norm(self.forward(dict(state,**solution))[1]-R)>.08 or
                    max(abs(solution[name]-state[name]) for name in self.names)>.4):
                raise ValueError('continuous Cartesian path unreachable')
            previous_point = start+(index-1)/count*(target-start)
            for alpha in (.25,.5,.75):
                intermediate = dict(state,**{name:(1-alpha)*state[name]+alpha*solution[name] for name in self.names})
                deviation = float(np.linalg.norm(self.forward(intermediate)[0]-((1-alpha)*previous_point+alpha*point)))
                if deviation>.008:
                    raise ValueError('Cartesian segment exceeds swept-path tolerance')
            worst_error = max(worst_error,error)
            path.append(solution)
            state.update(solution)
        return path,worst_error
