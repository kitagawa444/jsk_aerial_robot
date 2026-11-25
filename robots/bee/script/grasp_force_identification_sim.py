#!/usr/bin/env python3
"""Simulation-only reference relay for bounded force response experiments.

The passive extractor/fitter is in identify_grasp_force_response.py. This node
deliberately refuses hardware, owns the final reference topic, and never arms.
The normal mission performs contact acquisition; excitation begins only in Hold.
"""
import copy
import json
import math
import os
import time
from urllib.parse import urlparse

from grasp_reference_axes import contact_normal
import numpy as np
import rospy
from bee.msg import ControllerStatus, BeeCommand
from std_msgs.msg import String, Bool


def vector(v):
    return np.array([v.x,v.y,v.z],dtype=float)


def set_vector(dst,value):
    dst.x,dst.y,dst.z=map(float,value)


def basis(normal):
    normal=normal/np.linalg.norm(normal)
    up=np.array([0.,0.,1.])-normal[2]*normal
    if np.linalg.norm(up)<1e-6:
        raise ValueError('Identification requires a nonvertical normal')
    up/=np.linalg.norm(up)
    return np.array([-normal,up,np.cross(up,normal)])


def margins(reference,observed,config):
    """Conservative force checks in the same axes as the central allocator."""
    n=contact_normal(reference);b=basis(n)
    force=vector(reference.force)
    normal=-np.dot(n,force)
    tangent=abs(np.dot(b[1],force))+abs(np.dot(b[2],force))
    error=config['allocation_tangent_up_error']+config['allocation_tangent_side_error']
    measured=max(0.,np.dot(n,vector(observed.external_force)))
    thrust=force+config['robot_mass']*(np.array([reference.navigation.target_acc_x, reference.navigation.target_acc_y, reference.navigation.target_acc_z])+np.array([0.,0.,config['gravity']]))
    return min(normal,config['max_normal_force']-config['allocation_normal_error']-normal,
        config['allocation_mu']*(normal-config['allocation_normal_error'])-error-tangent,
        config['allocation_mu']*max(0.,measured-config['allocation_observer_normal_error'])-error-tangent,
        config['allocation_thrust_limit']-config['allocation_thrust_reserve']-np.abs(thrust).sum())


class Experiment:
    # Repeated step-and-return records: first four pulses train, last two validate.
    # Normal only increases squeeze; no command drops below nominal preload.
    NORMAL=[0,1,0,.5,0,1,0,.5,0,1,0,.5,0]
    TANGENT=[0,1,0,-1,0,.5,0,-.5,0,1,0,-1,0]

    def __init__(self):
        master=urlparse(os.environ.get('ROS_MASTER_URI',''))
        if master.hostname not in ('localhost','127.0.0.1','::1') or master.port in (None,11311):
            raise RuntimeError('Use an isolated localhost simulation master, never port 11311')
        self.names=['bee1','bee2','bee3','bee4']
        deadline=time.monotonic()+60
        while not rospy.is_shutdown():
            if (rospy.get_param('/use_sim_time',False) and
                rospy.has_param('/bee_cooperative_grasp/model_mass')) and all(rospy.get_param(
                '/'+n+'/simulation/robot_hw_sim_plugin_name','')==
                'mujoco_ros_control/AerialRobotHWSim' for n in self.names):break
            if time.monotonic()>deadline:raise RuntimeError('MuJoCo-only guard failed')
            time.sleep(.1)
        self.config=rospy.get_param('/bee_cooperative_grasp/controller')
        self.amplitude=float(rospy.get_param('~amplitude',.16))
        self.dwell=float(rospy.get_param('~dwell',2.5))
        self.slew=float(rospy.get_param('~slew',1.0))
        self.settle=float(rospy.get_param('~settle_time',120.0))
        self.force_mode=rospy.get_param('~force_mode','pi')
        if self.force_mode not in ('p','pi'):raise ValueError('Use p or pi force mode')
        if not 0<self.amplitude<=.25 or not 1<=self.dwell<=10 or not 0<self.slew<=2:
            raise ValueError('Invalid identification amplitude/dwell/slew')
        if not 3<=self.settle<=300:raise ValueError('Invalid settling interval')
        self.phase='Idle';self.start=None;self.status={};self.previous={};self.delta={}
        self.label='';self.rejections=0;self.finished=False
        self.control_pub={n:rospy.Publisher('/'+n+'/command',BeeCommand,queue_size=1) for n in self.names}
        self.marker=rospy.Publisher('/bee_force_identification/segment',String,queue_size=10,latch=True)
        self.done=rospy.Publisher('/bee_force_identification/done',Bool,queue_size=1,latch=True)
        self.done.publish(False)
        self.subs=[rospy.Subscriber('/bee_cooperative_grasp/phase',String,self.phase_cb)]
        for n in self.names:
            self.subs.append(rospy.Subscriber('/'+n+'/controller/identification_nominal',BeeCommand,self.nominal_control,callback_args=n,queue_size=1))
            self.subs.append(rospy.Subscriber('/'+n+'/controller/tracking_status',ControllerStatus,self.status_cb,callback_args=n,queue_size=1))

    def nominal_control(self, message, name):
        if message.stop:
            self.control_pub[name].publish(message)
            return
        self.reference(message, name)

    def phase_cb(self,message):
        self.phase=message.data
        if self.phase=='Hold' and self.start is None:self.start=rospy.get_time()+self.settle

    def status_cb(self,message,name):
        # Retain compatibility with old status streams containing zero placeholders.
        # Keep the last loaded sample;
        # freshness is still checked before every injected command.
        if message.fault or not message.active or not message.calibrated or np.linalg.norm(vector(message.external_force))>.1:
            self.status[name]=message

    def reference(self,message,name):
        now=message.header.stamp.to_sec();out=copy.deepcopy(message)
        if name in self.previous:
            previous_time,previous=self.previous[name]
            if now<previous_time:
                raise RuntimeError('Simulation clock moved backwards')
            if now==previous_time:
                # No control time elapsed: hold the exact previous command.
                # Wall-timer callbacks can otherwise invent a zero-dt slew.
                self.control_pub[name].publish(previous)
                return
        if self.phase=='Hold':out.force_integral_enabled=self.force_mode=='pi'
        if self.start is None or now<self.start or self.phase!='Hold' or self.finished:
            self.control_pub[name].publish(out);self.previous[name]=(now,copy.deepcopy(out));return
        axis_index=int((now-self.start)//(len(self.NORMAL)*self.dwell))
        if axis_index>=3:
            self.finished=True
            self.marker.publish(json.dumps({'state':'complete','time':now,'rejections':self.rejections}))
            self.done.publish(True);self.control_pub[name].publish(out);return
        axis=['normal','up','side'][axis_index]
        block=int((now-self.start)//self.dwell)%len(self.NORMAL)
        label=f'{axis}:{block}'
        if label!=self.label:
            self.label=label
            self.marker.publish(json.dumps({'state':'excite','axis':axis,'block':block,
                'axis_start':self.start+axis_index*len(self.NORMAL)*self.dwell,
                'dwell':self.dwell,'amplitude_N':self.amplitude,'time':now,
                'validation_from_block':8,'integral_enabled':bool(out.force_integral_enabled)}))
        if not (message.enable_force_control and message.acceleration_bounds):
            raise RuntimeError('Hold must use calibrated vector force control')
        observed=self.status.get(name)
        if observed is None or not observed.active or not observed.calibrated or observed.fault or now-observed.header.stamp.to_sec()>.1:
            raise RuntimeError('Missing identification observer status')
        target=self.amplitude*(self.NORMAL if axis_index==0 else self.TANGENT)[block]
        if axis_index:target*=1 if self.names.index(name)%2==0 else -1
        old_time,previous=self.previous.get(name,(now-.01,message));dt=max(0.,min(.05,now-old_time))
        old_axis,old_delta=self.delta.get(name,(axis,0.))
        if old_axis!=axis:old_delta=0.
        delta=old_delta+np.clip(target-old_delta,-self.slew*dt,self.slew*dt)
        direction = basis(contact_normal(message))[axis_index]
        force = vector(message.force) + delta * direction
        set_vector(out.force, force)
        old_force = vector(previous.force)
        margin=margins(out,observed,self.config)
        change=float(np.max(abs(force-old_force)))
        if margin<-1e-5 or change>self.config['allocation_force_rate']*dt+1e-5:
            # Do not produce a clipped, unrecorded test input. Forward nominal,
            # mark the invalid experiment and let the runner retain the record.
            self.rejections+=1
            self.marker.publish(json.dumps({'state':'rejected','axis':axis,'time':now,'bee':name,
                'minimum_margin_N':margin,'force_change_N':change,'allowed_change_N':self.config['allocation_force_rate']*dt}))
            out=copy.deepcopy(message);out.force_integral_enabled=self.force_mode=='pi';delta=0.
        self.delta[name]=(axis,delta);self.previous[name]=(now,copy.deepcopy(out))
        self.control_pub[name].publish(out)


if __name__=='__main__':
    rospy.init_node('bee_force_identification')
    experiment=Experiment()
    rospy.spin()
