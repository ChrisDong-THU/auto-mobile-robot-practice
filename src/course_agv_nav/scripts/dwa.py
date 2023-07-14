#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math
import random

class Config():
    def __init__(self) -> None:
        self.predict_dt = 0.1
        self.predict_time = 3

        self.max_speed = 0.5
        self.min_speed = 0.05
        self.max_yaw = 1

        self.max_accel = 4
        self.max_dif_yaw = math.pi

        # Samples
        self.v_samples = 10
        self.w_samples = 20

        # Distance cost: distance to obstacles along the trajectory
        self.dist_gain = 20
        # Velocity cost: vehicle's velocity, the larger the better, shorter time spent
        self.vel_gain = 5
        # Goal cost: distance to goal at the end of trajectory
        self.goal_gain = 20
        
        # Vehicle's radius
        self.robot_dis = 0.15
        self.obs_dis = 0.05

class DWA:
    def __init__(self):
        self.config = Config()
    
    # 速度空间内采样
    @staticmethod
    def sample(current_v, dv) -> float:
        current_v += (2*random.random()-1)*dv
             
        return current_v
    
    def v_set(self, v, level):
        v = max(self.config.min_speed, min(self.config.max_speed, v))

        # 阶梯抬速
        if level>3 and v<=0.2:
            v = v*0.3 + 0.15
        elif level >0.5 and v<=0.1:
            v = v*0.4 + 0.06
        elif level<0.2 and v>=0.1:
            v = v*0.5 + 0.05

        return v

    def w_set(self, w):
        w = max(-self.config.max_yaw, min(self.config.max_yaw, w))

        return w

    def goal_cost(self, traj, goal):
        dis = math.hypot(goal[0]-traj[-1][0], goal[1]-traj[-1][1])
        return dis

    def obs_cost(self, traj, ob):
        """
        Returns 0 if there is a collision with an obstacle
        """
        min_dist = 999
        nearest_obs = 999

        for node in traj:
            for i in range(len(ob)):
                dist = math.hypot(node[0] - ob[i, 0], node[1] - ob[i, 1])
                if dist < nearest_obs:
                    nearest_obs = dist

            if nearest_obs <= min_dist:
                min_dist = nearest_obs

        if min_dist < self.config.robot_dis+self.config.obs_dis:
            return 0
        
        return 1 / min_dist

    def vel_cost(self, v):
        return abs(self.config.max_speed*1.2 - v)

    def traj_cal(self, vx, vw):
        '''
        calculate one traj
        '''
        node = (0, 0, 0)
        traj = np.array(node)
        time = 0
        while time <= self.config.predict_time:
            node = self.step_cal(node, vx, vw)
            traj = np.vstack((traj, node))
            time += self.config.predict_dt
        return traj

    def step_cal(self, node, v, w):
        '''
        calculate one step: [x, y, orientation]
        '''
        next=[]
        next.append(node[0] + v * math.cos(node[2]) * self.config.predict_dt)
        next.append(node[1] + v * math.sin(node[2]) * self.config.predict_dt)
        next.append(node[2] + w * self.config.predict_dt)

        return np.array(next)
    
    def dwa_core(self, info, goal, ob):
        '''
        Calculate the least cost traj
        '''
        min_cost = 999
        
        # 损失存储字典
        cost_map = {}
        bset_vw = (0, 0)
        dist_total = 1
        vel_total = 0
        goal_total = 0

        level = math.hypot(goal[0], goal[1])
        print('goal:',goal)

        
        for j in range(self.config.w_samples):
            w = self.sample(info[4], self.config.max_dif_yaw*self.config.predict_dt)
            w = self.w_set(w)
            for i in range(self.config.v_samples):
                v = self.sample(info[3], self.config.max_accel*self.config.predict_dt)
                v = self.v_set(v, level)
                # 急转弯情形速度限幅度，避免出现过度预测轨迹情况
                if abs(w)>self.config.max_yaw*0.6:
                    v = max(v*0.1, 0.1)

                traj = self.traj_cal(v, w)

                d_cost = self.obs_cost(traj, ob)
                dist_total += d_cost

                v_cost = self.vel_cost(v)
                vel_total += v_cost

                g_cost = self.goal_cost(traj, goal)
                goal_total += g_cost

                cost_map[(v, w)] = (d_cost, v_cost, g_cost)

        for i in cost_map:
            # obs_cost=0 说明遇到障碍物，跳过轨迹，防止归一化时趋于0
            if cost_map[i][0] == 0:
                continue
            # 归一化
            normalized_dist_cost = self.config.dist_gain * cost_map[i][0] / dist_total
            normalized_vel_cost = self.config.vel_gain * cost_map[i][1] / vel_total
            normalized_goal_cost = self.config.goal_gain * cost_map[i][2] / goal_total
            total_cost = normalized_dist_cost + normalized_vel_cost + normalized_goal_cost
            if total_cost < min_cost:
                min_cost = total_cost
                bset_vw = i

        return bset_vw
    
    def plan(self, info, goal, ob):
        # Get the velocity and yawrate window for this planning, constrained by acceleration and maximum speed
        u = self.dwa_core(info, goal, ob)
        
        return u