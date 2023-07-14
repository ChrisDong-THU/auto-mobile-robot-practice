#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import math

class Config():
    def __init__(self) -> None:
        self.predict_dt = 0.1
        self.predict_time = 3.6

        self.max_speed = 0.3
        self.min_speed = 0
        self.max_yawrate = 0.15

        self.max_accel = 0.5
        self.max_dif_yaw = math.pi

        # Resolution of the window, velocity and yawrate resolution
        self.v_reso = 0.02
        self.yaw_reso = math.pi / 36

        # Heading cost: angle between the heading of the trajectory's end and the line connecting it to the goal
        self.heading_gain = 0
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

    def velocity_generate(self, info, level):
        '''
        当前状态可采样的速度空间的范围限定

        :param info: 当前信息
        :param level: 抬升区间
        :return: 速度范围[v_min, v_max, w_min, w_max]
        '''
        vlimit = [self.config.min_speed, self.config.max_speed, -self.config.max_yawrate, self.config.max_yawrate]
        # print('level:',level)
        vel = max(info[3],0)
        if level>6:
            vlevel=vel*0.3+0.15
        elif level>=2:
            vlevel=vel*0.3+0.1
        elif level>=0.8:
            vlevel=vel*0.6+0.05
        else:
            vlevel=info[3]

        wlevel=info[4]*0.1
        # 单步在dt时间内以最大加速度变化可达范围
        vmove = [vlevel - self.config.max_accel * self.config.predict_dt,
                 vlevel + self.config.max_accel * self.config.predict_dt,
                 wlevel - self.config.max_dif_yaw * self.config.predict_dt,
                 wlevel + self.config.max_dif_yaw * self.config.predict_dt]

        # 实际最大速度与角速度约束
        vw = [max(vlimit[0], vmove[0]), min(vlimit[1], vmove[1]),
              max(vlimit[2], vmove[2]), min(vlimit[3], vmove[3])]

        return vw

    def heading_cost(self, traj, goal):
        dx = goal[0] - traj[-1][0]
        dy = goal[1] - traj[-1][1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - traj[-1][2]
        dis = math.hypot(dx, dy)
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        return cost

    def goal_cost(self, traj, goal):
        dis = math.hypot(goal[0]-traj[-1][0], goal[1]-traj[-1][1])
        return dis

    def obs_cost(self, traj, ob):
        """
        Returns 0 if there is a collision with an obstacle
        """
        min_dist = float("inf")
        nearest = 999

        for node in traj:
            for i in range(len(ob)):
                dist = math.hypot(node[0] - ob[i, 0], node[1] - ob[i, 1])
                if dist < nearest:
                    nearest = dist

            if nearest < min_dist:
                min_dist = nearest

        if min_dist < self.config.robot_dis+self.config.obs_dis:
            return 0
        return 1 / min_dist

    def vel_cost(self, v):
        return abs(self.config.max_speed - v)

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
    
    def dwa_core(self, vw, goal, ob):
        '''
        Calculate the least cost traj
        '''
        min_cost = float("inf")
        
        # 损失存储字典
        cost_map = {}
        bset_vw = (0, 0)
        heading_total = 0
        dist_total = 1
        vel_total = 0
        goal_total = 0

        for v in np.arange(vw[0], vw[1], self.config.v_reso):
            for w in np.arange(vw[2], vw[3], self.config.yaw_reso):
                traj = self.traj_cal(v, w)
                h_cost = self.heading_cost(traj, goal)
                heading_total += h_cost

                d_cost = self.obs_cost(traj, ob)
                dist_total += d_cost

                v_cost = self.vel_cost(v)
                vel_total += v_cost

                g_cost = self.goal_cost(traj, goal)
                goal_total += g_cost

                cost_map[(v, w)] = (h_cost, d_cost, v_cost, g_cost)

        for i in cost_map:
            if cost_map[i][1] == 0:
                continue
            # 归一化
            normaled_h = self.config.heading_gain * cost_map[i][0] / heading_total
            normaled_d = self.config.dist_gain * cost_map[i][1] / dist_total
            normaled_v = self.config.vel_gain * cost_map[i][2] / vel_total
            normaled_g = self.config.goal_gain * cost_map[i][3] / goal_total
            total_cost = normaled_h + normaled_d + normaled_v + normaled_g
            if total_cost < min_cost:
                min_cost = total_cost
                bset_vw = i

        return bset_vw
    
    def plan(self, info, goal, ob):
        # Get the velocity and yawrate window for this planning, constrained by acceleration and maximum speed
        level = math.hypot(goal[0], goal[1])
        print('goal:',goal)
        vw = self.velocity_generate(info, level)
        u = self.dwa_core(vw, goal, ob)
        
        return u