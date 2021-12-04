#!/usr/bin/env python2

# -*- coding: utf-8 -*-

import numpy as np
import os
import csv
import matplotlib.pyplot as plt
import math
import sys
import re

# Define argumento of num scenario and num position
print("\n\t *** IMPORTANT: Use this script given two parameters: <scenario_number> <initial_position_number> ***\n")  # Warnning message
arg1 = sys.argv[1] # number of scenario to analize
arg2 = sys.argv[2] # number of position to analize

## Get Data and compute percentage related with Trajectory Optimized
#Open and managing files and data
path_I = "/home/simon/results_marsupial_optimizer/"
file_pathI = path_I + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_initial_kinematic_ugv-uav.txt'
path_O = "/home/simon/results_marsupial_optimizer/"
file_pathO = path_O + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_optimized_kinematic_ugv-uav.txt'
path_ugv = "/home/simon/results_marsupial_optimizer/"
file_path_ugv = path_ugv + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UGV.txt'
path_uav = "/home/simon/results_marsupial_optimizer/"
file_path_uav = path_uav + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UAV.txt'

# Initialization variables
sum_smooth_ugv = t_I_sum_smooth_ugv = t_O_sum_smooth_ugv = 0
sum_smooth_uav = t_I_sum_smooth_uav = t_O_sum_smooth_uav =0
count = 0
T_count = []
MI_ugv = []
MI_uav = []
MO_ugv = []
MO_uav = []
TI_mean_ugv = []
TI_mean_uav = []
TO_mean_ugv = []
TO_mean_uav = []
TI_ugv = []
TO_ugv = []
TI_uav = []
TO_uav = []

dist_obs_i_ugv_mean = []
dist_obs_i_ugv_min = []
dist_obs_i_cat_mean = [] 
dist_obs_i_cat_min = []
dist_obs_o_ugv_mean = [] 
dist_obs_o_ugv_min = []
dist_obs_o_cat_mean = [] 
dist_obs_o_cat_min = []
dist_obs_i_uav_mean = []
dist_obs_i_uav_min = []
dist_obs_i_cat_mean = [] 
dist_obs_i_cat_min = []
dist_obs_o_uav_mean = [] 
dist_obs_o_uav_min = []
dist_obs_o_cat_mean = [] 
dist_obs_o_cat_min = []
M_dist_obs_mean_ugv = []
M_dist_obs_min_ugv = []
M_dist_obs_mean_uav = []
M_dist_obs_min_uav = []
M_dist_obs_mean_cat = []
M_dist_obs_min_cat = []

# Reading Files
fileI = open(file_pathI, 'r')
TlinesI = fileI.read().splitlines()
fileO = open(file_pathO, 'r')
TlinesO = fileO.read().splitlines()
file_ugv = open(file_path_ugv, 'r')
Tlines_ugv = file_ugv.read().splitlines()
Tlines_ugv.pop(0) # To remove first line in case to have titles
file_uav = open(file_path_uav, 'r')
Tlines_uav = file_uav.read().splitlines()
Tlines_uav.pop(0) # To remove first line in case to have titles

# Filling Tuples
for l in TlinesI:
    line = re.split(';|/|', l)
    sum_smooth_ugv = 0
    sum_smooth_uav = 0
    count+=1
    TI_ugv = []
    TI_uav = []
    for ln in range(0, len(line)/3, 3):
        TI_ugv.append(float(line[ln+1]))
        TI_uav.append(float(line[ln+2]))
        sum_smooth_ugv = sum_smooth_ugv + float(line[ln+1])
        sum_smooth_uav = sum_smooth_uav + float(line[ln+2])
    MI_ugv.append(TI_ugv)
    MI_uav.append(TI_uav)
    T_count.append(count)
    TI_mean_ugv.append(sum_smooth_ugv/len(TI_ugv))
    TI_mean_uav.append(sum_smooth_uav/len(TI_uav))
    t_I_sum_smooth_ugv = t_I_sum_smooth_ugv + sum_smooth_ugv/len(TI_ugv)
    t_I_sum_smooth_uav = t_I_sum_smooth_uav + sum_smooth_uav/len(TI_uav)
I_mean_smooth_ugv = t_I_sum_smooth_ugv/count
I_mean_smooth_uav = t_I_sum_smooth_uav/count

for l in TlinesO:
    line = re.split(';|/|', l)
    sum_smooth_ugv = 0
    sum_smooth_uav = 0
    TO_ugv = []
    TO_uav = []
    for ln in range(0, len(line)/3, 3):
        TO_ugv.append(float(line[ln+1]))
        TO_uav.append(float(line[ln+2]))
        sum_smooth_ugv = sum_smooth_ugv + float(line[ln+1])
        sum_smooth_uav = sum_smooth_uav + float(line[ln+2])
    MO_ugv.append(TO_ugv)
    MO_uav.append(TO_uav)
    TO_mean_ugv.append(sum_smooth_ugv/len(TO_ugv))
    TO_mean_uav.append(sum_smooth_uav/len(TO_uav))
    t_O_sum_smooth_ugv = t_O_sum_smooth_ugv + sum_smooth_ugv/len(TO_ugv)
    t_O_sum_smooth_uav = t_O_sum_smooth_uav + sum_smooth_uav/len(TO_uav)

O_mean_smooth_ugv = t_O_sum_smooth_ugv/count
O_mean_smooth_uav = t_O_sum_smooth_uav/count

for l in Tlines_ugv:
    line = re.split(';', l)
    dist_obs_i_ugv_mean.append(float(line[6]))
    dist_obs_i_ugv_min.append(float(line[7]))
    dist_obs_o_ugv_mean.append(float(line[8]))
    dist_obs_o_ugv_min.append(float(line[9]))
    # dist_obs_i_cat_mean.append(float(line[10]))
    # dist_obs_i_cat_min.append(float(line[11]))
    # dist_obs_o_cat_mean.append(float(line[12]))
    # dist_obs_o_cat_min.append(float(line[13]))
M_dist_obs_mean_ugv.append(dist_obs_i_ugv_mean)
M_dist_obs_mean_ugv.append(dist_obs_o_ugv_mean)
M_dist_obs_min_ugv.append(dist_obs_i_ugv_min)
M_dist_obs_min_ugv.append(dist_obs_o_ugv_min)
# M_dist_obs_mean_cat.append(dist_obs_i_cat_mean)
# M_dist_obs_mean_cat.append(dist_obs_o_cat_mean)
# M_dist_obs_min_cat.append(dist_obs_i_cat_min)
# M_dist_obs_min_cat.append(dist_obs_o_cat_min)


for l in Tlines_uav:
    line = re.split(';', l)
    dist_obs_i_uav_mean.append(float(line[6]))
    dist_obs_i_uav_min.append(float(line[7]))
    dist_obs_o_uav_mean.append(float(line[8]))
    dist_obs_o_uav_min.append(float(line[9]))
    dist_obs_i_cat_mean.append(float(line[10]))
    dist_obs_i_cat_min.append(float(line[11]))
    dist_obs_o_cat_mean.append(float(line[12]))
    dist_obs_o_cat_min.append(float(line[13]))
M_dist_obs_mean_uav.append(dist_obs_i_uav_mean)
M_dist_obs_mean_uav.append(dist_obs_o_uav_mean)
M_dist_obs_min_uav.append(dist_obs_i_uav_min)
M_dist_obs_min_uav.append(dist_obs_o_uav_min)
M_dist_obs_mean_cat.append(dist_obs_i_cat_mean)
M_dist_obs_mean_cat.append(dist_obs_o_cat_mean)
M_dist_obs_min_cat.append(dist_obs_i_cat_min)
M_dist_obs_min_cat.append(dist_obs_o_cat_min)


# Fist Figure(line plot UGV) 
fig1, axs1 = plt.subplots(3, figsize=(16, 9), sharex=True, sharey=False, subplot_kw=dict(aspect="auto"))
fig1.suptitle('Result Smoothness UGV (test_number ='+str(len(TI_mean_uav))+")", fontsize=21, weight="bold")

l_ugv_I = axs1[0].plot(T_count, TI_mean_ugv, '-', color='red',label='mean ugv initial')
l_ugv_O = axs1[0].plot(T_count, TO_mean_ugv, '-', color='green',label='mean ugv optimized')
l_ugv = l_ugv_I+l_ugv_O
labs_ugv = [l.get_label() for l in l_ugv]
axs1[0].legend(l_ugv, labs_ugv, loc=0)
axs1[0].tick_params(axis='y', labelcolor='green')
axs1[0].grid(color='pink')
axs1[0].set_ylabel('Mean Smoothness [rad]',color="green", fontsize=12)
# axs1[0].set_xlabel('Experiment Number', fontsize=12)
T_av_I_var_smooth_ugv = [I_mean_smooth_ugv] * len(T_count)
axs1[1].boxplot(MI_ugv)
axs1[1].plot(T_count, T_av_I_var_smooth_ugv, '-', color='green', label='mean GP ={:.4f}'.format(np.mean(I_mean_smooth_ugv)))
axs1[1].legend()
axs1[1].grid(color='pink')
axs1[1].set_ylabel('Init Variation Smooth.[rad]',color="green", fontsize=12)
# axs1[1].set_xlabel('Experiment Number', fontsize=12)
T_av_O_var_smooth_ugv = [O_mean_smooth_ugv] * len(T_count)
axs1[2].boxplot(MO_ugv)
axs1[2].plot(T_count, T_av_O_var_smooth_ugv, '-', color='green', label='mean GP ={:.4f}'.format(np.mean(O_mean_smooth_ugv)))
axs1[2].legend()
axs1[2].grid(color='pink')
axs1[2].set_ylabel('Opt Variation Smooth.[rad]',color="green", fontsize=12)
axs1[2].set_xlabel('Experiment Number', fontsize=12)

plt.subplots_adjust(left=0.04, bottom=0.08, right=0.98, top=0.92, hspace= 0)

# Second Figure (line plot UAV) 
fig2, axs2 = plt.subplots(3, figsize=(16, 9), sharex=True, sharey=False, subplot_kw=dict(aspect="auto"))
fig2.suptitle('Result Smoothness UAV (test_number ='+str(len(TI_mean_uav))+")", fontsize=21, weight="bold")

l_uav_I = axs2[0].plot(T_count, TI_mean_uav, '-', color='red',label='mean uav initial')
l_uav_O = axs2[0].plot(T_count, TO_mean_uav, '-', color='blue',label='mean uav optimized')
l_uav = l_uav_I+l_uav_O
labs_uav = [l.get_label() for l in l_uav]
axs2[0].legend(l_uav, labs_uav, loc=0)
axs2[0].grid(color='pink')
axs2[0].set_ylabel('Mean Smoothness [rad]',color="blue", fontsize=12)
# axs2[0].set_xlabel('Experiment Number', fontsize=12)
T_av_I_var_smooth_uav = [I_mean_smooth_uav] * len(T_count)
axs2[1].boxplot(MI_uav)
axs2[1].plot(T_count, T_av_I_var_smooth_uav, '-', color='blue', label='mean GP ={:.4f}'.format(np.mean(I_mean_smooth_uav)))
axs2[1].legend()
axs2[1].grid(color='pink')
axs2[1].set_ylabel('Init Variation Smooth.[rad]',color="blue", fontsize=12)
# axs2[1].set_xlabel('Experiment Number', fontsize=12)
T_av_O_var_smooth_uav = [O_mean_smooth_uav] * len(T_count)
axs2[2].boxplot(MO_uav)
axs2[2].plot(T_count, T_av_O_var_smooth_uav, '-', color='blue', label='mean GP ={:.4f}'.format(np.mean(O_mean_smooth_uav)))
axs2[2].legend()
axs2[2].grid(color='pink')
axs2[2].set_ylabel('Opt Variation Smooth.[rad]',color="blue", fontsize=12)
axs2[2].set_xlabel('Experiment Number', fontsize=12)
plt.subplots_adjust(left=0.04, bottom=0.08, right=0.98, top=0.92, hspace= 0) #hspace reduce space between subplots

# Third Figure (Distance obstacles UGV, UAV and CAT) 
fig3, axs3 = plt.subplots(2, 3, figsize=(16, 9), sharex=True, sharey=False, subplot_kw=dict(aspect="auto"))
fig3.suptitle('Distance Obstacles (test_number ='+str(len(TI_mean_uav))+")", fontsize=21, weight="bold")
axs3[0,0].boxplot(M_dist_obs_mean_ugv, labels=['Initial','Optimized'])
axs3[0,0].set_ylabel('Mean dist. obs. UGV [m]',color="orange", fontsize=12)
axs3[0,0].grid(color='pink')
axs3[1,0].boxplot(M_dist_obs_min_ugv, labels=['Initial','Optimized'])
axs3[1,0].set_ylabel('Min. dist. obs. UGV [m]',color="orange", fontsize=12)
axs3[1,0].grid(color='pink')
axs3[0,1].boxplot(M_dist_obs_mean_uav, labels=['Initial','Optimized'])
axs3[0,1].set_ylabel('Mean dist. obs. UAV [m]',color="orange", fontsize=12)
axs3[0,1].grid(color='pink')
axs3[1,1].boxplot(M_dist_obs_min_uav, labels=['Initial','Optimized'])
axs3[1,1].set_ylabel('Min. dist. obs. UAV [m]',color="orange", fontsize=12)
axs3[1,1].grid(color='pink')
axs3[0,2].boxplot(M_dist_obs_mean_cat, labels=['Initial','Optimized'])
axs3[0,2].set_ylabel('Mean dist. obs. CAT [m]',color="orange", fontsize=12)
axs3[0,2].grid(color='pink')
axs3[1,2].boxplot(M_dist_obs_min_cat, labels=['Initial','Optimized'])
axs3[1,2].set_ylabel('Min. dist. obs. CAT [m]',color="orange", fontsize=12)
axs3[1,2].grid(color='pink')

plt.subplots_adjust(left=0.04, bottom=0.08, right=0.98, top=0.92, hspace= 0) #hspace reduce space between subplots

plt.show()