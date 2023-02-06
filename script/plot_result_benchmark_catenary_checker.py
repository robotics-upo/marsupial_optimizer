#!/usr/bin/env python3

# -*- coding: utf-8 -*-
import numpy as np
import os
import csv
import matplotlib.pyplot as plt
import math
import sys
import re

# Define argumento of num scenario and num position
print("\n\t *** IMPORTANT: Use this script given two parameters: <pc_user_name> <scenario_number> <initial_position_number> ***\n")  # Warnning message
pc_user = sys.argv[1] # number of position to analize

## Get Data and compute percentage related with Trajectory Optimized
#Open and managing files and data
user_ = pc_user # Change this value to the user name

path_parable = '/home/'+ user_ +'/results_marsupial_optimizer/'
file_path_parable = path_parable + 'catenary_file_parable.txt'
path_catenary = '/home/'+ user_ +'/results_marsupial_optimizer/'
file_path_catenary = path_catenary + 'catenary_file_catenary.txt'

# Initialization variables
count = 0
TI_mean_uav = []

M_t_path_cat = []
M_t_mean_cat = []
M_t_max_cat = []
M_t_min_cat = []
M_t_path_par = []
dist_obs_mean_cat = [] 
dist_obs_min_cat = []
M_t_path_par = []
M_t_mean_par = []
M_t_max_par = []
M_t_min_par = []
dist_obs_mean_par = [] 
dist_obs_min_par = []
M_dist_obs_min   = []
M_dist_obs_mean = []
count = 0
T_count_par = []
T_count_cat = []

t_maxPathPar = 0.0
t_minPathPar = 10000000.0
t_maxPathCat = 0.0
t_minPathCat = 0.0
t_MaxPar = 0.0
t_maxMinPar = 0.0
t_MaxCat = 0.0
t_maxMinCat = 0.0
t_maxMeanPar = 0.0
t_maxMeanCat = 0.0

# Reading Files
file_parable = open(file_path_parable, 'r')
Tline_parable = file_parable.read().splitlines()
Tline_parable.pop(0) # To remove first line in case to have titles
file_catenary = open(file_path_catenary, 'r')
Tline_catenary = file_catenary.read().splitlines()
Tline_catenary.pop(0) # To remove first line in case to have titles

# Filling Tuples
for l in Tline_parable:
    line = re.split(';', l)
    count+=1
    M_t_path_par.append(float(line[0]))
    if (t_maxPathPar < float(line[0])):
        t_maxPathPar = float(line[0])
    if (t_minPathPar > float(line[0])):
        t_minPathPar = float(line[0])
    M_t_mean_par.append(float(line[1]))
    if (t_maxMeanPar < float(line[1])):
        t_maxMeanPar = float(line[1])
    M_t_max_par.append(float(line[2]))
    if (t_MaxPar < float(line[2])):
        t_MaxPar = float(line[2])
    M_t_min_par.append(float(line[3]))
    if (t_maxMinPar < float(line[3])):
        t_maxMinPar = float(line[3])
    dist_obs_mean_par.append(float(line[4]))
    dist_obs_min_par.append(float(line[5]))
    T_count_par.append(count)
M_dist_obs_mean.append(dist_obs_mean_par)
M_dist_obs_min.append(dist_obs_min_par)
print("t_maxPathPar= ",t_maxPathPar," , t_maxMeanPar= ",t_maxMeanPar," , t_MaxPar= ",t_MaxPar," , t_maxMinPar= ",t_maxMinPar)

count = 0
# Filling Tuples
for l in Tline_catenary:
    line = re.split(';', l)
    count+=1
    M_t_path_cat.append(float(line[0]))
    if (t_maxPathCat < float(line[0])):
        t_maxPathCat = float(line[0])
    if (t_minPathCat > float(line[0])):
        t_minPathCat = float(line[0])
    M_t_mean_cat.append(float(line[1]))
    if (t_maxMeanCat < float(line[1])):
        t_maxMeanCat = float(line[1])
    M_t_max_cat.append(float(line[2]))
    if (t_MaxCat < float(line[2])):
        t_MaxCat = float(line[2])
    M_t_min_cat.append(float(line[3]))
    if (t_maxMinCat < float(line[3])):
        t_maxMinCat = float(line[3])
    dist_obs_mean_cat.append(float(line[4]))
    dist_obs_min_cat.append(float(line[5]))
    T_count_cat.append(count)
M_dist_obs_mean.append(dist_obs_mean_cat)
M_dist_obs_min.append(dist_obs_min_cat)
print("t_maxPathCat= ",t_maxPathCat," , t_maxMeanCat= ",t_maxMeanCat," , t_MaxCat= ",t_MaxCat," , t_maxMinCat= ",t_maxMinCat)


# First Figure (Distance obstacles UGV, UAV and CAT) 
fig1, axs1 = plt.subplots(1, 2, figsize=(16, 9), sharex=True, sharey=False, subplot_kw=dict(aspect="auto"))
fig1.suptitle('Distance Obstacles (Nº test par='+ str(len(T_count_par))+ ' and Nº test cat=' +str(len(T_count_cat))+')', fontsize=21, weight="bold")
axs1[0].boxplot(M_dist_obs_mean, labels=['Parable','Catenary'])
axs1[0].set_ylabel('Mean dist. obs. [m]',color="orange", fontsize=12)
axs1[0].grid(color='pink')
axs1[1].boxplot(M_dist_obs_min, labels=['Parable','Catenary'])
axs1[1].set_ylabel('Min dist. obs. [m]',color="orange", fontsize=12)
axs1[1].grid(color='pink')

plt.subplots_adjust(left=0.04, bottom=0.08, right=0.98, top=0.92, hspace= 0) #hspace reduce space between subplots

# Second Figure (line plot UAV) 
fig2, axs2 = plt.subplots(4, figsize=(16, 9), sharex=True, sharey=False, subplot_kw=dict(aspect="auto"))
fig2.suptitle('Time computing results (Nº test par='+ str(len(T_count_par))+ ' and Nº test cat=' +str(len(T_count_cat))+')', fontsize=21, weight="bold")
t_path_parable = axs2[0].plot(T_count_par, M_t_path_par, '-', color='red',label='t path parable')
axs20 = axs2[0].twinx()
t_path_catenary = axs20.plot(T_count_cat, M_t_path_cat, '-', color='blue',label='t path catenary')
t_path = t_path_parable + t_path_catenary
labs_path = [l.get_label() for l in t_path]
axs2[0].legend(t_path, labs_path, loc=0)
axs2[0].tick_params(axis='y', labelcolor='red')
axs2[0].grid(color='pink')
axs2[0].set_ylabel('Get Path [s]',color="red", fontsize=10)
axs2[0].set_ylim(ymin=(t_minPathPar), ymax=(t_maxPathPar))
axs20.tick_params(axis='y', labelcolor='blue')
axs20.grid(color='gray')
axs20.set_ylabel("Get Path [s]",color="blue",fontsize=12)
axs20.set_ylim(ymin=(t_minPathCat), ymax=(t_maxPathCat))

t_mean_parable = axs2[1].plot(T_count_par, M_t_mean_par, '-', color='red',label='t mean parable')
axs21 = axs2[1].twinx()
t_mean_catenary = axs21.plot(T_count_cat, M_t_mean_cat, '-', color='blue',label='t mean catenary')
t_mean = t_mean_parable + t_mean_catenary
labs_mean_t = [l.get_label() for l in t_path]
axs2[1].legend(t_mean, labs_mean_t, loc=0)
axs2[1].tick_params(axis='y', labelcolor='red')
axs2[1].grid(color='pink')
axs2[1].set_ylabel('compute mean value[s]',color="red", fontsize=10)
axs2[1].set_ylim(ymin=(0.0), ymax=(t_maxMeanPar))
axs21.tick_params(axis='y', labelcolor='blue')
axs21.grid(color='gray')
axs21.set_ylabel("compute mean value[s]",color="blue",fontsize=12)
ymin_ = 0.0
axs21.set_ylim(ymin=(0.0), ymax=(t_maxMeanCat))

t_max_parable = axs2[2].plot(T_count_par, M_t_max_par, '-', color='red',label='t max parable')
axs22 = axs2[2].twinx()
t_max_catenary = axs22.plot(T_count_cat, M_t_max_cat, '-', color='blue',label='t max catenary')
t_max = t_max_parable + t_max_catenary
labs_max_t = [l.get_label() for l in t_path]
axs2[2].legend(t_max, labs_max_t, loc=0)
axs2[2].tick_params(axis='y', labelcolor='red')
axs2[2].grid(color='pink')
axs2[2].set_ylabel('compute max value[s]',color="red", fontsize=10)
axs2[2].set_ylim(ymin=(0.0), ymax=(t_MaxPar))
axs22.tick_params(axis='y', labelcolor='blue')
axs22.grid(color='gray')
axs22.set_ylabel("compute max value[s]",color="blue",fontsize=12)
axs22.set_ylim(ymin=(0.0), ymax=(t_MaxCat))

t_min_parable = axs2[3].plot(T_count_par, M_t_min_par, '-', color='red',label='t min parable')
axs23 = axs2[3].twinx()
t_min_catenary = axs23.plot(T_count_cat, M_t_min_cat, '-', color='blue',label='t min catenary')
t_min = t_min_parable + t_min_catenary
labs_min_t = [l.get_label() for l in t_path]
axs2[3].legend(t_min, labs_min_t, loc=0)
axs2[3].tick_params(axis='y', labelcolor='red')
axs2[3].grid(color='pink')
axs2[3].set_ylabel('compute min value[s]',color="red", fontsize=10)
axs2[3].set_ylim(ymin=(0.0), ymax=(t_maxMinPar))
axs23.tick_params(axis='y', labelcolor='blue')
axs23.grid(color='gray')
axs23.set_ylabel("compute min value[s]",color="blue",fontsize=12)
axs23.set_ylim(ymin=(0.0), ymax=(t_maxMinCat))

plt.subplots_adjust(left=0.07, bottom=0.08, right=0.94, top=0.92, hspace= 0) #hspace reduce space between subplots

plt.show()