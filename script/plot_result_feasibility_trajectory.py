#!/usr/bin/env python3

# -*- coding: utf-8 -*-

import numpy as np
import os
import csv
import matplotlib.pyplot as plt
import math
import sys

# Define argumento of num scenario and num position
print("\n\t *** IMPORTANT: Use this script given two parameters: <pc_user_name> <scenario_number> <initial_position_number> ***\n")  # Warnning message
pc_user = sys.argv[1] # number of position to analize
arg1 = sys.argv[2] # number of scenario to analize
arg2 = sys.argv[3] # number of position to analize

## Get Data and compute percentage related with Trajectory Optimized
#Open and managing files and data
user_ = pc_user # Change this value to the user name

path = '/home/'+ user_ +'/results_marsupial_optimizer/'
file_path = path + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_feasibility_trajectory.txt'
path_time = '/home/'+ user_ +'/results_marsupial_optimizer/'
file_path_time = path_time + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UAV.txt'

count = 0
count_T = 0
count_feasibility = 0
count_ugv_coll = 0
count_uav_coll = 0
count_cat_coll = 0
sum_time_op = 0.0
sum_time_gp = 0.0

time_optimizer =[]
time_gp = []
count_all = []
count_all_T = []
cost_init_sol =[]
cost_opt_sol = []

file = open(file_path, 'r')
Tlines = file.read().splitlines()
Tlines.pop(0) # To remove first line in case to have titles

file_time = open(file_path_time, 'r')
f_time_lines = file_time.read().splitlines()
f_time_lines.pop(0) # To remove first line in case to have titles

max_value_time_op = 0 #to save max value for line plot
min_value_time_op = 10000000 #to save min value for line plot
for l in Tlines:
    line = l.split(';') 
    if (float(line[0]) > 0):
        count_feasibility += 1.0
    if (float(line[1]) > 0):
        count_ugv_coll += 1.0
    if (float(line[2]) > 0):
        count_uav_coll += 1.0
    if (float(line[3]) > 0):
        count_cat_coll += 1.0
    if (float(line[11]) > 0):
        sum_time_op += float(line[11])
        time_optimizer.append(float(line[11]))
    count+=1.0
    count_all.append(count)
    cost_init_sol.append(float(line[6]))
    cost_opt_sol.append(float(line[7]))
    if (max_value_time_op < float(line[11])):
        max_value_time_op = float(line[11])    
    if (min_value_time_op > float(line[11])):
        min_value_time_op = float(line[11]) 

max_value_time_gp = 0 #to save max value for line plot
min_value_time_gp = 10000000 #to save min value for line plot
for l in f_time_lines:
    line = l.split(';') 
    time_gp.append(float(line[0]))
    sum_time_gp += float(line[0])
    count_T+=1.0
    count_all_T.append(count_T)
    if (max_value_time_gp < float(line[0])):
        max_value_time_gp = float(line[0])  
    if (min_value_time_gp > float(line[0])):
        min_value_time_gp = float(line[0]) 
        
percentage_feasibility = (count_feasibility * 100.0)/count
percentage_ugv_coll = (count_ugv_coll * 100.0)/count
percentage_uav_coll = (count_uav_coll * 100.0)/count
percentage_cat_coll = (count_cat_coll * 100.0)/count
average_time_op = sum_time_op/float(count)
average_time_gp = sum_time_gp/float(count_T)

print("Percentage_feasibility: "+ str(percentage_feasibility))
print("Percentage_ugv_coll: " + str(percentage_ugv_coll))
print("Percentage_uav_coll: " + str(percentage_uav_coll))
print("Percentage_cat_coll: " + str(percentage_cat_coll))    
print("Total number of test: " + str(count))

## Get Plot DataTrajectory Optimized

# Make figure and axes
# fig, axs = plt.subplots(2, 3, figsize=(12, 9), subplot_kw=dict(aspect="equal"))
fig, axs = plt.subplots(2, 3, figsize=(12, 9), subplot_kw=dict(aspect="auto"), gridspec_kw={'width_ratios': [2, 1, 1]})
# fig, axs = plt.subplots(2, 3, figsize=(12, 9))
fig.suptitle('Result factibility and collision trajectory optimized (test_number ='+str(count)+")", fontsize=21, weight="bold")

# First Bar Plot (bar plot for average optimizer time)
# labels5 = ['Average']
# time_means = [average_time_op]
# width1 = 120  # the width of the bars
# rects1 = axs[0,0].bar(labels5, time_means, width1)
# axs[0,0].set_ylabel('Time[sec]')
# axs[0,0].set_title('Average Optimizer Time [sec]', weight="bold"
# for i, v in enumerate(time_means):
#     axs[0,0].text(-width1/4, float(v) , round(average_time_op, 2), color = 'grey', weight="bold")

# Fist plot (line plot for optimizer and path compute time) 
list_average_time_gp = [average_time_gp] * len(time_gp)
list_average_time_op = [average_time_op] * len(time_optimizer)

# Fist Plot Line (all plots Together)
# lns1_1 = axs[0,0].plot(count_all_T, time_gp, '-', color='red', label = 'GP')
# lns1_2 = axs[0,0].plot(count_all_T, list_average_time_gp, '-', color='magenta', label='mean GP ={:.1f}'.format(np.mean(average_time_gp)))
# lns2_1 = axs[0,0].plot(count_all, time_optimizer, '-', color='blue', label = 'Opt')
# lns2_2 = axs[0,0].plot(count_all, list_average_time_op, '-', color='cyan', label = 'mean Opt ={:.1f}'.format(np.mean(average_time_op)))
# lns1 = lns1_1+lns1_2+lns2_1+lns2_2    #For label
# labs1 = [l.get_label() for l in lns1]   #For label
# axs[0,0].legend(lns1, labs1, loc=0)    #For label
# axs[0,0].grid(color='pink')
# axs[0,0].set_xlabel('Test Number', fontsize=12)
# axs[0,0].set_ylabel('Compute Time[sec]', fontsize=12)

lns1_1 = axs[0,0].plot(count_all_T, time_gp, '-', color='red', label = 'GP')
lns1_2 = axs[0,0].plot(count_all_T, list_average_time_gp, '-', color='magenta', label='mean GP ={:.1f}'.format(np.mean(average_time_gp)))
axs2 = axs[0,0].twinx()
lns2_1 = axs2.plot(count_all, time_optimizer, '-', color='blue', label = 'Opt')
lns2_2 = axs2.plot(count_all, list_average_time_op, '-', color='cyan', label = 'mean Opt ={:.1f}'.format(np.mean(average_time_op)))
lns1 = lns1_2+lns2_2    #For label
labs2 = [l.get_label() for l in lns1]   #For label
axs[0,0].legend(lns1, labs2, loc=0)    #For label
axs[0,0].set_ylim(ymin=(min_value_time_gp-min_value_time_gp*0.2), ymax=(max_value_time_gp+min_value_time_gp*0.2))
axs[0,0].tick_params(axis='y', labelcolor='red')
axs[0,0].grid(color='pink')
axs[0,0].set_ylabel('Global Planner Time[sec]',color="red", fontsize=12)
axs[0,0].set_xlabel('Test Number', fontsize=12)
axs2.tick_params(axis='y', labelcolor='blue')
axs2.grid(color='gray')
axs2.set_ylabel("Optimization Time[sec]",color="blue",fontsize=12)
axs2.set_ylim(min_value_time_op-min_value_time_op*0.1, max_value_time_op+min_value_time_op*0.1)
# axs2.set_title('Time Analysis[sec]', weight="bold")

# Second Bar Plot
# collisions = [count_ugv_coll, count_uav_coll, count_cat_coll]
# x = np.arange(len(collisions))
# bar_width2 = 1.4
# for i in range(len(collisions)):
#     b = axs[1,0].bar(bar_width2/2 + bar_width2*2*i, float(collisions[i]), bar_width2,  color='red' , bottom=0, label=['UGV','UAV','Cat'])
#     axs[1,0].text(bar_width2/4 + bar_width2*2*i, collisions[i] , int(collisions[i]), color = 'grey', weight="bold")
# axs[1,0].set_title(' Fail Trajectory Number for collision', weight="bold")
# axs[1,0].set_ylabel('Collision Number')
# axs[1,0].set_xticks(bar_width2/2 + bar_width2*2*x )
# axs[1,0].set_xticklabels(('UGV' , 'UAV', 'Cat'))

# Second Line Plot 
lns3_1 = axs[1,0].plot(count_all_T, cost_init_sol, '-', color='red', label = 'GP')
lns4_1 = axs[1,0].plot(count_all_T, cost_opt_sol, '-', color='blue', label = 'Opt')
lns2 = lns3_1+lns4_1
labs2 = [l.get_label() for l in lns2]
axs[1,0].legend(lns2, labs2, loc=0)
axs[1,0].grid(color='pink')
axs[1,0].set_ylabel('Costs Proccess', fontsize=12)
lns2 = lns3_1+lns4_1
labs2 = [l.get_label() for l in lns2]
axs[1,0].legend(lns2, labs2, loc=0)
axs[1,0].grid(color='pink')
axs[1,0].set_ylabel('Costs Proccess', fontsize=12)
axs[1,0].set_xlabel('Test Number', fontsize=12)



# Some data
labels1 = ["Feasibility", "Not Feasibility"]
perc1 = [percentage_feasibility, 100-percentage_feasibility]
value1 = [count_feasibility, count-count_feasibility]
labels2 = 'Collision', 'Not Collision'
perc2 = [percentage_ugv_coll, 100-percentage_ugv_coll]
value2 = [count_ugv_coll, count-count_ugv_coll]
labels3 = 'Collision', 'Not Collision'
perc3 = [percentage_uav_coll, 100-percentage_uav_coll]
value3 = [count_uav_coll, count-count_uav_coll]
labels4 = 'Collision', 'Not Collision'
perc4 = [percentage_cat_coll, 100-percentage_cat_coll]
value4 = [count_cat_coll, count-count_cat_coll]

# First Pie Plot
# axs[0, 1].pie(perc1, labels=labels1, radius=0.5, frame=True)
axs[0, 1].pie(perc1, radius=0.5, frame=True)
axs[0, 1].set_title("Feasibility Trajectory",fontsize=14, weight="bold")

# Second Pie Plot
# axs[1, 1].pie(perc2, labels=labels2)
axs[1, 1].pie(perc2)
axs[1, 1].set_title("Collision Trajectory UGV",fontsize=14, weight="bold")

# Third Pie Plot
# axs[0, 2].pie(perc3, labels=labels3)
axs[0, 2].pie(perc3)
axs[0, 2].set_title("Collision Trajectory UAV",fontsize=14, weight="bold")

# Fourth Pie Plot
# axs[1, 2].pie(perc4, labels=labels4)
axs[1, 2].pie(perc4)
axs[1, 2].set_title("Collision Trajectory Catenary",fontsize=14, weight="bold")

def func(pct, allvals):
    absolute = int(round(pct/100.*np.sum(allvals)))
    return "{:.1f}%\n({:d})".format(pct, absolute)

wedges, texts, autotexts = axs[0, 1].pie(value1, autopct=lambda pct: func(pct, value1), textprops=dict(color="w"), colors=['g', 'r'], wedgeprops = { 'linewidth' : 2, 'edgecolor' : 'white' })
axs[0,1].legend(wedges, labels1, title="Status", loc="best", bbox_to_anchor=(1, 0, 0.5, 1))

wedges, texts, autotexts = axs[1, 1].pie(value2, autopct=lambda pct: func(pct, value2), textprops=dict(color="w"), colors=['r', 'g'], wedgeprops = { 'linewidth' : 2, 'edgecolor' : 'white' })
axs[1,1].legend(wedges, labels2, title="Status", loc="best", bbox_to_anchor=(1, 0, 0.5, 1))

wedges, texts, autotexts = axs[0, 2].pie(value3, autopct=lambda pct: func(pct, value3), textprops=dict(color="w"), colors=['r', 'g'], wedgeprops = { 'linewidth' : 2, 'edgecolor' : 'white' })
axs[0,2].legend(wedges, labels3, title="Status", loc="best", bbox_to_anchor=(1, 0, 0.5, 1))

wedges, texts, autotexts = axs[1, 2].pie(value4, autopct=lambda pct: func(pct, value4), textprops=dict(color="w"), colors=['r', 'g'], wedgeprops = { 'linewidth' : 2, 'edgecolor' : 'white' })
axs[1,2].legend(wedges, labels4, title="Status", loc="best", bbox_to_anchor=(1, 0, 0.5, 1))

autotexts[0].set_color('white')
plt.subplots_adjust(left=0.06, bottom=0.08, right=0.98, top=0.92) 
plt.show()

