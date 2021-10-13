#!/usr/bin/env python2

# -*- coding: utf-8 -*-

import numpy as np
import os
import csv
import matplotlib.pyplot as plt
import math
import time

## Get Data and compute percentage related with Trajectory Optimized
#Open and managing files and data
path = "/home/simon/results_marsupial_optimizer/"
file_path = path + 'feasibility_analisys_for_trajectory.txt'

# q_ = "e"

# while q_ != "q":
count = 0
count_feasibility = 0
count_ugv_coll = 0
count_uav_coll = 0
count_cat_coll = 0
count_time = 0.0

file = open(file_path, 'r')
Tlines = file.read().splitlines()
Tlines.pop(0) # To remove first line in case to have titles
        
Tl_col_00 = []

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
        count_time += float(line[11])
    count+=1.0
        
percentage_feasibility = (count_feasibility * 100.0)/count
percentage_ugv_coll = (count_ugv_coll * 100.0)/count
percentage_uav_coll = (count_uav_coll * 100.0)/count
percentage_cat_coll = (count_cat_coll * 100.0)/count
average_time = count_time/float(count)

print("Percentage_feasibility: "+ str(percentage_feasibility))
print("Percentage_ugv_coll: " + str(percentage_ugv_coll))
print("Percentage_uav_coll: " + str(percentage_uav_coll))
print("Percentage_cat_coll: " + str(percentage_cat_coll))    
print("Total number of test: " + str(count))

## Get Plot DataTrajectory Optimized

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

# Make figure and axes
fig, axs = plt.subplots(2, 3, figsize=(12, 9), subplot_kw=dict(aspect="equal"))
fig.suptitle('Result factibility and collision trajectory optimized (test_number ='+str(count)+")", fontsize=21, weight="bold")

X = np.arange(0, math.pi*2, 0.05)
Y = np.cos(X)

# Fist bar plot
labels5 = ['Average']
time_means = [average_time]
width1 = 120  # the width of the bars
rects1 = axs[0,0].bar(labels5, time_means, width1)
axs[0,0].set_ylabel('Time[sec]')
axs[0,0].set_title('Average Optimizer Time [sec]', weight="bold")

for i, v in enumerate(time_means):
    axs[0,0].text(-width1/4, float(v) , round(average_time, 2), color = 'grey', weight="bold")



# Second bar plot
labels6 = ['UGV Collision' , 'UAV Collision', 'Cat Collision']
collisions = [count_ugv_coll, count_uav_coll, count_cat_coll]
x = np.arange(len(collisions))
bar_width2 = 1.4
for i in range(len(collisions)):
    b = axs[1,0].bar(bar_width2/2 + bar_width2*2*i, float(collisions[i]), bar_width2, bottom=0, label=['UGV','UAV','Cat'])
    axs[1,0].text(bar_width2/4 + bar_width2*2*i, collisions[i] , int(collisions[i]), color = 'grey', weight="bold")
axs[1,0].set_title(' Fail Trajectory Number for collision', weight="bold")
axs[1,0].set_ylabel('Collision Number')
axs[1,0].set_xticks(bar_width2/2 + bar_width2*2*x )
axs[1,0].set_xticklabels(('UGV' , 'UAV', 'Cat'))


# First pie plot
axs[0, 1].pie(perc1, labels=labels1)
axs[0, 1].set_title("Percentage Feasibility Trajectory",fontsize=14, weight="bold")

# Second pie plot
axs[1, 1].pie(perc2, labels=labels2)
axs[1, 1].set_title("Percentage Collision UGV in Trajectory",fontsize=14, weight="bold")

# Third pie plot
axs[0, 2].pie(perc3, labels=labels3)
axs[0, 2].set_title("Percentage Collision UAV in Trajectory",fontsize=14, weight="bold")

# Fourth pie plot
axs[1, 2].pie(perc4, labels=labels4)
axs[1, 2].set_title("Percentage Collision Catenary in Trajectory",fontsize=14, weight="bold")

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

plt.show()

    # plt.ion()
    # plt.show(block='False')
    
    # # Here to update plot after 1 min
    # count_t = 0
    # if count_t < 60:
    #     time.sleep(1)
    #     count_t += 1
    #     localtime = time.localtime()
    #     result = time.strftime("%I:%M:%S %p", localtime)
    #     print(result)
    #     print ("count_t= ",count_t)

    # plt.close()

