# -*- coding: utf-8 -*-

#This code calculate the Standar Error of the Mean (SEM) for two rows (4 and 5) using the numpy lib, and also
# graph the average of the cost and de SEM for 3 different problems, which each one has 4 algorithms (RRT,tRRT,biRRT and tbiRRT)


import numpy as np
import matplotlib.pyplot as plt
# import seaborn as sns


# import os
#iterate files in directory
file_path = '/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/results_test.csv' 

#number of tests per file
total_rows = 200

#column numbers for algorithms

path_cost_col = 4
SEM_cost_col = 5

test_time_col = 2
SEM_test_time = 6

path_time_col = 3
SEM_path_time = 7


file = open(file_path, 'r')
lines = file.read().splitlines() #To create a list
lines.pop(0) #we remove the first line (used in case to remove the titule)

cost_list = [] 
SEM_list = []  

for l in lines:
        line = l.split(',') #return list
        cost_list.append(float(line[path_cost_col]))                  #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS
        SEM_list.append(float(line[SEM_cost_col]))                    #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS
        # cost_list.append(float(line[test_time_col]))                  #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS
        # SEM_list.append(float(line[SEM_test_time]))                    #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS
        # cost_list.append(float(line[path_time_col]))                  #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS
        # SEM_list.append(float(line[SEM_path_time]))                    #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS

RRT_cost_list = []
tRRT_cost_list = []
biRRT_cost_list = []
tbiRRT_cost_list = []
RRT_SEM_list = []
tRRT_SEM_list = []
biRRT_SEM_list = []
tbiRRT_SEM_list = []





# # This part is to considerate to graph only the information of the FIRST three experiments (1, 2 and 3)
# # Is cheking in CSV every 4 lines to take only the RRT values

for i in range (1,24,4):                                                #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS OF KIND PROBLEM
        RRT_cost_list.append(cost_list[i])        
        RRT_SEM_list.append(SEM_list[i])      

for i in range (3,24,4):
        tRRT_cost_list.append(cost_list[i])        
        tRRT_SEM_list.append(SEM_list[i]) 

for i in range (0,24,4):
        biRRT_cost_list.append(cost_list[i])        
        biRRT_SEM_list.append(SEM_list[i])   

for i in range (2,24,4):
        tbiRRT_cost_list.append(cost_list[i])        
        tbiRRT_SEM_list.append(SEM_list[i])    


# This part is to considerate to graph only the information of the SENCOND three experiments (4, 5 and 6)
# Is cheking in CSV every 4 lines to take only the RRT values

# for i in range (13,24,4):                                               #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS OF KIND PROBLEM      
#         RRT_cost_list.append(cost_list[i])        
#         RRT_SEM_list.append(SEM_list[i])      

# for i in range (15,24,4):
#         tRRT_cost_list.append(cost_list[i])        
#         tRRT_SEM_list.append(SEM_list[i]) 

# for i in range (12,24,4):
#         biRRT_cost_list.append(cost_list[i])        
#         biRRT_SEM_list.append(SEM_list[i])   

# for i in range (14,24,4):
#         tbiRRT_cost_list.append(cost_list[i])        
#         tbiRRT_SEM_list.append(SEM_list[i])    





# #Convert from list to tuple, type neccessaty to to the graphs
RRT_cost_tuple = tuple(RRT_cost_list)          
tRRT_cost_tuple = tuple(tRRT_cost_list)
biRRT_cost_tuple = tuple(biRRT_cost_list)
tbiRRT_cost_tuple = tuple(tbiRRT_cost_list)
RRT_SEM_tuple = tuple(RRT_SEM_list)
tRRT_SEM_tuple = tuple(tRRT_SEM_list)
biRRT_SEM_tuple = tuple(biRRT_SEM_list)
tbiRRT_SEM_tuple = tuple(tbiRRT_SEM_list)



ind = np.arange(len(RRT_cost_tuple))  # the x locations for the groups
width = 0.20  # the width of the bars
fig, ax = plt.subplots()





##First argument neccessary to give the position of the bar in the grahp

RRT_cost = ax.bar(ind - 6*width/4, RRT_cost_tuple, width, yerr=RRT_SEM_tuple,                 #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS  
                  color='DodgerBlue', label='RRT cost')
tRRT_cost = ax.bar(ind - 2*width/4, tRRT_cost_tuple, width, yerr=tRRT_SEM_tuple,
                  color='IndianRed', label='tRRT cost')
biRRT_cost = ax.bar(ind + 2*width/4, biRRT_cost_tuple, width, yerr=biRRT_SEM_tuple,
                  color='Orange', label='biRRT cost')
tbiRRT_cost = ax.bar(ind + 6*width/4, tbiRRT_cost_tuple, width, yerr=tbiRRT_SEM_tuple,
                  color='SeaGreen', label='tbiRRT cost')

# RRT_cost = ax.bar(ind - 6*width/4, RRT_cost_tuple, width, yerr=RRT_SEM_tuple,                 #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS     
#                   color='DodgerBlue', label='RRT execution time')
# tRRT_cost = ax.bar(ind - 2*width/4, tRRT_cost_tuple, width, yerr=tRRT_SEM_tuple,
#                   color='IndianRed', label='tRRT execution time')
# biRRT_cost = ax.bar(ind + 2*width/4, biRRT_cost_tuple, width, yerr=biRRT_SEM_tuple,
#                   color='Orange', label='biRRT execution time')
# tbiRRT_cost = ax.bar(ind + 6*width/4, tbiRRT_cost_tuple, width, yerr=tbiRRT_SEM_tuple,
#                   color='SeaGreen', label='tbiRRT execution time')

# RRT_cost = ax.bar(ind - 6*width/4, RRT_cost_tuple, width,                                      #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS     
#                   color='DodgerBlue', label='RRT trajectory time')
# tRRT_cost = ax.bar(ind - 2*width/4, tRRT_cost_tuple, width, yerr=tRRT_SEM_tuple,
#                   color='IndianRed', label='tRRT trajectory time')
# biRRT_cost = ax.bar(ind + 2*width/4, biRRT_cost_tuple, width, yerr=biRRT_SEM_tuple,
#                   color='Orange', label='biRRT trajectory time')
# tbiRRT_cost = ax.bar(ind + 6*width/4, tbiRRT_cost_tuple, width, yerr=tbiRRT_SEM_tuple,
#                   color='SeaGreen', label='tbiRRT trajectory time')

# # Add some text for labels, title and custom x-axis tick labels, etc.
# ax.set_ylabel('Costs', labelpad = 28)                                                 
# ax.set_title('Costs for algorithms in each problem')                   
# ax.set_ylabel('Execution Time')                                             
# ax.set_title('Execution Time for algorithms in each problem')               
#ax.set_ylabel('Trajectory Time')                                             
#ax.set_title('Trajectory Time for algorithms in each problem')              

ax.set_xticks(ind)
ax.legend(loc=2, fontsize=24)

# ax.set_xticklabels(('Problem 1', 'Problem 2', 'Problem 3'))           #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS OF KIND PROBLEM
ax.set_xticklabels(('Problem 1', 'Problem 2', 'Problem 3','Problem 4', 'Problem 5', 'Problem 6'))           #UNCOMMENT HERE TO CHANGE GRAPH OPTIONS OF KIND PROBLEM


ax.errorbar(ind - 6*width/4+width/2, RRT_cost_tuple, yerr=RRT_SEM_tuple,
            fmt='.', alpha = 0.5, ecolor='b', elinewidth=3,  capthick=2, capsize= 10)

ax.errorbar(ind - 2*width/4+width/2, tRRT_cost_tuple, yerr=tRRT_SEM_tuple,
            fmt='.', ecolor='darkred',elinewidth=3,  capthick=2, capsize= 10)

ax.errorbar(ind + 2*width/4+width/2, biRRT_cost_tuple, yerr=biRRT_SEM_tuple,
            fmt='.', ecolor='darkorange',elinewidth=3,  capthick=2, capsize= 10)

ax.errorbar(ind + 6*width/4+width/2, tbiRRT_cost_tuple, yerr=tbiRRT_SEM_tuple,
            fmt='.', ecolor='darkgreen', elinewidth=3,  capthick=2, capsize= 10)




def autolabel(rects, xpos='center'):
#     """
#     Attach a text label above each bar in *rects*, displaying its height.

#     *xpos* indicates which side to place the text w.r.t. the center of
#     the bar. It can be one of the following {'center', 'right', 'left'}.
#     """

    xpos = xpos.lower()  # normalize the case of the parameter
    ha = {'center': 'center', 'right': 'left', 'left': 'right'}
    offset = {'center': 0.5, 'right': 0.57, 'left': 0.43}  # x_txt = x + w*off

    for rect in rects:
        height = rect.get_height()
        ax.text(rect.get_x() + rect.get_width()*offset[xpos], height -0.2e6,
                '{:.3e}'.format(height), ha=ha[xpos], va='top', size = 24, rotation = 90, weight= 'ultralight')
        for tick in ax.xaxis.get_major_ticks():                #to change the size of values in axi X
                tick.label.set_fontsize(24)
        for tick in ax.yaxis.get_major_ticks():                #to change the size of values in axi Y
                tick.label.set_fontsize(24)  
 

autolabel(RRT_cost, "center")
autolabel(tRRT_cost, "center")
autolabel(biRRT_cost, "center")
autolabel(tbiRRT_cost, "center")

# # rect parameter to define the margins of the graph
fig.tight_layout(pad = 0, w_pad = 0, h_pad = 0, rect = (0.0,0.001,0.99,0.99)) 


plt.show()





