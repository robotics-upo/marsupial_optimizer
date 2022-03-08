#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import os
import csv
import sys

print("\n\t *** IMPORTANT: Use this script given two parameters: <pc_user_name> <scenario_number> <initial_position_number> ***\n")  # Warnning message
pc_user = sys.argv[1] # number of position to analize
arg1 = sys.argv[2] # number of position to analize
arg2 = sys.argv[3] # number of position to analize

# user_ = 'simon' # Change this value to the user name
user_= pc_user # number of scenario to analize

NE = 100.0 # number of Experiments
num_param_method = 6
num_param_solutions = 2
stage_num = arg1 
initial_pos = arg2 
file_path_methods = '/home/' + user_ + '/results_marsupial_optimizer/rrt_time_methods_analisys_scenario_'+ stage_num +'_num_pos_initial_'+ initial_pos + '.txt'
file_path_solutions = '/home/' + user_ + '/results_marsupial_optimizer/rrt_time_solutions_analisys_scenario_'+ stage_num +'_num_pos_initial_'+ initial_pos + '.txt'
print('Magement Data For: ',file_path_methods) 

matrix1=[] #values for methods
matrix2=[] #values for solutions
sM1=[] # sum values in matrix to get average methods
sM2=[] # sum values in matrix to get average solutions
file_methods = open(file_path_methods, 'r')
lines_methods = file_methods.read().splitlines() 
lines_methods.pop(0) # To remover first row in case have titles 
file_results = open(file_path_solutions, 'r')
lines_results = file_results.read().splitlines() 
lines_results.pop(0) # To remover first row in case have titles 

# Create Matrix to save in rows the acumulative values of each method and solution to compute the average in the end
i = 0
for l in lines_methods: # row
    line = l.split(';') 
    matrix1.append([])
    sM1.append([])
    # print('i=',i)
    for j in range(num_param_method): # column
        # print(' line[j]:',line[j],' j:',j)
        matrix1[i].append(float(line[j]))
        if i== 0:
            value = matrix1[i][j]
        else:
            value = sM1[i-1][j] + matrix1[i][j]

        sM1[i].append(value)
        # print('i:',i,' j:',j,' matrix1[i][j]:',matrix1[i][j],' value:',value, ' sM1[i-1][j]:',sM1[i-1][j])
    i+= 1
print('value of i=',i)
k = 0
for l in lines_results: # row
    line = l.split(';') 
    matrix2.append([])
    sM2.append([])
    # print('k=',k)
    for j in range(num_param_solutions): # column
        # print(' line[j]:',line[j],' j:',j)
        matrix2[k].append(float(line[j]))
        if k== 0:
            value = matrix2[k][j]
        else:
            value = sM2[k-1][j] + matrix2[k][j]
        sM2[k].append(value)
        # print('k:',k,' j:',j,' matrix2[k][j]:',matrix2[k][j],' value:',value, ' sM2[k-1][j]:',sM2[k-1][j])
    k+= 1
print('value of k=',k)

output_file1 = open('/home/' + user_ + '/results_marsupial_optimizer/results_management_time_RRT_methods.cvs', 'a') # If doesn't exist file, is create and write in the last row
# Order Colums: (From "sM1 or sM2" UGV-UAV)mean_vTI;max_vTI;mean_vTO;max_vTO;mean_aTI;max_aTI;mean_aTO;max_aTO;
output_file1.writelines(str(sM1[i-1][0]/i)+';'+str(sM1[i-1][1]/i)+';'+ str(sM1[i-1][2]/i)+';'+str(sM1[i-1][3]/i)+';'+str(sM1[i-1][4]/i)+';'+str(sM1[i-1][5]/i)+'\n')
output_file1.close()

output_file2 = open('/home/' + user_ + '/results_marsupial_optimizer/results_management_time_RRT_solutions.cvs', 'a') # If doesn't exist file, is create and write in the last row
# Order Colums: ("From sM1 or sM2" UGV-UAV)tTI;tTO; (UGV From"sM1":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (UAV From "sM2":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (CAT From "sM1 or sM2":10-13)mean_dcI;min_dcI;mean_dcO;min_dcO;
output_file2.writelines(str(sM2[k-1][0]/k)+';'+str(sM2[k-1][1]/k)+'\n')
output_file2.close()

