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
# extra_path_ = "Opt_Params_Catenary/"
extra_path_ = ""

num_parameters = 11
stage_num = arg1 
initial_pos = arg2 

file_path_ugv = '/home/' + user_ + '/results_marsupial_optimizer/'+extra_path_+'results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UGV.txt'
file_path_uav = '/home/' + user_ + '/results_marsupial_optimizer/'+extra_path_+'results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UAV.txt'
file_path = '/home/' + user_ + '/results_marsupial_optimizer/'+extra_path_+'results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_feasibility_trajectory.txt'
print('Feasibility Data in file: ',file_path) 
print('UGV Data in file: ',file_path_ugv) 
print('UAV Data in file: ',file_path_uav) 

matrix1=[] #values for UGV
matrix2=[] #values for UAV
sM1=[] # sum values in matrix to get average UGV
sM2=[] # sum values in matrix to get average UAV
file_ugv = open(file_path_ugv, 'r')
lines_ugv = file_ugv.read().splitlines() 
lines_ugv.pop(0) # To remover first row in case have titles 
file_uav = open(file_path_uav, 'r')
lines_uav = file_uav.read().splitlines() 
lines_uav.pop(0) # To remover first row in case have titles 
file = open(file_path, 'r')
lines = file.read().splitlines() 
lines.pop(0) # To remover first row in case have titles 

# Create Matrix to save in rows the values of each experiment
max_TCGP = 0
max_TCO = 0
i = 0
for l in lines_ugv: # row
    line = l.split(';') 
    matrix1.append([])
    sM1.append([])
    # print('i=',i)
    for j in range(num_parameters): # column
        # print('j:',j,'  line[j]:',line[j])
        matrix1[i].append(float(line[j]))
        if i== 0:
            value = matrix1[i][j]
        else:
            value = sM1[i-1][j] + matrix1[i][j]
        if (j == 0 and max_TCGP < matrix1[i][j]): # To get Maximum TCGP value
            max_TCGP = matrix1[i][j]
        if (j == 1 and max_TCO < matrix1[i][j]): # To get Maximum TCO value
            max_TCO = matrix1[i][j]

        sM1[i].append(value)
        # print('i:',i,' j:',j,' matrix1[i][j]:',matrix1[i][j],' value:',value, ' sM1[i-1][j]:',sM1[i-1][j])
    i+= 1
i = 0
for l in lines_uav: # row
    line = l.split(';') 
    matrix2.append([])
    sM2.append([])
    # print('i=',i)
    for j in range(num_parameters): # column
        # print(' line[j]:',line[j],' j:',j)
        matrix2[i].append(float(line[j]))
        if i== 0:
            value = matrix2[i][j]
        else:
            value = sM2[i-1][j] + matrix2[i][j]
        sM2[i].append(value)
        # print('i:',i,' j:',j,' matrix2[i][j]:',matrix2[i][j],' value:',value, ' sM2[i-1][j]:',sM2[i-1][j])
    i+= 1

sum_feasibility = 0
total_exp = 0
for l in lines: # row
    line = l.split(';') 
    sum_feasibility = int(line[4]) + sum_feasibility
    total_exp += 1
total_feasibility = 100*sum_feasibility / (total_exp)    

NE = len(matrix2) # number of Experiments
length_ = NE - 1
# NEXT LINEs FOR CASE OF DEBUG
# length_ugv = len(matrix1)
# length_uav = len(matrix2)
# if length_ugv > 0:
#     num_col_ugv = len(matrix1[0]) 
# else:
#     num_col_ugv = 0
# if length_uav > 0:
#     num_col_uav = len(matrix2[0]) 
# else:
#     num_col_uav = 0
# print('UGV fila=',length_ugv, ' colum=',num_col_ugv)
# print('UAV fila=',length_uav, ' colum=',num_col_uav)


# output_file1 = open('/home/' + user_ + '/results_marsupial_optimizer/results_management_feasibility_time.cvs', 'a') # If doesn't exist file, is create and write in the last row
# # Order Colums: (From "sM1 or sM2" UGV-UAV)mean_vTI;max_vTI;mean_vTO;max_vTO;mean_aTI;max_aTI;mean_aTO;max_aTO;
# output_file1.writelines(str(total_feasibility)+';'+str(sM1[length_][0]/NE)+';'+str(sM1[length_][1]/NE)+';'+ str(sM1[length_][7]/NE)+';'+str(sM1[length_][8]/NE)+';'+str(sM1[length_][9]/NE)+';'+str(sM1[length_][10]/NE)+';'+str(sM2[length_][7]/NE)+';'+str(sM2[length_][8]/NE)+';'+str(sM2[length_][9]/NE)+';'+str(sM2[length_][10]/NE)+'\n')
# output_file1.close()

# output_file2 = open('/home/' + user_ + '/results_marsupial_optimizer/results_management_ugv_uav.cvs', 'a') # If doesn't exist file, is create and write in the last row
# # Order Colums: ("From sM1 or sM2" UGV-UAV)tTI;tTO; (UGV From"sM1":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (UAV From "sM2":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (CAT From "sM1 or sM2":10-13)mean_dcI;min_dcI;mean_dcO;min_dcO;
# output_file2.writelines(str(sM1[length_][1]/NE)+';'+str(sM1[length_][5]/NE)+';'+str(sM1[length_][2]/NE)+';'+str(sM1[length_][3]/NE)+';'+str(sM1[length_][6]/NE)+';'+str(sM1[length_][7]/NE)+';'+str(sM1[length_][8]/NE)+';'+str(sM1[length_][9]/NE)+';'+str(sM2[length_][2]/NE)+';'+str(sM2[length_][3]/NE)+';'+str(sM2[length_][6]/NE)+';'+str(sM2[length_][7]/NE)+';'+str(sM2[length_][8]/NE)+';'+str(sM2[length_][9]/NE)+';'+str(sM1[length_][10]/NE)+';'+str(sM1[length_][11]/NE)+';'+str(sM1[length_][12]/NE)+';'+str(sM1[length_][13]/NE)+'\n')
# output_file2.close()

# output_file3 = open('/home/' + user_ + '/results_marsupial_optimizer/'+extra_path_+'results_maximum_compute_time.cvs', 'a') # If doesn't exist file, is create and write in the last row
# output_file3.writelines(str(max_TCGP)+';'+str(max_TCO)+'\n')
# output_file3.close()

output_file3 = open('/home/' + user_ + '/results_marsupial_optimizer/'+extra_path_+'results_for_paper.cvs', 'a') # If doesn't exist file, is create and write in the last row
output_file3.writelines(str(total_feasibility)+';'+str(sM1[length_][1]/NE)+';'+str(sM1[length_][5]/NE)+';'+str(sM1[length_][6]/NE)+';'+str(sM1[length_][3]/NE)+';'+str(sM1[length_][4]/NE)+';'+str(sM1[length_][7]/NE)+';'+str(sM1[length_][8]/NE)+';'+str(sM1[length_][9]/NE)+';'+str(sM1[length_][10]/NE)+';'+str(sM2[length_][3]/NE)+';'+str(sM2[length_][4]/NE)+';'+str(sM2[length_][7]/NE)+';'+str(sM2[length_][8]/NE)+';'+str(sM2[length_][9]/NE)+';'+str(sM2[length_][10]/NE)+'\n')
output_file3.close()
