#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import os
import csv
import sys

print("\n\t *** IMPORTANT: Use this script given two parameters: <scenario_number> <initial_position_number> <mode UGV or UAV>***\n")  # Warnning message
arg1 = sys.argv[1] # number of scenario to analize
arg2 = sys.argv[2] # number of position to analize

user_ = 'simon' # Change this value to the user name

NE = 100.0 # number of Experiments
num_parameters = 22
stage_num = arg1 
initial_pos = arg2 

file_path_ugv = '/home/' + user_ + '/results_marsupial_optimizer/results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UGV.txt'
file_path_uav = '/home/' + user_ + '/results_marsupial_optimizer/results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UAV.txt'
print('Magement Data For: ',file_path_ugv)

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

# Create Matrix to save in rows the values of each experiment
i = 0
for l in lines_ugv: # row
    line = l.split(';') 
    matrix1.append([])
    sM1.append([])
    # print('i=',i)
    for j in range(num_parameters): # column
        # print(' line[j]:',line[j],' j:',j)
        matrix1[i].append(float(line[j]))
        if i== 0:
            value = matrix1[i][j]
        else:
            value = sM1[i-1][j] + matrix1[i][j]
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


output_file = open('/home/' + user_ + '/results_marsupial_optimizer/results_ugv_uav.cvs', 'a') # If doesn't exist file, is create and write in the last row
# Order Colums: ("From sM1 or sM2" UGV-UAV)tTI;tTO; (UGV From"sM1":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (UAV From "sM2":2-3,6-9)dTI;dTO;mean_doI;min_doI;mean_doO;min_doO; (CAT From "sM1 or sM2":10-13)mean_dcI;min_dcI;mean_dcO;min_dcO;
output_file.writelines(str(sM1[99][4]/NE)+';'+str(sM1[99][5]/NE)+';'+str(sM1[99][2]/NE)+';'+str(sM1[99][3]/NE)+';'+str(sM1[99][6]/NE)+';'+str(sM1[99][7]/NE)+';'+str(sM1[99][8]/NE)+';'+str(sM1[99][9]/NE)+';'+str(sM2[99][2]/NE)+';'+str(sM2[99][3]/NE)+';'+str(sM2[99][6]/NE)+';'+str(sM2[99][7]/NE)+';'+str(sM2[99][8]/NE)+';'+str(sM2[99][9]/NE)+';'+str(sM1[99][10]/NE)+';'+str(sM1[99][11]/NE)+';'+str(sM1[99][12]/NE)+';'+str(sM1[99][13]/NE)+'\n')

output_file.close()