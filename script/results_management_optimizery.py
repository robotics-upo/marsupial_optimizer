#!/usr/bin/env python3

# -*- coding: utf-8 -*-

#Apertura de archivos CSV y gestion de datos

import numpy as np
import os
import csv

num_execute_scenario = 10
total_row = 1

# path = "/home/simon/"
# file_path = path + 'time_compute_initial_planner.txt'
# Tcol_00 = 0
# file = open(file_path, 'r')
# Tlines = file.read().splitlines() #crea una lista
# #lines.pop(0) #quitamos primera linea en caso de tener titulos 
# #print(lines)
# Tl_col_00 = []
# for l in Tlines:
#     line = l.split(';') #devuelve igualmente una lista
#     Tl_col_00.append(float(line[Tcol_00]))
#     # TvCol01= float(line[Tcol_00])

num_exp= 100.0
user_ = 'simon'
stage_num = '1'
initial_pos = '1'
file_path_ugv = '/home/' + user_ + '/results_marsupial_optimizer/results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UGV.txt'
file_path_uav = '/home/' + user_ + '/results_marsupial_optimizer/results_stage_'+ stage_num +'_InitPos_'+ initial_pos + '_method_UAV.txt'
print('Magement Data For: ',file_path_ugv)
col_00 = 0
col_01 = 1
col_02 = 2
col_03 = 3
col_04 = 4
col_05 = 5
col_06 = 6
col_07 = 7
col_08 = 8
col_09 = 9 
col_10 = 10
col_11 = 11
col_12 = 12
col_13 = 13
col_14 = 14
col_15 = 15
col_16 = 16
col_17 = 17
col_18 = 18
col_19 = 19
col_20 = 20
col_21 = 21
# col_22 = 22
# col_23 = 23
# col_24 = 24
# col_25 = 25

l_col_00 = []
l_col_01 = []
l_col_02 = []
l_col_03 = []
l_col_04 = []
l_col_05 = []
l_col_06 = []
l_col_07 = []
l_col_08 = []
l_col_09 = [] 
l_col_10 = []
l_col_11 = []
l_col_12 = []
l_col_13 = []
l_col_14 = []
l_col_15 = []
l_col_16 = []
l_col_17 = []
l_col_18 = []
l_col_19 = []
l_col_20 = []
l_col_21 = []
# l_col_22 = []
# l_col_23 = []
# l_col_24 = []
# l_col_25 = []

# Save colums values sum
file = open(file_path_ugv, 'r')
lines = file.read().splitlines() 
lines.pop(0) #quitamos primera linea en caso de tener titulos 
count = 0
for l in lines:
    line = l.split(';') 
    l_col_00.append(float(line[col_00]))
    l_col_01.append(float(line[col_01]))
    l_col_02.append(float(line[col_02]))
    l_col_03.append(float(line[col_03]))
    l_col_04.append(float(line[col_04]))
    l_col_05.append(float(line[col_05]))
    l_col_06.append(float(line[col_06]))
    l_col_07.append(float(line[col_07]))
    l_col_08.append(float(line[col_08]))
    l_col_09.append(float(line[col_09]))
    l_col_10.append(float(line[col_10]))
    l_col_11.append(float(line[col_11]))
    l_col_12.append(float(line[col_12]))
    l_col_13.append(float(line[col_13]))
    l_col_14.append(float(line[col_14]))
    l_col_15.append(float(line[col_15]))
    l_col_16.append(float(line[col_16]))
    l_col_17.append(float(line[col_17]))
    l_col_18.append(float(line[col_18]))
    l_col_19.append(float(line[col_19]))
    l_col_20.append(float(line[col_20]))
    l_col_21.append(float(line[col_21]))
    # l_col_22.append(float(line[col_22]))
    # l_col_23.append(float(line[col_23]))
    # l_col_24.append(float(line[col_24]))
    # l_col_25.append(float(line[col_25]))
    # print('l_col_00:',line[col_00])

# Initialize value to save colums values sum
vCol00= 0.0
vCol01= 0.0
vCol02= 0.0
vCol03= 0.0
vCol04= 0.0
vCol05= 0.0
vCol06= 0.0
vCol07= 0.0
vCol08= 0.0
vCol09= 0.0
vCol10= 0.0
vCol11= 0.0
vCol12= 0.0
vCol13= 0.0
vCol14= 0.0
vCol15= 0.0
vCol16= 0.0
vCol17= 0.0
vCol18= 0.0
vCol19= 0.0
vCol20= 0.0
vCol21= 0.0
# vCol22= 0.0
# vCol23= 0.0
# vCol24= 0.0
# vCol25= 0.0

output_file = open('/home/' + user_ + '/results_marsupial_optimizer/results.cvs', 'a') #Si no existia el archivo, lo crea y escribe al final
count = 0
for i in range(0, total_row):
    #Anadir resultados, segun columnas: nombre de test, tiempo medio de tes ...
    output_file.writelines(str(l_col_00[i]) + ';' + str(l_col_01[i]) + ';' + str(l_col_02[i]) + ';' + str(l_col_03[i]) + ';' + str(l_col_04[i]) + ';' + str(l_col_05[i]) + ';' + str(l_col_06[i]) + ';' + str(l_col_07[i]) + ';' + str(l_col_08[i]) + ';' + str(l_col_09[i]) + ';' + str(l_col_10[i]) + ';' + str(l_col_11[i]) + ';' + str(l_col_12[i]) + ';' + str(l_col_13[i]) + ';' + str(l_col_14[i]) + ';' + str(l_col_15[i]) + ';' + str(l_col_16[i]) + ';' + str(l_col_17[i]) + ';' + str(l_col_18[i]) + ';' + str(l_col_19[i]) + ';' + str(l_col_20[i]) + ';' + str(l_col_21[i]) + '\n')
    vCol00 = vCol00 + float(l_col_00[i])
    vCol01 = vCol01 + float(l_col_01[i])
    vCol02 = vCol02 + float(l_col_02[i])
    vCol03 = vCol03 + float(l_col_03[i])
    vCol04 = vCol04 + float(l_col_04[i])
    vCol05 = vCol05 + float(l_col_05[i])
    vCol06 = vCol06 + float(l_col_06[i])
    vCol07 = vCol07 + float(l_col_07[i])
    vCol08 = vCol08 + float(l_col_08[i])
    vCol09 = vCol09 + float(l_col_09[i])
    vCol10 = vCol10 + float(l_col_10[i])
    vCol11 = vCol11 + float(l_col_11[i])
    vCol12 = vCol12 + float(l_col_12[i])
    vCol13 = vCol13 + float(l_col_13[i])
    vCol14 = vCol14 + float(l_col_14[i])
    vCol15 = vCol15 + float(l_col_15[i])
    vCol16 = vCol16 + float(l_col_16[i])
    vCol17 = vCol17 + float(l_col_17[i])
    vCol18 = vCol18 + float(l_col_18[i])
    vCol19 = vCol19 + float(l_col_19[i])
    vCol20 = vCol20 + float(l_col_20[i])
    vCol21 = vCol21 + float(l_col_21[i])
    # vCol22 = vCol22 + float(l_col_22[i])
    # vCol23 = vCol23 + float(l_col_23[i])
    # vCol24 = vCol24 + float(l_col_24[i]) 
    # vCol25 = vCol25 + float(l_col_25[i]) 
    
    count+= 1
    if count % num_execute_scenario == 0 :
        output_file.writelines( str(TvCol00/num_exp) + ';' +  str(vCol00/num_exp) + ';' +  str(vCol01/num_exp) + ';' +  str(vCol02/num_exp) + ';' +  str(vCol03/num_exp) + ';' +  str(vCol04/num_exp) + ';' +  str(vCol05/num_exp) + ';' +  str(vCol06/num_exp) + ';' +  str(vCol07/num_exp) + ';' +  str(vCol08/num_exp) + ';' +  str(vCol09/num_exp) + ';' +  str(vCol10/num_exp) + ';' +  str(vCol11/num_exp) + ';' +  str(vCol12/num_exp) + ';' +  str(vCol13/num_exp) + ';' +  str(vCol14/num_exp) + ';' +  str(vCol15/num_exp) + ';' +  str(vCol16/num_exp) + ';' +  str(vCol17/num_exp) + ';' +  str(vCol18/num_exp) + ';' +  str(vCol19/num_exp) + ';' +  str(vCol20/num_exp) + ';' +  str(vCol21/num_exp) + ';' +  str(vCol22/num_exp) + ';' +  str(vCol23/num_exp) + ';' +  str(vCol24/num_exp) + '\n')
        TvCol00 = 0.0
        vCol00= 0.0
        vCol01= 0.0
        vCol02= 0.0
        vCol03= 0.0
        vCol04= 0.0
        vCol05= 0.0
        vCol06= 0.0
        vCol07= 0.0
        vCol08= 0.0
        vCol09= 0.0
        vCol10= 0.0
        vCol11= 0.0
        vCol12= 0.0
        vCol13= 0.0
        vCol14= 0.0
        vCol15= 0.0
        vCol16= 0.0
        vCol17= 0.0
        vCol18= 0.0
        vCol19= 0.0
        vCol20= 0.0
        vCol21= 0.0
        # vCol22= 0.0
        # vCol23= 0.0
        # vCol24= 0.0

output_file.close()

