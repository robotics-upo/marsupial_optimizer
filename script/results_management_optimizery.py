#!/usr/bin/env python2

# -*- coding: utf-8 -*-

#Apertura de archivos CSV y gestion de datos
import numpy as np
import os
import csv

num_execute_scenario = 10
total_row = 1

path = "/home/simon/"
file_path = path + 'time_compute_initial_planner.txt'
Tcolum_00 = 0

file = open(file_path, 'r')
Tlines = file.read().splitlines() #crea una lista
#lines.pop(0) #quitamos primera linea en caso de tener titulos 
#print(lines)
    
Tl_col_00 = []

for l in Tlines:
    line = l.split(';') #devuelve igualmente una lista
    Tl_col_00.append(float(line[Tcolum_00]))
    # TvCol01= float(line[Tcolum_00])

file_path = path + 'results_optimization_stage_6_InitPos_2_goal_0.txt'
colum_00 = 0
colum_01 = 1
colum_02 = 2
colum_03 = 3
colum_04 = 4
colum_05 = 5
colum_06 = 6
colum_07 = 7
colum_08 = 8
colum_09 = 9 
colum_10 = 10
colum_11 = 11
colum_12 = 12
colum_13 = 13
colum_14 = 14
colum_15 = 15
colum_16 = 16
colum_17 = 17
colum_18 = 18
colum_19 = 19
colum_20 = 20
colum_21 = 21
colum_22 = 22
colum_23 = 23
colum_24 = 24

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
l_col_22 = []
l_col_23 = []
l_col_24 = []

file = open(file_path, 'r')
lines = file.read().splitlines() 

count = 0

for l in lines:
    line = l.split(';') 

    l_col_00.append(float(line[colum_00]))
    l_col_01.append(float(line[colum_01]))
    l_col_02.append(float(line[colum_02]))
    l_col_03.append(float(line[colum_03]))
    l_col_04.append(float(line[colum_04]))
    l_col_05.append(float(line[colum_05]))
    l_col_06.append(float(line[colum_06]))
    l_col_07.append(float(line[colum_07]))
    l_col_08.append(float(line[colum_08]))
    l_col_09.append(float(line[colum_09]))
    l_col_10.append(float(line[colum_10]))
    l_col_11.append(float(line[colum_11]))
    l_col_12.append(float(line[colum_12]))
    l_col_13.append(float(line[colum_13]))
    l_col_14.append(float(line[colum_14]))
    l_col_15.append(float(line[colum_15]))
    l_col_16.append(float(line[colum_16]))
    l_col_17.append(float(line[colum_17]))
    l_col_18.append(float(line[colum_18]))
    l_col_19.append(float(line[colum_19]))
    l_col_20.append(float(line[colum_20]))
    l_col_21.append(float(line[colum_21]))
    l_col_22.append(float(line[colum_22]))
    l_col_23.append(float(line[colum_23]))
    l_col_24.append(float(line[colum_24]))

    # vCol00= float(line[colum_00])
    # vCol01= float(line[colum_01])
    # vCol02= float(line[colum_02])
    # vCol03= float(line[colum_03])
    # vCol04= float(line[colum_04])
    # vCol05= float(line[colum_05])
    # vCol06= float(line[colum_06])
    # vCol07= float(line[colum_07])
    # vCol08= float(line[colum_08])
    # vCol09= float(line[colum_09])
    # vCol10= float(line[colum_10])
    # vCol11= float(line[colum_11])
    # vCol12= float(line[colum_12])
    # vCol13= float(line[colum_13])
    # vCol14= float(line[colum_14])
    # vCol15= float(line[colum_15])
    # vCol16= float(line[colum_16])
    # vCol17= float(line[colum_17])
    # vCol18= float(line[colum_18])
    # vCol19= float(line[colum_19])
    # vCol20= float(line[colum_20])
    # vCol21= float(line[colum_21])
    # vCol22= float(line[colum_22])
    # vCol23= float(line[colum_23])
    # vCol24= float(line[colum_24])

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
    vCol22= 0.0
    vCol23= 0.0
    vCol24= 0.0

    

output_file = open(path + 'results.cvs', 'a') #Si no existia el archivo, lo crea y escribe al final
count = 0
for i in range(0, total_row):
    #Anadir resultados, segun columnas: nombre de test, tiempo medio de tes ...
    output_file.writelines( str(Tl_col_00[i]) + ';' + str(l_col_00[i]) + ';' + str(l_col_01[i]) + ';' + str(l_col_02[i]) + ';' + str(l_col_03[i]) + ';' + str(l_col_04[i]) + ';' + str(l_col_05[i]) + ';' + str(l_col_06[i]) + ';' + str(l_col_07[i]) + ';' + str(l_col_08[i]) + ';' + str(l_col_09[i]) + ';' + str(l_col_10[i]) + ';' + str(l_col_11[i]) + ';' + str(l_col_12[i]) + ';' + str(l_col_13[i]) + ';' + str(l_col_14[i]) + ';' + str(l_col_15[i]) + ';' + str(l_col_16[i]) + ';' + str(l_col_17[i]) + ';' + str(l_col_18[i]) + ';' + str(l_col_19[i]) + ';' + str(l_col_20[i]) + ';' + str(l_col_21[i]) + ';' + str(l_col_22[i]) + ';' + str(l_col_23[i]) + ';' + str(l_col_24[i]) + '\n')
    
    TvCol00 = TvCol00 + float(Tl_col_00[i])
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
    vCol22 = vCol22 + float(l_col_22[i])
    vCol23 = vCol23 + float(l_col_23[i])
    vCol24 = vCol24 + float(l_col_24[i]) 

    
    count+= 1

    if count % num_execute_scenario == 0 :
        output_file.writelines( str(TvCol00/10.0) + ';' +  str(vCol00/10) + ';' +  str(vCol01/10) + ';' +  str(vCol02/10) + ';' +  str(vCol03/10) + ';' +  str(vCol04/10) + ';' +  str(vCol05/10) + ';' +  str(vCol06/10) + ';' +  str(vCol07/10) + ';' +  str(vCol08/10) + ';' +  str(vCol09/10) + ';' +  str(vCol10/10) + ';' +  str(vCol11/10) + ';' +  str(vCol12/10) + ';' +  str(vCol13/10) + ';' +  str(vCol14/10) + ';' +  str(vCol15/10) + ';' +  str(vCol16/10) + ';' +  str(vCol17/10) + ';' +  str(vCol18/10) + ';' +  str(vCol19/10) + ';' +  str(vCol20/10) + ';' +  str(vCol21/10) + ';' +  str(vCol22/10) + ';' +  str(vCol23/10) + ';' +  str(vCol24/10) + '\n')
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
        vCol22= 0.0
        vCol23= 0.0
        vCol24= 0.0

output_file.close()

