#!/usr/bin/env python2

# -*- coding: utf-8 -*-

#Apertura de archivos CSV y gestion de datos
import numpy as np
import os
#iterate files in directory
files = os.listdir('/home/simon/results_optimizer/stage_1/goal_1/')
for file_name in files:
    file_path = '/home/simon/results_optimizer/stage_1/goal_1/' + file_name
    #number of tests per file
    total_rows = 100

    initcolum_opt_compute_time_ = 0
    initcolum_init_traj_distance_= 1
    initcolum_opt_traj_distance_ = 2
    initcolum_opt_traj_time_ = 3
    initcolum_distance_obs_init_mean_= 4
    initcolum_distance_obs_init_min_ = 5
    initcolum_distance_obs_opt_mean_ = 6
    initcolum_distance_obs_opt_min_ = 7
    initcolum_distance_obs_cat_init_mean_ = 8 
    initcolum_distance_obs_cat_init_min_ = 9
    initcolum_distance_obs_cat_opt_mean_ = 10
    initcolum_distance_obs_cat_opt_min_ = 11
    initcolum_opt_traj_vel_mean_= 12
    initcolum_opt_traj_vel_max_ = 13
    initcolum_opt_traj_acc_mean_ = 14
    initcolum_opt_traj_acc_max_ = 15
   
    file = open(file_path, 'r')
    lines = file.read().splitlines() #crea una lista
    #lines.pop(0) #quitamos primera linea en caso de tener titulos 
    #print(lines)

    opt_compute_time_ = []
    init_traj_distance_ = []
    opt_traj_distance_ = []
    opt_traj_time_ = []
    distance_obs_init_mean_ = []
    distance_obs_init_min_ = []
    distance_obs_opt_mean_ = []
    distance_obs_opt_min_ = []
    distance_obs_cat_init_mean_ = []
    distance_obs_cat_init_min_ = []
    distance_obs_cat_opt_mean_ = []
    distance_obs_cat_opt_min_ = []
    opt_traj_vel_mean_ = []
    opt_traj_vel_max_ = []
    opt_traj_acc_mean_ = [] 
    opt_traj_acc_max_ = []


    for l in lines:
        line = l.split(',') #devuelve igualmente una lista
        #print line

        opt_compute_time_.append(float(line[initcolum_opt_compute_time_]))
        init_traj_distance_.append(float(line[initcolum_init_traj_distance_]))
        opt_traj_distance_.append(float(line[initcolum_opt_traj_distance_]))
        opt_traj_time_.append(float(line[initcolum_opt_traj_time_]))
        distance_obs_init_mean_.append(float(line[initcolum_distance_obs_init_mean_]))
        distance_obs_init_min_.append(float(line[initcolum_distance_obs_init_min_]))
        distance_obs_opt_mean_.append(float(line[initcolum_distance_obs_opt_mean_]))
        distance_obs_opt_min_.append(float(line[initcolum_distance_obs_opt_min_]))
        distance_obs_cat_init_mean_.append(float(line[initcolum_distance_obs_cat_init_mean_]))
        distance_obs_cat_init_min_.append(float(line[initcolum_distance_obs_cat_init_min_]))
        distance_obs_cat_opt_mean_.append(float(line[initcolum_distance_obs_cat_opt_mean_]))
        distance_obs_cat_opt_min_.append(float(line[initcolum_distance_obs_cat_opt_min_]))
        opt_traj_vel_mean_.append(float(line[initcolum_opt_traj_vel_mean_]))
        opt_traj_vel_max_.append(float(line[initcolum_opt_traj_vel_max_]))
        opt_traj_acc_mean_.append(float(line[initcolum_opt_traj_acc_mean_l]))
        opt_traj_acc_max_.append(float(line[initcolum_opt_traj_acc_max_]))

        #Calculos en todos los tests
        #print float(line[test_time_col])
          
    opt_compute_time_ = np.asarray(opt_compute_time_)
    opt_compute_time = np.std(opt_compute_time_, axis=0)/np.sqrt(len(opt_compute_time_))

    init_traj_distance_ = np.asarray(init_traj_distance_)
    init_traj_distance = np.std(init_traj_distance_, axis=0)/np.sqrt(len(init_traj_distance_))

    opt_traj_distance_ = np.asarray(opt_traj_distance_)
    opt_traj_distance = np.std(opt_traj_distance_, axis=0)/np.sqrt(len(opt_traj_distance_))

    opt_traj_time_ = np.asarray(opt_traj_time_)
    opt_traj_time = np.std(opt_traj_time_, axis=0)/np.sqrt(len(opt_traj_time_))

    distance_obs_init_mean_ = np.asarray(distance_obs_init_mean_)
    distance_obs_init_mean = np.std(distance_obs_init_mean_, axis=0)/np.sqrt(len(distance_obs_init_mean_))

    distance_obs_init_min_ = np.asarray(distance_obs_init_min_)
    distance_obs_init_min = np.std(distance_obs_init_min_, axis=0)/np.sqrt(len(distance_obs_init_min_))

    distance_obs_opt_mean_ = np.asarray(distance_obs_opt_mean_)
    distance_obs_opt_mean = np.std(distance_obs_opt_mean_, axis=0)/np.sqrt(len(distance_obs_opt_mean_))

    distance_obs_opt_min_ = np.asarray(distance_obs_opt_min_)
    distance_obs_opt_min = np.std(distance_obs_opt_min_, axis=0)/np.sqrt(len(distance_obs_opt_min_))

    distance_obs_cat_init_mean_= np.asarray(distance_obs_cat_init_mean_)
    distance_obs_cat_init_mean= np.std(distance_obs_cat_init_mean_, axis=0)/np.sqrt(len(distance_obs_cat_init_mean_))

    distance_obs_cat_init_min_ = np.asarray(distance_obs_cat_init_min_)
    distance_obs_cat_init_min = np.std(distance_obs_cat_init_min_, axis=0)/np.sqrt(len(distance_obs_cat_init_min_))

    distance_obs_cat_opt_mean_= np.asarray(distance_obs_cat_opt_mean_)
    distance_obs_cat_opt_mean= np.std(distance_obs_cat_opt_mean_, axis=0)/np.sqrt(len(distance_obs_cat_opt_mean_))

    distance_obs_cat_opt_min_ = np.asarray(distance_obs_cat_opt_min_)
    distance_obs_cat_opt_min = np.std(distance_obs_cat_opt_min_, axis=0)/np.sqrt(len(distance_obs_cat_opt_min_))

    opt_traj_vel_mean_ = np.asarray(opt_traj_vel_mean_)
    opt_traj_vel_mean = np.std(opt_traj_vel_mean_, axis=0)/np.sqrt(len(opt_traj_vel_mean_))

    opt_traj_vel_max_ = np.asarray(opt_traj_vel_max_)
    opt_traj_vel_max = np.std(opt_traj_vel_max_, axis=0)/np.sqrt(len(opt_traj_vel_max_))

    opt_traj_acc_mean_ = np.asarray(opt_traj_acc_mean_)
    opt_traj_acc_mean = np.std(opt_traj_acc_mean_, axis=0)/np.sqrt(len(opt_traj_acc_mean_))

    opt_traj_acc_max_ = np.asarray(opt_traj_acc_max_)
    opt_traj_acc_max = np.std(opt_traj_acc_max_, axis=0)/np.sqrt(len(opt_traj_acc_max_))

   
    #Generar archivo de solucion, comprobar si existe, si no crearlo poniendole los titulos
    if os.path.isfile('/home/simon/results_optimizer/stage_1/goal_1/results.cvs')==False: #comprueba si existe el archivo
        output_file = open('/home/simon/results_optimizer/stage_1/goal_1/results.cvs','w') #Si no existia el archivo, lo crea 
        #Anadimos titulos de columnas
        output_file.writelines('Test name;opt_compute_time;init_traj_distance;opt_traj_distance;opt_traj_time;distance_obs_init_mean;distance_obs_init_min;distance_obs_opt_mean;distance_obs_opt_min_h;distance_obs_cat_init_mean;distance_obs_cat_init_min;distance_obs_cat_opt_mean;distance_obs_cat_opt_min;opt_traj_vel_mean;opt_traj_vel_max;opt_traj_acc_mean;opt_traj_acc_max\n')
        output_file.close()
    
    
    output_file = open('/home/simon/results_optimizer/stage_1/goal_1/results.cvs', 'a') #Si no existia el archivo, lo crea y escribe al final
    #Anadir resultados, segun columnas: nombre de test, tiempo medio de tes ...
    output_file.writelines(str(opt_compute_time) + ';' + str(init_traj_distance) + ';' + str(opt_traj_distance) + ';' + str(opt_traj_time) + ';' + str(distance_obs_init_mean) + ';' + str(distance_obs_init_min) + ';' + str(distance_obs_opt_mean) + ';' + str(distance_obs_opt_min) + ';' + str(distance_obs_cat_init_mean) + ';' + str(distance_obs_cat_init_min) + ';' + str(distance_obs_cat_opt_mean) + ';' + str(distance_obs_cat_opt_min) + ';' + str(opt_traj_vel_mean) + ';' + str(opt_traj_vel_max) + ';' + str(opt_traj_acc_mean) + ';' + str(opt_traj_acc_max) + '\n')
    output_file.close()


