# -*- coding: utf-8 -*-

#Apertura de archivos CSV y gestion de datos


import numpy as np
import os
#iterate files in directory
files = os.listdir('/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/')
for file_name in files:
    file_path = '/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/' + file_name
    #number of tests per file
    total_rows = 200
    success_rows = 0 
    
    #column numbers for rrt_antiguos
    
    test_time_col = 0
    path_time_col = 1
    path_cost_col = 4

    
    #output values
    fail_num = 0
    test_time_average = 0.0 #cambiar esto para que sean NAN??
    path_time_average = 0.0
    path_cost_average = 0.0
    
    file = open(file_path, 'r')
    lines = file.read().splitlines() #crea una lista
    #lines.pop(0) #quitamos primera linea en caso de tener titulos 
    #print(lines)
    
    SEM_cost_col = []
    SEM_time_test = []
    SEM_time_path = []


    for l in lines:
        line = l.split(',') #devuelve igualmente una lista
        #print line

        SEM_cost_col.append(float(line[path_cost_col]))
        SEM_time_test.append(float(line[test_time_col]))
        SEM_time_path.append(float(line[path_time_col]))
        #Calculos en todos los tests
        #print float(line[test_time_col])
        test_time_average = test_time_average + float(line[test_time_col])
       
        #Calculos en test con solucion
        if float(line[path_time_col]) != 0:
            #print float(line[path_time_col])
            success_rows = success_rows + 1
            path_time_average = path_time_average + float(line[path_time_col])
            path_cost_average = path_cost_average + float(line[path_cost_col])
    
    SEM_cost_col = np.asarray(SEM_cost_col)
    sem_cost = np.std(SEM_cost_col, axis=0)/np.sqrt(len(SEM_cost_col))

    SEM_time_test = np.asarray(SEM_time_test)
    sem_time_t = np.std(SEM_time_test, axis=0)/np.sqrt(len(SEM_time_test))

    SEM_time_path = np.asarray(SEM_time_path)
    sem_time_p = np.std(SEM_time_path, axis=0)/np.sqrt(len(SEM_time_path))


    fail_num = total_rows - success_rows
    test_time_average = test_time_average/total_rows
    if success_rows != 0:
        path_time_average = path_time_average/success_rows
        path_cost_average = path_cost_average/success_rows
    
    
    print 'Solutions are:'
    print fail_num
    print test_time_average
    print path_time_average
    print path_cost_average
    
    #Generar archivo de solucion, comprobar si existe, si no crearlo poniendole los titulos
    if os.path.isfile('/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/results_test.csv')==False: #comprueba si existe el archivo
        output_file = open('/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/results_test.csv', 'w') #Si no existía el archivo, lo crea 
        #Añadimos titulos de columnas
        output_file.writelines('Test name;Fallos en 200;Test time average;Trajectory time average;Path cost average;SEM Cost;SEM Time Test;SEM Time Path\n')
        output_file.close()
    
    
    output_file = open('/home/saimon/siar_ws/src/siar_navigation/siar_planner/output_files/nFailMax=10/results_test.csv', 'a') #Si no existía el archivo, lo crea y escribe al final
    #Añadir resultados, segun columnas: nombre de test, tiempo medio de test, tiempo medio de path obtenido, coste medio de path obtenido
    output_file.writelines(file_name + ';' + str(fail_num) + ';' + str(test_time_average) + ';' + str(path_time_average) + ';' + str(path_cost_average) + ';' + str(sem_cost) + ';' + str(sem_time_t) + ';' + str(sem_time_p)+ '\n')
    output_file.close()
