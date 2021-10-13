#!/usr/bin/env python2

# -*- coding: utf-8 -*-

import csv
import os 

path = "/home/simon/"
#input file
fin = open(path + "time_compute_initial_planner.txt", "rt")
#output file to write the result to
fout = open(path + "sample_time.csv", "wt")
#for each line in the input file
for line in fin:
	#read replace the string and write to output file	
	fout.write(line.replace('.', ','))
#close input and output files
fin.close()
fout.close()

#input file
fin = open(path + "results_optimization_stage_6_InitPos_2_goal_0.txt", "rt")
#output file to write the result to
fout = open(path + "sample_optimizer.csv", "wt")
#for each line in the input file
for line in fin:
	#read replace the string and write to output file
	fout.write(line.replace(',', ';').replace('.', ','))
#close input and output files
fin.close()
fout.close()

reader = csv.reader(open(path + "sample_time.csv"))
reader1 = csv.reader(open(path + "sample_optimizer.csv"))

f = open(path + "results_optimization_all_data.csv", "w")
writer = csv.writer(f)
for row in reader:
	for row1 in reader1:
    		writer.writerow(row+row1)
f.close()

os.remove(path + "sample_time.csv")
os.remove(path + "sample_optimizer.csv")


# f = open(path + "results_optimization_all_data.csv", "w")
# writer = csv.writer(f)
#for row in reader1:
#    writer.writerow(row)
#f.close()


# ff = open("combined_last.csv", 'w')
# count = 0.0
# col = 0
# average = 0.0
# Sum = 0.0

# with open("combined.csv", "r") as file:
# 	writer = csv.writer(ff)
# 	reader2 = file.readlines()
# 	for row in reader2:
# 		writer.writerow(row)
# 		count += 1
# 		if count % 10 == 0.0 :
# 			for colum in row.split(';'):
# 				writer.writerow(colum)
# 				Sum += float(colum[col])
# 				print(Sum)
				
# 					value = Sum / 10
# 					average = 0.0
# 					file.writerow(col,)
# 			col += 1
		
