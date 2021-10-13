#!/usr/bin/env python2

# -*- coding: utf-8 -*-

import csv
import os 

path = "/home/simon/"

#input file
fin = open(path + "results_optimization_stage_6_InitPos_2_goal_0.txt", "rt")
#output file to write the result to
fout = open(path + "sample_optimizer.csv", "wt")
#for each line in the input file
for line in fin:
	#read replace the string and write to output file
	fout.write(line.replace('.', ','))
#close input and output files
fin.close()
fout.close()

reader = csv.reader(open(path + "sample_optimizer.csv"))

f = open(path + "results_optimization_all_data.csv", "w")
writer = csv.writer(f)
for row in reader:
    		writer.writerow(row)
f.close()

os.remove(path + "sample_optimizer.csv")
