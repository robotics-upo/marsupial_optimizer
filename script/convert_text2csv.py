#!/usr/bin/env python2

# -*- coding: utf-8 -*-

import csv
import os 
import sys

# Define argumento of num scenario and num position
print("\n\t *** IMPORTANT: Use this script given two parameters: <scenario_number> <initial_position_number> ***\n")  # Warnning message
arg1 = sys.argv[1] # number of scenario to analize
arg2 = sys.argv[2] # number of position to analize

path = "/home/simon/results_marsupial_optimizer/"
file1 = path + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UGV.txt'	#input file
file1_2 = path + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UGV.csv'

fin1 = open(file1, "rt")
#output file to write the result
fout1 = open(file1_2,'wt')

#for each line in the input file
for line1 in fin1:
	#read replace the string and write to output file
	fout1.write(line1.replace('.',','))
#close input and output files
fin1.close()
fout1.close()


file2 = path + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UAV.txt'	#input file
file2_2 = path + 'results'+'_stage_'+ str(arg1)+'_InitPos_'+str(arg2)+'_method_UAV.csv'

fin2 = open(file2, "rt")
#output file to write the result
fout2 = open(file2_2, 'wt')

#for each line in the input file
for line2 in fin2:
	#read replace the string and write to output file
	fout2.write(line2.replace('.',','))
#close input and output files
fin2.close()
fout2.close()

