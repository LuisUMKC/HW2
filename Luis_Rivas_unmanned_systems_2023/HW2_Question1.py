# -*- coding: utf-8 -*-
"""
Created on Mon Sep 11 11:27:20 2023

@author: mario
"""

def distance(x1:int, y1:int, x2:int, y2:int):
    
  # Calculating distance
  
  x1 = 2 
  y1 = 1 
  x2 = 3 
  y2 = 2
    
  return (((x2 - x1)**2 +(y2 - y1)**2)**0.5)
  
# run
  
print( distance(2, 1, 3, 2))