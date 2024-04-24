/**
 ============================================================================
 Name        : Parallel Maximum Clique (PMC) Library
 Author      : Ryan A. Rossi   (rrossi@purdue.edu)
 Description : A general high-performance parallel framework for computing
               maximum cliques. The library is designed to be fast for large
               sparse graphs.

 Copyright (C) 2012-2013, Ryan A. Rossi, All rights reserved.

 Please cite the following paper if used:
   Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa
   Patwary, A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs
   and Temporal Strong Components, arXiv preprint 1302.6256, 2013.

 See http://ryanrossi.com/pmc for more information.
 ============================================================================
 */

#ifndef PMC_HEADERS_H_
#define PMC_HEADERS_H_

#include <iostream>
#include <stdlib.h>
#include <vector>
#include <time.h>
#include <string.h>
#include <stdio.h>
#include <map>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <omp.h>
using namespace std;

#ifndef LINE_LENGTH
#define LINE_LENGTH 256
#endif
#define NANOSECOND 1000000000

#endif
