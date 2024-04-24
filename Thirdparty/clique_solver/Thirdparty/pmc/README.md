Parallel Maximum Clique (PMC) Library
=====================================

In short, a parameterized high performance library for computing maximum cliques in large sparse graphs.

Finding maximum cliques, k-cliques, and temporal strong components are in general NP-hard.
Yet, these can be computed fast in most social and information networks.
The PMC library is designed to be fast for solving these problems.
Algorithms in the PMC library are easily adaptable for use with a variety of orderings, heuristic strategies, and bounds.

* **Maximum clique:** 		 Given a simple undirected graph G and a number k, output the clique of largest size.
* **K-clique:** 	   		 In k-clique, the problem is to find a clique of size k if one exists.
* **Largest temporal-scc:** Given a temporal graph G, a temporal strong component is a set of vertices where all temporal paths exist between the vertices in that set. The Largest TSCC problem is to find the largest among all the temporal strong components.



Features
--------
0.  General framework for parallel maximum clique algorithms
1.	Optimized to be fast for large sparse graphs 
	+ 	Algorithms tested on networks of 1.8 billion edges
2.	Set of fast heuristics shown to give accurate approximations
3.	Algorithms for computing Temporal Strongly Connected Components (TSCC) of large dynamic networks
4.	Parameterized for computing k-cliques as fast as possible
5.  Includes a variety of tight linear time bounds for the maximum clique problem
6.  Ordering of vertices for each algorithm can be selected at runtime 
7.  Dynamically reduces the graph representation periodically as vertices are pruned or searched
	+   Lowers memory-requirements for massive graphs, increases speed, and has caching benefits


Synopsis
---------

### Setup
First, you'll need to compile the parallel maximum clique library.  

		$ cd path/to/pmc/
		$ make

Afterwards, the following should work:  

		# compute maximum clique using the full algorithm `-a 0`
		./pmc -f data/socfb-Texas84.mtx -a 0


*PMC* has been tested on Ubuntu linux (10.10 tested) and Mac OSX (Lion tested) with gcc-mp-4.7 and gcc-mp-4.5.4

Please let me know if you run into any issues.  

  
   
### Input file format
+ Matrix Market Coordinate Format (symmetric)  
For details see: <http://math.nist.gov/MatrixMarket/formats.html#MMformat>  

		%%MatrixMarket matrix coordinate pattern symmetric  
		4 4 6  
		2 1  
		3 1  
		3 2  
		4 1  
		4 2  
		4 3 


+ Edge list (symmetric and unweighted):
		Codes for transforming the graph into the correct format are provided in the experiments directory.


Overview
---------

The parallel maximum clique algorithms use tight bounds that are fast to compute.
A few of those are listed below.

* K-cores
* Degree
* Neighborhood cores
* Greedy coloring

All bounds are dynamically updated.  

Examples of the three main maximum clique algorithms are given below.
Each essentially builds on the other.

	# uses the four basic k-core pruning steps
	./pmc -f ../pmc/data/output/socfb-Stanford3.mtx -a 2

	# k-core pruning and greedy coloring
	./pmc -f ../pmc/data/output/socfb-Stanford3.mtx -a 1
	
	# neighborhood core pruning (and ordering for greedy coloring)
	./pmc -f ../pmc/data/output/socfb-Stanford3.mtx -a 0





### Dynamic graph reduction
	
The reduction wait parameter `-r` below is set to be 1 second (default = 4 seconds).
 
	./pmc -f data/sanr200-0-9.mtx -a 0 -t 2 -r 1

In some cases, it may make sense to turn off the explicit graph reduction. 
This is done by setting the reduction wait time '-r' to be very large.

	# Set the reduction wait parameter
	./pmc -f data/socfb-Stanford3.mtx -a 0 -t 2 -r 999






### Orderings

The PMC algorithms are easily adapted to use various ordering strategies. 
To prescribe a vertex ordering, use the -o option with one of the following:
+ `deg`
+ `kcore`
+ `dual_deg`&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;orders vertices by the sum of degrees from neighbors
+ `dual_kcore`&nbsp;&nbsp;orders vertices by the sum of core numbers from neighbors
+ `kcore_deg`&nbsp;&nbsp;&nbsp; vertices are ordered by the weight k(v)d(v)
+ `rand`&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; randomized ordering of vertices



##### Direction of ordering

Vertices are searched by default in increasing order, to search vertices in decreasing order, use the `d` option:

	./pmc -f data/p-hat700-2.mtx -a 0 -d




### Heuristic
The fast heuristic may also be customized to use various greedy selection strategies.
This is done by using `-h` with one of the following: 

+ `deg`
+ `kcore`
+ `kcore_deg`&nbsp;&nbsp;&nbsp; select vertex that maximizes k(v)d(v)
+ `rand`&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; randomly select vertices


#### Terminate after applying the heuristic
Approximate the maximum clique using _ONLY_ the heuristic by not setting the exact algorithm via the `-a [num]` option.
For example:  

	./pmc -f data/sanr200-0-9.mtx -h deg
	
#### Turning the heuristic off

	# heuristic is turned off by setting `-h 0`.
	./pmc -f data/tscc_enron-only.mtx -h 0 -a 0



### K-clique

The parallel maximum clique algorithms have also been parameterized to find cliques of size k.
This routine is useful for many tasks in network analysis such as mining graphs and community detection.

	# Computes a clique of size 50 from the Stanford facebook network
	./pmc -f data/socfb-Stanford3.mtx -a 0 -k 50


using `-o rand` to find potentially different cliques of a certain size

	# Computes a clique of size 36 from sanr200-0-9
	./pmc -f data/sanr200-0-9.mtx -a 0 -k 36 -o rand



Terms and conditions
--------------------
Please feel free to use these codes. We only ask that you cite:  

	Ryan A. Rossi, David F. Gleich, Assefaw H. Gebremedhin, Md. Mostofa Patwary,  
	A Fast Parallel Maximum Clique Algorithm for Large Sparse Graphs and Temporal  
	Strong Components, arXiv preprint 1302.6256, 2013.  

_These codes are research prototypes and may not work for you. No promises. But do email if you run into problems._


Copyright 2011-2013, Ryan A. Rossi. All rights reserved.
		