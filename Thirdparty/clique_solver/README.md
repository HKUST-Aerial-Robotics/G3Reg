# mac_solver
### Run
```angular2html
mkdir build
cd build
cmake ..
make
cd build/benchmarks
# 0 for pmc, 1 for cipper
./benchmark 0 && ./benchmark 1
```
### Bunny benchmark
#### CIPPER
```angular2html
+-------+---------+----------------+-------------------+---------------+------------+
| ρ [%] | # assoc |  affinity [ms] | dense clique [ms] | precision [%] | recall [%] |
+-------+---------+----------------+-------------------+---------------+------------+
| 0     | 64      |   3.12  ±  2.8 |      0.03  ±  0.0 | 100           | 89         |
| 0     | 256     |   2.72  ±  2.0 |      0.26  ±  0.0 | 100           | 89         |
| 0     | 512     |  10.19  ±  2.0 |      0.98  ±  0.0 | 100           | 89         |
| 0     | 1024    |  61.54  ±  2.8 |      3.22  ±  0.1 | 100           | 89         |
| 0     | 2048    | 433.69  ±  6.5 |     16.15  ±  0.4 | 100           | 89         |
+-------+---------+----------------+-------------------+---------------+------------+
| 20    | 64      |   1.91  ±  1.8 |      0.03  ±  0.0 | 100           | 89         |
| 20    | 256     |   1.99  ±  1.6 |      0.27  ±  0.1 | 100           | 89         |
| 20    | 512     |   8.64  ±  2.1 |      1.14  ±  0.5 | 100           | 89         |
| 20    | 1024    |  38.53  ±  3.2 |      4.64  ±  2.3 | 100           | 89         |
| 20    | 2048    | 261.79  ±  5.8 |     35.61  ± 36.6 | 99            | 89         |
+-------+---------+----------------+-------------------+---------------+------------+
| 40    | 64      |   1.26  ±  2.5 |      0.03  ±  0.0 | 99            | 90         |
| 40    | 256     |   1.78  ±  1.7 |      0.22  ±  0.1 | 100           | 89         |
| 40    | 512     |   4.31  ±  1.5 |      0.71  ±  0.2 | 99            | 89         |
| 40    | 1024    |  20.01  ±  0.4 |      3.31  ±  2.0 | 100           | 89         |
| 40    | 2048    | 142.29  ±  2.1 |     25.84  ± 13.4 | 99            | 89         |
+-------+---------+----------------+-------------------+---------------+------------+
| 80    | 64      |   0.74  ±  1.4 |      0.03  ±  0.0 | 99            | 91         |
| 80    | 256     |   0.76  ±  0.9 |      0.22  ±  0.0 | 100           | 90         |
| 80    | 512     |   1.62  ±  0.3 |      0.76  ±  0.2 | 100           | 89         |
| 80    | 1024    |   6.96  ±  1.3 |      2.60  ±  0.5 | 99            | 89         |
| 80    | 2048    |  39.61  ±  0.3 |      8.55  ±  4.6 | 99            | 89         |
+-------+---------+----------------+-------------------+---------------+------------+
| 90    | 64      |   2.01  ±  2.1 |      0.04  ±  0.0 | 100           | 95         |
| 90    | 256     |   3.93  ±  1.8 |      0.18  ±  0.0 | 99            | 90         |
| 90    | 512     |   4.51  ±  2.6 |      0.59  ±  0.1 | 99            | 91         |
| 90    | 1024    |   8.24  ±  2.8 |      2.98  ±  0.8 | 99            | 90         |
| 90    | 2048    |  33.88  ±  3.0 |      8.88  ±  2.5 | 99            | 90         |
+-------+---------+----------------+-------------------+---------------+------------+
```
#### PMC
```angular2html
+-------+---------+----------------+-------------------+---------------+------------+
| ρ [%] | # assoc |  affinity [ms] | dense clique [ms] | precision [%] | recall [%] |
+-------+---------+----------------+-------------------+---------------+------------+
| 0     | 64      |   1.58  ±  1.7 |      1.58  ±  1.7 | 100           | 100        |
| 0     | 256     |   2.31  ±  1.5 |      2.31  ±  1.5 | 100           | 100        |
| 0     | 512     |  11.09  ±  2.3 |     11.09  ±  2.3 | 100           | 100        |
| 0     | 1024    |  62.03  ±  2.7 |     62.03  ±  2.7 | 100           | 100        |
| 0     | 2048    | 433.62  ±  8.6 |    433.62  ±  8.6 | 100           | 100        |
+-------+---------+----------------+-------------------+---------------+------------+
| 20    | 64      |   3.89  ±  2.2 |      3.89  ±  2.2 | 99            | 100        |
| 20    | 256     |   2.77  ±  2.1 |      2.77  ±  2.1 | 99            | 99         |
| 20    | 512     |   7.01  ±  1.5 |      7.01  ±  1.5 | 99            | 99         |
| 20    | 1024    |  36.95  ±  1.8 |     36.95  ±  1.8 | 99            | 100        |
| 20    | 2048    | 245.18  ±  7.8 |    245.18  ±  7.8 | 99            | 99         |
+-------+---------+----------------+-------------------+---------------+------------+
| 40    | 64      |   2.05  ±  1.9 |      2.05  ±  1.9 | 100           | 100        |
| 40    | 256     |   1.92  ±  1.6 |      1.92  ±  1.6 | 99            | 100        |
| 40    | 512     |   5.35  ±  2.0 |      5.35  ±  2.0 | 99            | 99         |
| 40    | 1024    |  21.77  ±  2.5 |     21.77  ±  2.5 | 99            | 99         |
| 40    | 2048    | 135.76  ±  7.3 |    135.76  ±  7.3 | 99            | 99         |
+-------+---------+----------------+-------------------+---------------+------------+
| 80    | 64      |   2.52  ±  2.1 |      2.52  ±  2.1 | 98            | 100        |
| 80    | 256     |   2.58  ±  1.9 |      2.58  ±  1.9 | 98            | 99         |
| 80    | 512     |   3.59  ±  2.1 |      3.59  ±  2.1 | 98            | 99         |
| 80    | 1024    |   7.66  ±  2.1 |      7.66  ±  2.1 | 99            | 99         |
| 80    | 2048    |  34.28  ±  4.2 |     34.28  ±  4.2 | 99            | 99         |
+-------+---------+----------------+-------------------+---------------+------------+
| 90    | 64      |   1.97  ±  1.9 |      1.97  ±  1.9 | 94            | 100        |
| 90    | 256     |   1.83  ±  1.7 |      1.83  ±  1.7 | 94            | 99         |
| 90    | 512     |   2.90  ±  2.1 |      2.90  ±  2.1 | 97            | 99         |
| 90    | 1024    |   8.10  ±  3.1 |      8.10  ±  3.1 | 97            | 99         |
| 90    | 2048    |  30.02  ±  6.2 |     30.02  ±  6.2 | 97            | 99         |
+-------+---------+----------------+-------------------+---------------+------------+
```
