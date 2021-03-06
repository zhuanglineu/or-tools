************************************************************************
file with basedata            : md184_.bas
initial value random generator: 48588226
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  16
horizon                       :  115
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     14      0       27        2       27
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        3          2           7  15
   3        3          1          13
   4        3          3           5   7   8
   5        3          2           6  10
   6        3          3           9  11  14
   7        3          3           9  12  14
   8        3          3           9  10  11
   9        3          1          13
  10        3          2          14  15
  11        3          2          12  15
  12        3          1          13
  13        3          1          16
  14        3          1          16
  15        3          1          16
  16        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     3       6   10    7    5
         2     3       6    9    7    6
         3     7       6    7    5    2
  3      1     1       7    7    9    7
         2     1       6    8    6    7
         3     6       4    5    4    5
  4      1     3       7    4    7    8
         2     8       7    3    5    8
         3    10       4    3    5    7
  5      1     8       8    6    9    6
         2     8       8    8    8    8
         3     9       8    6    6    4
  6      1     2       6    7    7   10
         2     6       5    3    5    9
         3     7       5    2    4    6
  7      1     1       7    7    4    8
         2     4       5    6    3    4
         3     8       4    2    3    2
  8      1     4       6    5    4    7
         2     6       4    2    4    4
         3     7       1    2    1    4
  9      1     7       6    7    9    9
         2     8       4    3    9    8
         3     8       4    2    9    9
 10      1     3       7    5    7    4
         2     4       5    5    7    4
         3     9       3    3    6    3
 11      1     2       8    7    2    8
         2     6       8    5    2    8
         3     8       6    4    2    8
 12      1     1       3    4    5    9
         2     5       3    2    5    6
         3    10       3    1    3    4
 13      1     7       8    7    4    6
         2    10       6    5    3    4
         3    10       3    6    3    3
 14      1     5       7    8   10    8
         2     6       7    7    4    7
         3     6       7    6    6    8
 15      1     3       9    8   10    4
         2     5       5    7   10    3
         3    10       5    4    9    3
 16      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   25   27   87   93
************************************************************************
