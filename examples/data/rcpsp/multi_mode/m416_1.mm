************************************************************************
file with basedata            : cm416_.bas
initial value random generator: 10990
************************************************************************
projects                      :  1
jobs (incl. supersource/sink ):  18
horizon                       :  142
RESOURCES
  - renewable                 :  2   R
  - nonrenewable              :  2   N
  - doubly constrained        :  0   D
************************************************************************
PROJECT INFORMATION:
pronr.  #jobs rel.date duedate tardcost  MPM-Time
    1     16      0       12        1       12
************************************************************************
PRECEDENCE RELATIONS:
jobnr.    #modes  #successors   successors
   1        1          3           2   3   4
   2        4          1           5
   3        4          3           5   6   8
   4        4          3           8  14  16
   5        4          3          11  12  14
   6        4          2           7   9
   7        4          2          12  16
   8        4          2          11  12
   9        4          1          10
  10        4          2          11  13
  11        4          2          15  17
  12        4          2          15  17
  13        4          3          14  15  16
  14        4          1          17
  15        4          1          18
  16        4          1          18
  17        4          1          18
  18        1          0        
************************************************************************
REQUESTS/DURATIONS:
jobnr. mode duration  R 1  R 2  N 1  N 2
------------------------------------------------------------------------
  1      1     0       0    0    0    0
  2      1     3       8    5    2    0
         2     5       7    4    0    5
         3     7       6    4    0    4
         4     8       4    4    0    3
  3      1     4       3    8    0    7
         2     5       3    8    3    0
         3     6       3    7    0    6
         4    10       2    7    3    0
  4      1     2       4    6   10    0
         2     6       3    5    9    0
         3     7       3    3    8    0
         4     8       3    2    6    0
  5      1     4       3    8    7    0
         2     5       3    7    6    0
         3     9       3    7    0    6
         4    10       3    6    4    0
  6      1     1       8    5    0    6
         2     5       7    3    6    0
         3     6       7    3    3    0
         4     9       6    1    3    0
  7      1     3       9    7    5    0
         2     4       8    7    0    3
         3     6       8    4    0    3
         4     7       5    4    2    0
  8      1     1       8    8    6    0
         2     6       7    7    0    9
         3    10       7    6    0    5
         4    10       7    7    1    0
  9      1     1       8    9    0    6
         2     2       6    8    0    6
         3     4       6    6    2    0
         4     9       2    4    0    5
 10      1     1      10    6    7    0
         2     2       9    6    0    5
         3     8       9    5    6    0
         4    10       7    4    0    4
 11      1     2       2   10   10    0
         2     3       2   10    0    8
         3     4       1    9    0    5
         4    10       1    8    0    5
 12      1     2       8   10    5    0
         2     2       9    9    7    0
         3     3       8    6    0    2
         4     7       8    3    0    2
 13      1     1       2    7    0    5
         2     2       2    3    0    4
         3     2       2    5    1    0
         4     9       2    2    0    5
 14      1     2       9    3   10    0
         2     6       8    3    0    2
         3     8       6    3    2    0
         4    10       3    3    0    2
 15      1     2      10    3    8    0
         2     3       7    3    0    4
         3     9       6    3    0    4
         4    10       4    2    0    3
 16      1     1       9    7    0    5
         2     5       8    7    0    5
         3     6       8    7    6    0
         4     8       7    7    6    0
 17      1     1       6    7    6    0
         2     2       4    7    6    0
         3     4       4    7    0    4
         4     7       3    6    3    0
 18      1     0       0    0    0    0
************************************************************************
RESOURCEAVAILABILITIES:
  R 1  R 2  N 1  N 2
   29   30   51   39
************************************************************************
