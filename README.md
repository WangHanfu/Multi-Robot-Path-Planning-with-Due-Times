# Multi-Robot-Path-Planning-with-Due-Times
MATLAB source codes, dataset and raw data for IEEE RA-L paper "Multi-Robot Path Planning with Due Times" [1].

In this paper, the problem of multi-robot path planning with due times is proposed, and solved completely using reduction-based methods. This reduction-based methdology is firstly proposed in [2]. The problem of anonymous multi-robot path planning with due times is also studied. 


Interested readers could do the following.

1. The used ILP solver is Gurobi. Other solvers are also possible with some minor modifications, including the MATLAB self-contained linear programming solvers.

2. For multi-robot path planning on graph problems, Yu [2]  provides a Java realization of the reduction-based algorithms. In contrast, my Matlab realization is easily understood. I have leave some space for expansion to makespan, total travel time, maximum distance and total distance objectives. 

3. Interested readers can also try to use my source codes using the open source Octave instead with minor changes.




************************************************************************************************************************************************

[1] Wang Hanfu and Chen Weidong, "Multi-Robot Path Planning with Due Times." IEEE Robotics and Automation Letters.

[2] Yu, Jingjin, and Steven M. LaValle. "Optimal multirobot path planning on graphs: Complete algorithms and effective heuristics." IEEE Transactions on Robotics 32.5 (2016): 1163-1177.
