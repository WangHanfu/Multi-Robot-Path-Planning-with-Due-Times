# Multi-Robot-Path-Planning-with-Due-Times
MATLAB source codes, demo videos, dataset and raw data for my IEEE RA-L paper "Multi-Robot Path Planning with Due Times" [1].

In this paper, we propose the problem of multi-robot path planning with due times, and solve it optimally and completely using reduction-based methods firstly proposed in [2]. 
We also extend results to the anonymous setting. 

The ILP solver I'm using is Gurobi. Other solvers are also possible with some modifications, including the MATLAB self-contained linear programming solvers.

For multi-robot path planning on graph problems, Yu[2]  provides a Java realization of the reduction-based algorithms using ILP models. In contrast, my matlab realization is easily understood, and can be easily adapted to makespan, total travel time, maximum distance and total distance objectives. Interested readers can also try to use my source codes using the open source Octave software.

[1] Wang Hanfu and Chen Weidong, "Multi-Robot Path Planning with Due Times." IEEE Robotics and Automation Letters.

[2] Yu, Jingjin, and Steven M. LaValle. "Optimal multirobot path planning on graphs: Complete algorithms and effective heuristics." IEEE Transactions on Robotics 32.5 (2016): 1163-1177.
