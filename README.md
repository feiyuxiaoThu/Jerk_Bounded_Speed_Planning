# Jerk Constraint Velocity Planning for an Autonomous Vehicle: Quadratic Programming Approach

> A pure c implemention of this method can be found in [feiyuxiaoThu
/
piecewise_jerk_path_optimizer](https://github.com/feiyuxiaoThu/piecewise_jerk_path_optimizer). Please refer to it !

## Dependencies. 
This code has been tested on Ubuntu18.04/20.04. External dependencies are 

- Eigen3
- OSQP
- CMake3.3 or higher

## Acknowledgements
This code is the self-research code for performing velocity planning **on a fixed path** using convex optimization method, and the structure of the repo is highly based on the nice work [Jerk Constrained Velocity Planning for an Autonomous Vehicle: Linear Programming Approach](https://arixv.org/abs/2202.10029), and their open-source code [jerk_optimal_velocity_planning](https://github.com/pflab-ut/jerk_optimal_velocity_planning).

## Result

### Normal

![](/result/Normal.png)

### Cutin

![](/result/Cutin.png)

### Cutout

![](/result/Cutout.png)

### Stop

![](/result/Stop.png)

### Start 

![](/result/Start.png)

### Acc

![](/result/Acc.png)

## References

For further interests on this topic, I recommend you refer to the following papers:

+ Optimal Vechile Path Planning Using Quadratic Optimization for Baidu Apollo Open Platform
+ A Real-Time Motion Planner with Trajectory Optimization for Autonomous Vehicles
+ Safe Trajectory Generation For Complex Urban Environments Using Spatio-temporal Semantic Corridor (More deeper ...)





