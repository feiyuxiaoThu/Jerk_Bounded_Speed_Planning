# 基于动态规划的速度规划

Apollo piecewise_jerk算法主要包括两方面，路径规划和速度规划，最后组合成轨迹

路径规划: PIECEWISE_JERK_PATH_OPTIMIZER (QP)
速度规划: SPEED_HEURISTIC_OPTIMIZER (DP)
         PIECEWISE_JERK_NONLINEAR_SPEED_OPTIMIZER  (QP)

Apollo内部关于车辆速度曲线的规划，主要是涉及了几个阶段的主要任务表述:

+ Speed_bounds_prior_decider:
      首先，将障碍物的轨迹映射到s-t graph，随后计算出障碍物的轨迹（prior过程，障碍物在速度方面不存在decision，因此此次计算轨迹是withoutdecision），并将其放置boundaries集合中，随后设置速度最大限制（沿规划好路径），最后，st_graph_data保存boundaries,speed_limit等.
+ DP_ST_SPEED_OPTIMIZER：
      根据上述的boundaries，进行计算启发式速度曲线搜索，得到最优轨迹
+ SPEED_DECIDER：
      根据上述的speed曲线，来初步判断对障碍物的decision，其中行人需要特别处理，设置相应的决策行为
+ SPEED_BOUNDS_FINAL_DECIDER：
      重新计算障碍物stboundary，然后根据障碍物的decision，确定速度的边界
+ PIECEWISE_JERK_SPEED_OPTIMIZER
      基于osqp进行速度曲线的数值优化计算，得出最优曲线.