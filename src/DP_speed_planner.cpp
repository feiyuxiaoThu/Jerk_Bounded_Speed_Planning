// https://blog.csdn.net/weixin_43078345/article/details/103737910

// 速度规划——S-T图
// S-T Graph
// S-T Graph 是将有可能和path有overlap的障碍物投影到该path上面，生成st bounary
// 而速度规划则根据生成的st boundary进行纵向规划，其中s是指主车在该path上的行驶里程。
// S-T graph经常被用来做速度规划

#include "DP_speed_planner.h"

// 数据
std::vector<StBoundary> st_boundaries;

double FLAGS_planning_upper_speed_limit = 15;
static constexpr double kDoubleEpsilon = 1.0e-6;
double unit_t_ = 1.0;

// 总时间   // level
double total_length_t_ = 2;

double total_length_s_ = 30;

double dense_dimension_s_ = 101;

double sparse_unit_s_ = 1.0;
double dense_unit_s_ = 0.1;

double max_acceleration_ = 2.0;
double max_deceleration_ = -4.0;

uint32_t dimension_t_;
uint32_t dimension_s_;

std::vector<std::vector<StGraphPoint>> cost_table_;
std::vector<double> speed_limit_by_index_;
std::vector<double> spatial_distance_by_index_;

bool InitCostTable()
{
  // Time dimension is homogeneous while Spatial dimension has two resolutions,
  // dense and sparse with dense resolution coming first in the spatial horizon

  dimension_t_ = static_cast<uint32_t>(std::ceil(
                     total_length_t_ / static_cast<double>(unit_t_))) +
                 1;

  // double sparse_length_s =
  //     total_length_s_ -
  //     static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_;

  // uint32_t sparse_dimension_s_ =
  //     sparse_length_s > std::numeric_limits<double>::epsilon()
  //         ? static_cast<uint32_t>(std::ceil(sparse_length_s / sparse_unit_s_))
  //         : 0;
  // dense_dimension_s_ =
  //     sparse_length_s > std::numeric_limits<double>::epsilon()
  //         ? dense_dimension_s_
  //         : static_cast<uint32_t>(std::ceil(total_length_s_ / dense_unit_s_)) +
  //               1;
  // dimension_s_ = dense_dimension_s_ + sparse_dimension_s_;

  dimension_s_ = 10;

  // Sanity Check
  if (dimension_t_ < 1 || dimension_s_ < 1)
  {
    const std::string msg = "Dp st cost table size incorrect.";
    return false;
  }

  cost_table_.clear();
  StPoint st_point(0, 0);

  std::cout << "dimension_t_   " << dimension_t_ << std::endl;
  std::cout << "dimension_s_   " << dimension_s_ << std::endl;
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      dimension_t_, std::vector<StGraphPoint>(dimension_s_, StGraphPoint(st_point)));

  // double curr_t = 0.0;
  // for (uint32_t i = 0; i < cost_table_.size(); ++i, curr_t += unit_t_)
  // {
  //   auto &cost_table_i = cost_table_[i];
  //   double curr_s = 0.0;
  //   for (uint32_t j = 0; j < dense_dimension_s_; ++j, curr_s += dense_unit_s_)
  //   {
  //     cost_table_i[j].Init(i, j, StPoint(curr_s, curr_t));
  //   }
  //   curr_s = static_cast<double>(dense_dimension_s_ - 1) * dense_unit_s_ +
  //            sparse_unit_s_;
  //   for (uint32_t j = dense_dimension_s_; j < cost_table_i.size();
  //        ++j, curr_s += sparse_unit_s_)
  //   {
  //     cost_table_i[j].Init(i, j, StPoint(curr_s, curr_t));
  //   }
  // }

  // const auto &cost_table_0 = cost_table_[0]; //第一列数据
  // spatial_distance_by_index_.clear();
  // spatial_distance_by_index_ = std::vector<double>(cost_table_0.size(), 0.0);
  // for (uint32_t i = 0; i < cost_table_0.size(); ++i)
  // {
  //   spatial_distance_by_index_[i] = cost_table_0[i].st_point.s;
  // }
  return true;
}

bool InitSpeedLimitLookUp()
{
  speed_limit_by_index_.clear();

  speed_limit_by_index_.resize(dimension_s_);

  // const auto& speed_limit = st_graph_data_.speed_limit();

  // for (uint32_t i = 0; i < dimension_s_; ++i) {
  //   speed_limit_by_index_[i] =
  //       speed_limit.GetSpeedLimitByS(cost_table_[0][i].point().s());
  // }
  return true;
}

// node和障碍物的cost
double GetObstacleCost(const StGraphPoint st_graph_point)
{
  float epslion = 0.001;
  float node_s = st_graph_point.st_point.s;
  float node_t = st_graph_point.st_point.t;

  float cost = 0;

  // std::cout << "st_boundaries.size()   " << (st_boundaries.size()) << std::endl;

  for (int i = 0; i < st_boundaries.size(); i++)
  {
    st_boundaries[i].lower_points_;
    for (int m = 0; m < st_boundaries[i].lower_points_.size(); m++)
    {
      StPoint st_point_tmp = st_boundaries[i].lower_points_[m];
      // std::cout << "lower_points_  t,s  " << st_point_tmp.t << "   " << st_point_tmp.s << std::endl;
      if (st_point_tmp.t == node_t)
      {
        cost += 1 / abs(st_point_tmp.s - node_s + 1);
      }
    }

    for (int m = 0; m < st_boundaries[i].upper_points_.size(); m++)
    {
      StPoint st_point_tmp = st_boundaries[i].upper_points_[m];
      // std::cout << "upper_points_  t,s  " << st_point_tmp.t << "   " << st_point_tmp.s << std::endl;

      if (st_point_tmp.t == node_t)
      {
        cost += 1 / abs(st_point_tmp.s - node_s + 1);
      }
    }
  }

  return cost;
}

bool CalculateCostAt(size_t c, size_t r)
{
  //   const uint32_t c = c;
  // const uint32_t r = r;
  StGraphPoint cost_cr = cost_table_[c][r];
  // if ((c - 1) >= 0)
  // {
  //   cost_cr.SetPreNode(cost_table_[c - 1][r]);
  // }
  // else
  // {
  //   cost_cr.SetPreNode(cost_table_[0][r]);
  // }

  if (0)
  {
    // dp_st_cost_.GetObstacleCost(cost_cr)
    // 和障碍物的cost
    double ObstacleCost_ = GetObstacleCost(cost_cr);
    cost_cr.ObstacleCost = ObstacleCost_;

    // cost_cr.SetObstacleCost(ObstacleCost_);
    // if (
    //   .ObstacleCost > std::numeric_limits<double>::max())
    // {
    //   return true;
    // }

    // dp_st_cost_.GetSpatialPotentialCost(cost_cr)
    double SpatialPotentialCost = 0;
    // cost_cr.SetSpatialPotentialCost(SpatialPotentialCost);
    cost_cr.SpatialPotentialCost = SpatialPotentialCost;

    const auto &cost_init = cost_table_[0][0];
    double init_point_v = 0;
    if (c == 0)
    {
      cost_cr.SetTotalCost(0.0);
      cost_cr.SetCruiseSpeed(init_point_v);
      return true;
    }

    const double speed_limit = speed_limit_by_index_[r];
    double st_graph_data_cruise_speed = 10;
    const double cruise_speed = st_graph_data_cruise_speed;
    // The mininal s to model as constant acceleration formula
    // default: 0.25 * 7 = 1.75 m
    const double min_s_consider_speed = dense_unit_s_ * dimension_t_;

    if (c == 1)
    {
      const double acc =
          2 * (cost_cr.st_point.s / unit_t_ - init_point_v) / unit_t_;
      if (acc < max_deceleration_ || acc > max_acceleration_)
      {
        return true;
      }

      if (init_point_v + acc * unit_t_ < -kDoubleEpsilon &&
          cost_cr.st_point.s > min_s_consider_speed)
      {
        return true;
      }

      // if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
      //                             cost_init)) {
      //   return;
      // }

      cost_cr.SetTotalCost(
          cost_cr.ObstacleCost + cost_cr.SpatialPotentialCost +
          cost_init.cost /*+
        CalculateEdgeCostForSecondCol(r, speed_limit, cruise_speed)*/
      );

      cost_cr.SetPreNode(cost_init);
      cost_cr.SetCruiseSpeed(init_point_v + acc * unit_t_);
      return true;
    }

    static constexpr double kSpeedRangeBuffer = 0.20;
    const double pre_lowest_s =
        cost_cr.st_point.s -
        FLAGS_planning_upper_speed_limit * (1 + kSpeedRangeBuffer) * unit_t_;
    const auto pre_lowest_itr =
        std::lower_bound(spatial_distance_by_index_.begin(),
                         spatial_distance_by_index_.end(), pre_lowest_s);
    uint32_t r_low = 0;
    if (pre_lowest_itr == spatial_distance_by_index_.end())
    {
      r_low = dimension_s_ - 1;
    }
    else
    {
      r_low = static_cast<uint32_t>(
          std::distance(spatial_distance_by_index_.begin(), pre_lowest_itr));
    }
    const uint32_t r_pre_size = r - r_low + 1;
    const auto &pre_col = cost_table_[c - 1];
    double curr_speed_limit = speed_limit;

    if (c == 2)
    {
      for (uint32_t i = 0; i < r_pre_size; ++i)
      {
        uint32_t r_pre = r - i;
        if (std::isinf(pre_col[r_pre].cost) ||
            pre_col[r_pre].pre_node == nullptr)
        {
          continue;
        }
        // TODO(Jiaxuan): Calculate accurate acceleration by recording speed
        // data in ST point.
        // Use curr_v = (point.s - pre_point.s) / unit_t as current v
        // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
        // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
        // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
        const double curr_a =
            2 *
            ((cost_cr.st_point.s - pre_col[r_pre].st_point.s) / unit_t_ -
             pre_col[r_pre].crusie_velocity) /
            unit_t_;
        if (curr_a < max_deceleration_ || curr_a > max_acceleration_)
        {
          continue;
        }

        if (pre_col[r_pre].crusie_velocity + curr_a * unit_t_ <
                -kDoubleEpsilon &&
            cost_cr.st_point.s > min_s_consider_speed)
        {
          continue;
        }

        // Filter out continuous-time node connection which is in collision with
        // obstacle
        // if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
        //                             pre_col[r_pre])) {
        //   continue;
        // }

        curr_speed_limit =
            std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
        const double cost = cost_cr.ObstacleCost +
                            cost_cr.SpatialPotentialCost +
                            pre_col[r_pre].cost /* +
                          CalculateEdgeCostForThirdCol(
                              r, r_pre, curr_speed_limit, cruise_speed)*/
            ;

        if (cost < cost_cr.cost)
        {
          cost_cr.SetTotalCost(cost);
          cost_cr.SetPreNode(pre_col[r_pre]);
          cost_cr.SetCruiseSpeed(pre_col[r_pre].crusie_velocity +
                                 curr_a * unit_t_);
        }
      }
      return true;
    }

    for (uint32_t i = 0; i < r_pre_size; ++i)
    {
      uint32_t r_pre = r - i;
      if (std::isinf(pre_col[r_pre].cost) ||
          pre_col[r_pre].pre_node == nullptr)
      {
        continue;
      }
      // Use curr_v = (point.s - pre_point.s) / unit_t as current v
      // Use pre_v = (pre_point.s - prepre_point.s) / unit_t as previous v
      // Current acc estimate: curr_a = (curr_v - pre_v) / unit_t
      // = (point.s + prepre_point.s - 2 * pre_point.s) / (unit_t * unit_t)
      const double curr_a =
          2 *
          ((cost_cr.st_point.s - pre_col[r_pre].st_point.s) / unit_t_ -
           pre_col[r_pre].crusie_velocity) /
          unit_t_;
      if (curr_a > max_acceleration_ || curr_a < max_deceleration_)
      {
        continue;
      }

      if (pre_col[r_pre].crusie_velocity + curr_a * unit_t_ < -kDoubleEpsilon &&
          cost_cr.st_point.s > min_s_consider_speed)
      {
        continue;
      }

      // if (CheckOverlapOnDpStGraph(st_graph_data_.st_boundaries(), cost_cr,
      //                             pre_col[r_pre])) {
      //   continue;
      // }

      uint32_t r_prepre = pre_col[r_pre].pre_node->index_s_;
      const StGraphPoint &prepre_graph_point = cost_table_[c - 2][r_prepre];
      if (std::isinf(prepre_graph_point.cost))
      {
        continue;
      }

      if (!prepre_graph_point.pre_node)
      {
        continue;
      }
      const StPoint &triple_pre_point = prepre_graph_point.pre_node->st_point;
      const StPoint &prepre_point = prepre_graph_point.st_point;
      const StPoint &pre_point = pre_col[r_pre].st_point;
      const StPoint &curr_point = cost_cr.st_point;
      curr_speed_limit =
          std::fmin(curr_speed_limit, speed_limit_by_index_[r_pre]);
      double cost = cost_cr.ObstacleCost + cost_cr.SpatialPotentialCost +
                    pre_col[r_pre].cost /* +
                  CalculateEdgeCost(triple_pre_point, prepre_point, pre_point,
                                    curr_point, curr_speed_limit, cruise_speed)*/
          ;

      if (cost < cost_cr.cost)
      {
        cost_cr.SetTotalCost(cost);
        cost_cr.SetPreNode(pre_col[r_pre]);
        cost_cr.SetCruiseSpeed(pre_col[r_pre].crusie_velocity +
                               curr_a * unit_t_);
      }
    }
  }
  return 1;
}

void GetRowRange(StGraphPoint point,
                 size_t next_highest_row,
                 size_t next_lowest_row)
{
  double v0 = 0.0;
  // TODO(all): Record speed information in StGraphPoint and deprecate this.
  // A scaling parameter for DP range search due to the lack of accurate
  // information of the current velocity (set to 1 by default since we use
  // past 1 second's average v as approximation)
  double acc_coeff = 0.5;
  if (!point.pre_node)
  {
    v0 = 0;
  }
  else
  {
    // opitmal_speed;
    v0 = 10;
  }

  const auto max_s_size = dimension_s_ - 1;
  const double t_squared = unit_t_ * unit_t_;
  const double s_upper_bound = v0 * unit_t_ +
                               acc_coeff * max_acceleration_ * t_squared +
                               point.st_point.s;

  const auto next_highest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_upper_bound);
  if (next_highest_itr == spatial_distance_by_index_.end())
  {
    next_highest_row = max_s_size;
  }
  else
  {
    next_highest_row =
        std::distance(spatial_distance_by_index_.begin(), next_highest_itr);
  }

  const double s_lower_bound =
      std::fmax(0.0, v0 * unit_t_ + acc_coeff * max_deceleration_ * t_squared) +
      point.st_point.s;
  const auto next_lowest_itr =
      std::lower_bound(spatial_distance_by_index_.begin(),
                       spatial_distance_by_index_.end(), s_lower_bound);
  if (next_lowest_itr == spatial_distance_by_index_.end())
  {
    next_lowest_row = max_s_size;
  }
  else
  {
    next_lowest_row =
        std::distance(spatial_distance_by_index_.begin(), next_lowest_itr);
  }
}

bool CalculateTotalCost()
{
  size_t next_highest_row = 0;
  size_t next_lowest_row = 0;
  if (0)
  {
    // CalculateTotalCost()的外层循环 // level循环
    for (size_t c = 0; c < cost_table_.size(); ++c)
    {
      size_t highest_row = 0;
      size_t lowest_row = cost_table_.back().size() - 1;

      int count = static_cast<int>(next_highest_row) -
                  static_cast<int>(next_lowest_row) + 1;
      if (count > 0)
      {
        std::vector<std::future<void>> results;
        for (size_t r = highest_row; r <= lowest_row; ++r)
        {
          CalculateCostAt(c, r);
        }

        // CalculateTotalCost()的内层循环
        for (size_t r = next_lowest_row; r <= next_highest_row; ++r)
        {
          const auto &cost_cr = cost_table_[c][r];
          if (cost_cr.cost < std::numeric_limits<double>::infinity())
          {
            size_t h_r = 0;
            size_t l_r = 0;
            GetRowRange(cost_cr, h_r, l_r);
            highest_row = std::max(highest_row, h_r);
            lowest_row = std::min(lowest_row, l_r);
          }
        }
        next_highest_row = highest_row;
        next_lowest_row = lowest_row;
      }
    }
    return true;
  }
}

bool RetrieveSpeedProfile()
{
  // if (0)
  // {
  // StGraphPoint *best_end_point = nullptr;
  // double min_cost = std::numeric_limits<double>::infinity();
  //   for (int i = 0; i < cost_table_.back().size(); i++)
  //   {
  //     if (!std::isinf(cost_table_.back()[i].cost &&
  //                     cost_table_.back()[i].cost < min_cost))
  //     {
  //       best_end_point = &cost_table_.back()[i];
  //       min_cost = cost_table_.back()[i].cost;
  //     }
  //   }
  // }

  // 初始化 cost_table_
  StPoint st_point(0, 0);
  int level_num = 4;       // level的数量
  int num_each_level = 10; // 每个level中点的数目
  // 初始化 cost_table_
  cost_table_ = std::vector<std::vector<StGraphPoint>>(
      level_num, std::vector<StGraphPoint>(num_each_level, StGraphPoint(st_point)));

  std::cout << "cost_table size   " << cost_table_.size() << ",  " << cost_table_[2].size() << std::endl;

  // 赋值 cost_table_
  //
  float dt = 0.5;
  float crusie_velocity = 5;
  float crusie_velocity_interval = crusie_velocity / num_each_level;
  for (int c = 0; c < cost_table_.size(); c++)
  {
    for (int r = 0; r < cost_table_[c].size(); r++)
    {
      // StPoint st_point{float(c), float(r)};
      StPoint st_point{0, 0};
      st_point.t = (float(c) * dt);
      // 速度撒点 乘以 时间
      st_point.s = (float(r) * crusie_velocity_interval * st_point.t);
      // node 赋值
      cost_table_[c][r].Init(c, r, st_point);
      cost_table_[c][r].SetCruiseSpeed(crusie_velocity);
      cost_table_[c][r].SetCruiseSpeed(float(r) * crusie_velocity_interval);
      cost_table_[c][r].current_velocity = (float(r) * crusie_velocity_interval);
      { // debug code
        if (c == 0)
        {
          // 对于第一个level不用考虑上一个level中的最小cost的node
          cost_table_[c][r].cost = double(c + r) + GetObstacleCost(cost_table_[c][r]);
        }
        else
        {
          double min_cost = std::numeric_limits<double>::max();
          for (int i = 0; i <= r; i++)
          {
            if (cost_table_[c - 1][i].cost < min_cost)
            {
              min_cost = cost_table_[c - 1][i].cost;
              // 当前node的cost是 上个level中最小的cost，加状态转移的cost
              cost_table_[c][r].cost = double(c + r) + GetObstacleCost(cost_table_[c][r]) +
                                       min_cost + 10*abs(cost_table_[c][r].current_velocity - crusie_velocity);
            }
          }
        }

        // if (c == 2 && r == 0)
        // {
        //   cost_table_[c][r].cost = 10;
        // }
      }
      // (s,t)
      cost_table_[c][r].st_point = st_point;
    }
  }

  { // 设置 node 的 pre_node
    for (int c = 0; c < cost_table_.size(); c++)
    {
      for (int r = 0; r < cost_table_[c].size(); r++)
      {
        // 对于第一个level的node,没有pre_node
        if (c == 0)
        {
          continue;
        }
        else
        {
          double min_cost = std::numeric_limits<double>::max();
          for (int i = 0; i <= r; i++)
          {
            if (cost_table_[c - 1][i].cost < min_cost)
            {
              min_cost = cost_table_[c - 1][i].cost;
              // 当前node的pre_node一定是上个level中cost最小的node
              cost_table_[c][r].pre_node = &cost_table_[c - 1][i];
            }
          }
        }
      }
    }
  }

  // std::cout << "cost_table_[3][5].pre_node->cost   " << cost_table_[3][5].pre_node->cost
  //           << "   " << cost_table_[3][5].pre_node->st_point.t << " " << cost_table_[3][5].pre_node->st_point.s << std::endl;

  // 找到最后一个level中cost最小的node作为当前node
  StGraphPoint cur_point = cost_table_.back()[5];
  {
    double min_cost = std::numeric_limits<double>::max();
    for (int i = 0; i <= cost_table_.back().size() - 1; i++)
    {
      if (cost_table_.back()[i].cost < min_cost)
      {
        min_cost = cost_table_.back()[i].cost;
        // 当前node的pre_node一定是上个level中cost最小的node
        cur_point = cost_table_.back()[i];
      }
    }
  }

  // { // debug code
  //   for (int c = 0; c < cost_table_.size(); c++)
  //   {
  //     for (int r = 0; r < cost_table_[c].size(); r++)
  //     {
  //       std::cout << "index_s, index_t, cost   " << cost_table_[c][r].index_s_ << "  " << cost_table_[c][r].index_t_ << "   " << cost_table_[c][r].cost << std::endl;
  //     }
  //   }

  // std::cout << "current_node (t s)  ( " << cur_point.index_t_ << "  "
  //           << cur_point.index_s_ << " )"
  //           << "  cost  " << cur_point.cost << std::endl;
  // }

  // cur_point = cost_table_.back()[5];
  // 判断是否找到当前node
  if (&cur_point == nullptr)
  {
    return false;
  }

  std::vector<SpeedPoint> speed_profile;
  int index = 0;
  while (&cur_point != nullptr)
  {
    std::cout << "current_node (t s)  ( " << cur_point.st_point.t << "  "
              << cur_point.st_point.s << " )"
              << "  cost  " << cur_point.cost << std::endl;

    SpeedPoint speed_point(0, 0, 0);
    speed_point.set_s(cur_point.st_point.s);
    speed_point.set_t(cur_point.st_point.t);
    speed_profile.push_back(speed_point);

    if (cur_point.pre_node != nullptr)
    {
      cur_point = *cur_point.pre_node;
    }
    else
    {
      break;
    }

    index++;
  }

  std::reverse(speed_profile.begin(), speed_profile.end());

  // static constexpr double kEpsilon = std::numeric_limits<double>::epsilon();
  // if (speed_profile.front().t > kEpsilon ||
  //     speed_profile.front().s > kEpsilon)
  // {
  //   return false;
  // }

  // for (size_t i = 0; i + 1 < speed_profile.size(); ++i)
  // {
  //   double v = (speed_profile[i + 1].s - speed_profile[i].s) /
  //              (speed_profile[i + 1].t - speed_profile[i].t + 1e-3);
  //   speed_profile[i].set_v(v);
  //   speed_profile[i].v = v;
  //   std::cout << "speed_profile[i + 1].s    " << speed_profile[i + 1].s << std::endl;
  //   std::cout << "speed_profile[i ].s    " << speed_profile[i].s << std::endl;
  //   std::cout << "speed_profile[i + 1].t    " << speed_profile[i + 1].t << std::endl;
  //   std::cout << "speed_profile[i].t    " << speed_profile[i].t << std::endl;
  // }

  // std::cout << "speed_profile.size()  " << speed_profile.size() << std::endl;
  // for (int i = 0; i < speed_profile.size(); i++)
  // {
  //   std::cout << "index, (s,t,v)  " << i << " (" << speed_profile[i].s << " " << speed_profile[i].t << " "
  //             << speed_profile[i].v << ")" << std::endl;
  // }

  // *speed_data = SpeedData(speed_profile);
  return true;
}

bool DP_speed_Search(std::vector<StBoundary> st_boundaries)
{
  static constexpr double kBounadryEpsilon = 1e-2;

  // 遍历的是STGraph中所有的boundary，每一个障碍物都会生成一个对应的boundary，这里就是将所有障碍的boundary都判断一遍
  for (const auto &boundary : st_boundaries)
  {
    // KeepClear obstacles not considered in Dp St decision

    //   // If init point in collision with obstacle, return speed fallback
    //   if (boundary->IsPointInBoundary({0.0, 0.0}) ||
    //       (std::fabs(boundary->min_t()) < kBounadryEpsilon &&
    //        std::fabs(boundary->min_s()) < kBounadryEpsilon))
    //   {
    //     dimension_t_ = static_cast<uint32_t>(std::ceil(
    //                        total_length_t_ / static_cast<double>(unit_t_))) +
    //                    1;
    //     std::vector<SpeedPoint> speed_profile;
    //     double t = 0.0;
    //     for (uint32_t i = 0; i < dimension_t_; ++i, t += unit_t_)
    //     {
    //       speed_profile.push_back(PointFactory::ToSpeedPoint(0, t));
    //     }
    //     *speed_data = SpeedData(speed_profile);
    //     return Status::OK();
    //   }
    // }
  }

  if (0)
  {
    if (!InitCostTable())
    {
      // return false;
    }
  }

  if (!InitSpeedLimitLookUp())
  {
    // return false;
  }

  if (!CalculateTotalCost())
  {
    // return false;
  }

  if (!RetrieveSpeedProfile())
  {
    // return false;
  }

  std::cout << "DP Speed Search " << std::endl;
  return true;
}

int main()
{

  // 加入障碍物
  std::vector<Waypoint> points;
  std::shared_ptr<DiscretizedPath> path_;
  for (double x = 0.0; x < 20; x += 0.1)
  {
    points.emplace_back(x, x, M_PI / 4.0, std::hypot(x, x));
  }
  path_ = std::make_shared<DiscretizedPath>(points);

  Box2d obs_box3({1, 1}, M_PI / 4.0, 1.0, 1.0);
  std::vector<TrajectoryPoint> traj_points;
  double t = 0.0;
  for (double x = 5.0; x > 0.0; x -= 0.1)
  {
    TrajectoryPoint traj_p({x, -x + 5, 3 * M_PI / 4.0, 0}, 0.0, t);
    traj_points.emplace_back(traj_p);
    t += 0.1;
  }

  DiscretizedTrajectory obs_trajectory(traj_points);
  Obstacle obs1("3", obs_box3, false, obs_trajectory);

  Box2d obs_box4({1, 1}, M_PI / 4.0, 1, 1.0);
  Obstacle obs2("4", obs_box4, true);

  StGraph st_graph(*path_, std::vector<Obstacle>{obs1, obs2});

  st_graph.GetAllObstacleStBoundary(&st_boundaries);

  //////////////////////////////////////////////////////
  // std::vector<StBoundary> st_boundaries_debug;
  // StBoundary st_boundary_1;
  // StBoundary st_boundary_2;
  // StBoundary st_boundary_3;

  // StPoint lower_point(0.0, 0.0);
  // StPoint upper_point(1.0, 1.0);
  // std::vector<StPoint> lower_points;
  // std::vector<StPoint> upper_points;

  // lower_points.push_back(lower_point);
  // upper_points.push_back(upper_point);

  // st_boundary_1.Init("1", lower_points, upper_points);
  // st_boundary_1.Init("2", lower_points, upper_points);
  // st_boundary_1.Init("3", lower_points, upper_points);

  // st_boundaries_debug.push_back(st_boundary_1);
  // st_boundaries_debug.push_back(st_boundary_2);
  // st_boundaries_debug.push_back(st_boundary_3);
  // DP用于速度规划
  DP_speed_Search(st_boundaries);
  std::cout << "End DP_speed " << std::endl;
  return 0;
}
