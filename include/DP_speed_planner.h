
#include <array>
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <string>
#include <memory>
#include <future> // std::async, std::future
#include <chrono> // std::chrono::milliseconds
#include <cmath>

class Box2d
{
public:
    Box2d(const std::vector<double> &center, const double heading, const double length,
          const double width)
    {
        // std::cout << "Box2d  " << center[0] << "  " << center[1] << std::endl;
        center_ = (center);
        length_ = (length);
        width_ = (width);
        half_length_ = (length / 2.0);
        half_width_ = (width / 2.0);
        heading_ = (heading);
        cos_heading_ = (cos(heading));
        sin_heading_ = (sin(heading));
        InitCorners();
    }

    double center_x()
    {
        return center_[0];
    }

    double center_y()
    {
        return center_[1];
    }

    bool HasOverlap(Box2d box)
    {
    }
    double DistanceTo(Box2d &point)
    {

        double x_diff = center_[0] - point.center_[0];
        double y_diff = center_[1] - point.center_[1];

        double distance = hypot(y_diff, x_diff);

        // distance = 0.5;
        return distance;
    }

    void InitCorners()
    {
        const double dx1 = cos_heading_ * half_length_;
        const double dy1 = sin_heading_ * half_length_;
        const double dx2 = sin_heading_ * half_width_;
        const double dy2 = -cos_heading_ * half_width_;
        corners_.clear();

        std::vector<double> corner_point_1{center_[0] + dx1 + dx2, center_[1] + dy1 + dy2};
        std::vector<double> corner_point_2{center_[0] + dx1 - dx2, center_[1] + dy1 - dy2};
        std::vector<double> corner_point_3{center_[0] - dx1 - dx2, center_[1] - dy1 - dy2};
        std::vector<double> corner_point_4{center_[0] - dx1 + dx2, center_[1] - dy1 + dy2};

        corners_.push_back(corner_point_1);
        corners_.push_back(corner_point_2);
        corners_.push_back(corner_point_3);
        corners_.push_back(corner_point_4);

        for (auto &corner : corners_)
        {
            max_x_ = std::fmax(corner[0], max_x_);
            min_x_ = std::fmin(corner[0], min_x_);
            max_y_ = std::fmax(corner[1], max_y_);
            min_y_ = std::fmin(corner[1], min_y_);
        }
    }
    std::vector<double> center_;
    double length_ = 0.0;
    double width_ = 0.0;
    double half_length_ = 0.0;
    double half_width_ = 0.0;
    double heading_ = 0.0;
    double cos_heading_ = 1.0;
    double sin_heading_ = 0.0;

    double max_x_ = std::numeric_limits<double>::lowest();
    double min_x_ = std::numeric_limits<double>::max();
    double max_y_ = std::numeric_limits<double>::lowest();
    double min_y_ = std::numeric_limits<double>::max();

    std::vector<std::vector<double>> corners_;
};

// 数据部分
struct Waypoint
{
    Waypoint(const double _x, const double _y, const double _theta, const double _s)
        : x(_x), y(_y), theta(_theta), s(_s)
    {
    }
    double x;
    double y;
    double theta;
    double s;
};

struct DiscretizedPath
{
    explicit DiscretizedPath(const std::vector<Waypoint> &_waypoints) : points(_waypoints)
    {
    }
    DiscretizedPath() = default;
    std::vector<Waypoint> points;
};

struct TrajectoryPoint
{
    TrajectoryPoint(const Waypoint &_waypoint, const double _v, const double _relative_time)
        : waypoint(_waypoint), v(_v), relative_time(_relative_time)
    {
    }
    Waypoint waypoint;
    double v;
    double relative_time;
};

struct DiscretizedTrajectory
{
    explicit DiscretizedTrajectory(const std::vector<TrajectoryPoint> &_traj_points)
        : traj_points(_traj_points)
    {
    }
    DiscretizedTrajectory() = default;
    std::vector<TrajectoryPoint> traj_points;
};

// 障碍物
struct Obstacle
{
    Obstacle(const std::string &_id, const Box2d &_bounding_box, const bool _is_static,
             const DiscretizedTrajectory &_trajectory)
        : id(_id), bounding_box(_bounding_box), is_static(_is_static), trajectory(_trajectory)
    {
    }
    Obstacle(const std::string &_id, const Box2d &_bounding_box, const bool _is_static)
        : id(_id), bounding_box(_bounding_box), is_static(_is_static)
    {
    }
    std::string id;
    Box2d bounding_box;
    bool is_static;
    DiscretizedTrajectory trajectory;
};

// StBoundary
struct SpeedPoint
{
    SpeedPoint(const double _t, const double _s, double _v) : t(_t), s(_s), v(_v)
    {
    }

    void set_s(std::uint32_t s_)
    {
        s = s_;
    }

    void set_t(std::uint32_t t_)
    {
        t = t_;
    }

    void set_v(std::uint32_t v_)
    {
        v = v_;
    }

    double t;
    double s;
    double v;
};

// StBoundary
struct StPoint
{
    StPoint(float _t, float _s) : t(_t), s(_s)
    {
    }

    void set_s(float s_)
    {
        s = s_;
    }

    void set_t(float t_)
    {
        t = t_;
    }

    float t;
    float s;
};

struct SlPoint
{
    SlPoint(float _s, float _l) : s(_s), l(_l)
    {
    }
    float s;
    float l;
};

struct StGraphPoint
{
    StGraphPoint(StPoint &_st_point)
        : st_point(_st_point), cost(std::numeric_limits<double>::max())
    {
    }

    void Init(float index_t,
              float index_s, StPoint st_point)
    {
        index_t_ = index_t;
        index_s_ = index_s;
        st_point = st_point;
    }

    void SetPreNode(StGraphPoint pre_node_)
    {
        pre_node = &pre_node_;
    }

    StPoint st_point;

    // , pre_node(nullptr)
    StGraphPoint *pre_node = nullptr;

    void SetTotalCost(double cost_)
    {
        cost = cost_;
    }
    double cost;

    void SetObstacleCost(double ObstacleCost_)
    {
        ObstacleCost = ObstacleCost_;
    }
    double ObstacleCost;

    void SetSpatialPotentialCost(double SpatialPotentialCost_)
    {
        ObstacleCost = SpatialPotentialCost_;
    }
    double SpatialPotentialCost;

    float index_s_ = 0;
    float index_t_ = 0;

    double crusie_velocity;
    double current_velocity;

    void SetCruiseSpeed(double crusie_velocity_)
    {
        crusie_velocity = crusie_velocity_;
    }

    void SetCurrentSpeed(double current_velocity_)
    {
        current_velocity = current_velocity_;
    }
};

class StBoundary
{
public:
    StBoundary();

    void Init(std::string obs_id, std::vector<StPoint> lower_points,
              std::vector<StPoint> upper_points);

    bool Evaluate(const double t, double *const lower_s, double *const upper_s) const;

    std::string id() const
    {
        return obs_id_;
    }

public:
    bool is_init_ = false;
    std::string obs_id_;
    std::vector<StPoint> lower_points_;
    std::vector<StPoint> upper_points_;
};

StBoundary::StBoundary()
{
    is_init_ = false;
}

void StBoundary::Init(std::string obs_id, std::vector<StPoint> lower_points,
                      std::vector<StPoint> upper_points)
{
    // CHECK_EQ(lower_points.size(), upper_points.size());
    // CHECK_GE(lower_points.size(), 2);

    obs_id_ = obs_id;
    lower_points_ = lower_points;
    upper_points_ = upper_points;
    is_init_ = true;
}

bool StBoundary::Evaluate(const double t, double *const lower_s, double *const upper_s) const
{
    // CHECK_NOTNULL(lower_s);
    // CHECK_NOTNULL(upper_s);
    // CHECK(is_init_);
    if (t < lower_points_.front().t)
    {
        return false;
    }
    if (t > lower_points_.back().t)
    {
        return false;
    }

    auto func = [](const StPoint &st_point, const double t) { return st_point.t < t; };
    const auto &lower_iter = std::lower_bound(lower_points_.begin(), lower_points_.end(), t, func);
    if (lower_iter != lower_points_.end())
    {
        *lower_s = lower_iter->s;
    }
    else
    {
        return false;
    }
    const auto &upper_iter = std::lower_bound(upper_points_.begin(), upper_points_.end(), t, func);
    if (upper_iter != upper_points_.end())
    {
        *upper_s = upper_iter->s;
    }
    else
    {
        return false;
    }
    return true;
}

// S-T Graph
class StGraph final
{
public:
    StGraph(const DiscretizedPath &path, const std::vector<Obstacle> &obstacles);

    bool GetAllObstacleStBoundary(std::vector<StBoundary> *const st_boundaries) const;

    bool GetObstacleStBoundary(const std::string &obs_id, StBoundary *const st_boundary) const;

    bool GetAllBlockedSRangesByT(const double t,
                                 std::vector<std::pair<double, double>> *const blocked_ranges) const;

    bool GetBlockedSRangeByT(const double t, const StBoundary &st_boundary,
                             std::pair<double, double> *blocked_range) const;

private:
    void CalculateAllObstacleStBoundary(const std::vector<Obstacle> &obstacles);

    bool CalculateObstacleStBoundary(const Obstacle &obs, StBoundary *const st_boundary) const;

    bool CalculateStaticObstacleStBoundary(const std::string &obs_id, const Box2d &obs_box,
                                           StBoundary *const st_boundary) const;

    bool CalculateDynamicObstacleStBoundary(const Obstacle &obs, StBoundary *const st_boundary) const;

    std::vector<Waypoint> GetWaypointsWithinDistance(const double x, const double y,
                                                     const double radius) const;

    bool GetBlockedSRange(const Box2d &obs_box, double *const lower_s, double *const upper_s) const;

private:
    std::unordered_map<std::string, StBoundary> obs_st_boundary_;
    DiscretizedPath discretized_path_;

    const double veh_heading_ = M_PI / 4.0;
    const double veh_length_ = 2.0;
    const double veh_width_ = 1.0;
    const double time_range_ = 4.0;
};

StGraph::StGraph(const DiscretizedPath &path, const std::vector<Obstacle> &obstacles)
    : discretized_path_(path)
{
    // CHECK_GE(discretized_path_.points.size(), 2);

    obs_st_boundary_.clear();
    CalculateAllObstacleStBoundary(obstacles);
}

bool StGraph::GetAllObstacleStBoundary(std::vector<StBoundary> *const st_boundaries) const
{
    // CHECK_NOTNULL(st_boundaries);

    for (auto iter = obs_st_boundary_.begin(); iter != obs_st_boundary_.end(); ++iter)
    {
        st_boundaries->emplace_back(iter->second);
    }
    return true;
}

bool StGraph::GetObstacleStBoundary(const std::string &obs_id,
                                    StBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    const auto &iter = obs_st_boundary_.find(obs_id);
    if (iter == obs_st_boundary_.end())
    {
        return false;
    }
    *st_boundary = iter->second;
    return true;
}

bool StGraph::GetAllBlockedSRangesByT(
    const double t, std::vector<std::pair<double, double>> *const blocked_ranges) const
{
    // CHECK_NOTNULL(blocked_ranges);
    std::vector<StBoundary> st_boundaries;
    GetAllObstacleStBoundary(&st_boundaries);

    for (const auto &st_boundary : st_boundaries)
    {
        std::pair<double, double> blocked_range;
        if (GetBlockedSRangeByT(t, st_boundary, &blocked_range))
        {
            blocked_ranges->emplace_back(blocked_range);
        }
    }
    return true;
}

bool StGraph::GetBlockedSRangeByT(const double t, const StBoundary &st_boundary,
                                  std::pair<double, double> *blocked_range) const
{
    // CHECK_NOTNULL(blocked_range);
    double lower_s = 0.0;
    double upper_s = 0.0;
    if (st_boundary.Evaluate(t, &lower_s, &upper_s))
    {
        *blocked_range = std::make_pair(lower_s, upper_s);
        return true;
    }
    return false;
}

void StGraph::CalculateAllObstacleStBoundary(const std::vector<Obstacle> &obstacles)
{
    for (const auto &obstacle : obstacles)
    {
        const auto &iter = obs_st_boundary_.find(obstacle.id);
        // CHECK(iter == obs_st_boundary_.end());
        StBoundary st_boundary;
        if (!CalculateObstacleStBoundary(obstacle, &st_boundary))
        {
            // AWARN << "Failed to get obstacle: " << obstacle.id << " st_boundary. ";
            continue;
        }
        obs_st_boundary_.insert(std::make_pair(obstacle.id, st_boundary));
    }
}

bool StGraph::CalculateObstacleStBoundary(const Obstacle &obstacle,
                                          StBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    if (obstacle.is_static)
    {
        return CalculateStaticObstacleStBoundary(obstacle.id, obstacle.bounding_box, st_boundary);
    }
    else
    {
        return CalculateDynamicObstacleStBoundary(obstacle, st_boundary);
    }
}

bool StGraph::CalculateStaticObstacleStBoundary(const std::string &obs_id, const Box2d &obs_box,
                                                StBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);

    double lower_s = 0.0;
    double upper_s = 0.0;
    if (!GetBlockedSRange(obs_box, &lower_s, &upper_s))
    {
        return false;
    }
    std::vector<StPoint> lower_points{StPoint(0.0, lower_s), StPoint(time_range_, lower_s)};
    std::vector<StPoint> upper_points{StPoint(0.0, upper_s), StPoint(time_range_, upper_s)};
    st_boundary->Init(obs_id, lower_points, upper_points);
    return true;
}

bool StGraph::GetBlockedSRange(const Box2d &obs_box, double *const lower_s,
                               double *const upper_s) const
{
    // CHECK_NOTNULL(lower_s);
    // CHECK_NOTNULL(upper_s);

    const double radius = 5.0;
    double radiAus = 2;
    const auto &points = GetWaypointsWithinDistance(obs_box.center_[0], obs_box.center_[1], radiAus);
    if (points.empty())
    {
        // ADEBUG << "Obstacle is not on the path. ";
        return false;
    }

    bool is_updated = false;

    for (auto iter = points.begin(); iter != points.end(); ++iter)
    {
        Box2d veh_box({iter->x, iter->y}, veh_heading_, veh_length_, veh_width_);
        if (veh_box.HasOverlap(obs_box))
        {
            iter = iter == points.begin() ? iter : iter - 1;
            *lower_s = iter->s;
            is_updated = true;
            break;
        }
    }

    if (!is_updated)

    {
        // ADEBUG << "There is no collision with obstacle. ";
        return false;
    }

    for (auto iter = points.rbegin(); iter != points.rend(); ++iter)
    {
        Box2d veh_box({iter->x, iter->y}, veh_heading_, veh_length_, veh_width_);
        if (veh_box.HasOverlap(obs_box))
        {
            iter = iter == points.rbegin() ? iter : iter - 1;
            *upper_s = iter->s;
            break;
        }
    }
    // CHECK_GE(*upper_s, *lower_s) << "upper_s: " << *upper_s << ", lower_s: " << *lower_s;
    return true;
}

bool StGraph::CalculateDynamicObstacleStBoundary(const Obstacle &obstacle,
                                                 StBoundary *const st_boundary) const
{
    // CHECK_NOTNULL(st_boundary);
    // CHECK_GE(obstacle.trajectory.traj_points.size(), 2);

    bool has_st_boundary = false;
    std::vector<StPoint> lower_points;
    std::vector<StPoint> upper_points;
    for (const auto &p : obstacle.trajectory.traj_points)
    {
        Box2d obs_box({p.waypoint.x, p.waypoint.y}, p.waypoint.theta, obstacle.bounding_box.length_,
                      obstacle.bounding_box.width_);
        double lower_s = 0.0;
        double upper_s = 0.0;
        if (GetBlockedSRange(obs_box, &lower_s, &upper_s))
        {
            lower_points.emplace_back(p.relative_time, lower_s);
            upper_points.emplace_back(p.relative_time, upper_s);
            has_st_boundary = true;
        }
    }
    st_boundary->Init(obstacle.id, lower_points, upper_points);
    return has_st_boundary;
}

std::vector<Waypoint> StGraph::GetWaypointsWithinDistance(const double x, const double y,
                                                          const double radius) const
{
    std::vector<Waypoint> nearest_points;
    for (const auto &p : discretized_path_.points)
    {
        if (std::hypot(p.x - x, p.y - y) < radius)
        {
            nearest_points.emplace_back(p);
        }
    }
    return nearest_points;
}

void DisplayStGraph(const std::vector<StBoundary> &st_boundaries)
{
}
