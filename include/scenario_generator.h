#ifndef FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H
#define FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H

#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <vector>
#include "obstacle.h"

class ScenarioGenerator
{
public:
    enum ScenarioNumber
    {
        Normal = 0,
        Cutin = 1,
        Cutout = 2,
        Stop = 3,
        Start = 4,
        Acc = 5,
        //Accelerate = 1,
        //Stop = 2,
        //Wait = 3,

    };

    struct ScenarioData
    {
        // 1. Velocity
        int N_;
        double v0_;
        double a0_;
        double dt_;
        double refv_;


        std::vector<double> ref_position_;
        std::vector<double> max_position_;
        std::vector<double> min_position_;
        
    };


    ScenarioGenerator() = default;
    ~ScenarioGenerator() = default;

    void generate(const ScenarioNumber& scenario_num, ScenarioData& scenario_data);
    void generateNormalScenario(ScenarioData& scenario_data);
    void generateCutinScenario(ScenarioData& scenario_data);
    void generateCutoutScenario(ScenarioData& scenario_data);
    void generateStopScenario(ScenarioData& scenario_data);
    void generateStartScenario(ScenarioData& scenario_data);
    void generateAccScenario(ScenarioData& scenario_data);
    
};

#endif //FILTER_POSITION_OPTIMIZATION_SCENARIO_GENERATOR_H
