#include "scenario_generator.h"

void ScenarioGenerator::generate(const ScenarioNumber& scenario_num,
                                 ScenarioData& scenario_data)
{
    if(scenario_num==Normal)
        return generateNormalScenario(scenario_data);

}

void ScenarioGenerator::generateNormalScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 5.0;
    scenario_data.a0 = 1.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = 6.0*i*scenario_data.dt_;
        scenario_data.max_position_[i] = 20 + 5.0*i*scenario_data.dt_;
        scenario_data.min_position_[i] = -40 + 7.0*i*scenario_data.dt_;
    }
}


