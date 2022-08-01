#include "scenario_generator.h"

void ScenarioGenerator::generate(const ScenarioNumber& scenario_num,
                                 ScenarioData& scenario_data)
{
    if(scenario_num==Normal)
        return generateNormalScenario(scenario_data);
    else if(scenario_num==Cutin)
        return generateCutinScenario(scenario_data);
    else if(scenario_num==Cutout)
        return generateCutoutScenario(scenario_data);
    else if(scenario_num==Stop)
        return generateStopScenario(scenario_data);
    else if(scenario_num==Start)
        return generateStartScenario(scenario_data);
    else 
        return generateAccScenario(scenario_data);

}

void ScenarioGenerator::generateNormalScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 15.0;
    scenario_data.a0_ = 0.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = 8*i*scenario_data.dt_ ;

        scenario_data.max_position_[i] = 30 + 22.0*i*scenario_data.dt_;
        scenario_data.min_position_[i] = -40 + 18.0*i*scenario_data.dt_;


        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);


    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }
}


void ScenarioGenerator::generateCutinScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 20.0;
    scenario_data.a0_ = 1.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        
        scenario_data.min_position_[i] = -60 + 20.0*i*scenario_data.dt_;

        if (i < 0.5*(scenario_data.N_ + 1)){
            scenario_data.max_position_[i] = 40 + 22.0*i*scenario_data.dt_;
        }
        else{
            scenario_data.max_position_[i] = 30 + 15.0*i*scenario_data.dt_;
        }

        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);
    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }
}

void ScenarioGenerator::generateCutoutScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 22.0;
    scenario_data.a0_ = -1.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        
        scenario_data.min_position_[i] = -60 + 20.0*i*scenario_data.dt_;

        if (i < 0.5*(scenario_data.N_ + 1)){
            scenario_data.max_position_[i] = 40 + 15.0*i*scenario_data.dt_;
        }
        else{
            scenario_data.max_position_[i] = 30 + 20.0*i*scenario_data.dt_;
        }

        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);
    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }
}


void ScenarioGenerator::generateStopScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 20.0;
    scenario_data.a0_ = 0.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        
        scenario_data.min_position_[i] = 0.0;

        scenario_data.max_position_[i] = 40 + 20.0*i*scenario_data.dt_-1*(i*scenario_data.dt_)*(i*scenario_data.dt_);
        //scenario_data.min_position_[i] = -60 + 24.0*i*scenario_data.dt_;

        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);
    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }
}

void ScenarioGenerator::generateStartScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 0.0;
    scenario_data.a0_ = 0.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        
        scenario_data.min_position_[i] = 0.0;

        scenario_data.max_position_[i] = 10 + 1.2*(i*scenario_data.dt_)*(i*scenario_data.dt_);
        //scenario_data.min_position_[i] = -60 + 24.0*i*scenario_data.dt_;



        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);
    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }
}

void ScenarioGenerator::generateAccScenario(ScenarioData &scenario_data)
{
    const double t_tol = 10.0;
    scenario_data.N_ = 51;
    scenario_data.v0_ = 18.0;
    scenario_data.a0_ = 0.0;

    scenario_data.dt_ = t_tol/(scenario_data.N_ - 1);

    scenario_data.ref_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.max_position_ = std::vector<double>(scenario_data.N_,0.0);
    scenario_data.min_position_ = std::vector<double>(scenario_data.N_,0.0);

    for(int i=0; i<scenario_data.N_; i++){
        
        scenario_data.min_position_[i] = 0.0;

        scenario_data.max_position_[i] = 30 + 15*(i*scenario_data.dt_);
        //scenario_data.min_position_[i] = -60 + 24.0*i*scenario_data.dt_;


        //double final_s = 0.5*(scenario_data.min_position_[scenario_data.N_-1]);
        scenario_data.ref_position_[i] =  0.5*(scenario_data.min_position_[i] + scenario_data.max_position_[i]);
    }

    double initial_pos = scenario_data.ref_position_[0];
    for(int i=0; i<scenario_data.N_; i++){
        scenario_data.ref_position_[i] = scenario_data.ref_position_[i] - initial_pos;
    }

    
}


