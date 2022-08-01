#include "utils.h"

namespace Utils
{
    void outputObsToFile(const std::string& filename,
                         const Obstacle& obs)
    {
        double t0 = 0.0;

        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "obs_time" <<  "," << "obs_s" << std::endl;

        writing_file << obs.t_.first  << "," << obs.s_.first  << std::endl;
        writing_file << obs.t_.second << "," << obs.s_.second << std::endl;
        writing_file.close();
    }

    void outputToFile(const std::string& filename,
                      const std::vector<double>& ref_position,
                      const std::vector<double>& max_position,
                      const std::vector<double>& min_position,
                      const BaseSolver::OutputInfo& qp_output)
    {
        
        
        std::ofstream writing_file;
        writing_file.open(filename, std::ios::out);
        writing_file << "ref_position" <<  "," << "max_position" << "," << "min_position" << "," << "opt_position" << ","
                     << "opt_velocity" <<  "," << "opt_acceleration" << "," << "opt_jerk" << "," << "time" << ","
                     << std::endl;

        for(int i=0; i<ref_position.size(); ++i)
        {
            writing_file << ref_position[i] << "," << max_position[i] << "," << min_position[i] << "," << qp_output.position[i] << ","
                         << qp_output.velocity[i] << "," << qp_output.acceleration[i] << "," << qp_output.jerk[i] << ","
                         << qp_output.time[i] << "," 
                         << std::endl;
        }

        writing_file.close();
    }
}