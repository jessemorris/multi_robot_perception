#include "vdo_slam/utils/statistics.h"
#include "vdo_slam/utils/seralization.h"
#include "vdo_slam/Macros.h"

#include <iostream>
#include <string>
#include <sstream> 


namespace VDO_SLAM {

    std::vector<StatisticsBase*> WriterObserver::statistics_obsevers{};


    void WriterObserver::addObserver(StatisticsBase* stats) {
        statistics_obsevers.push_back(stats);
    }

    void WriterObserver::write() {

        for(auto base : statistics_obsevers) {
            if (base) {
                base->implWrite();
            }
        }
    }

    StatisticsManager::StatisticsManager(bool should_write_)
        :   should_write(should_write_) {

            VDO_INFO_MSG("Results dir: " << RESULTS_DIR << " and writing set to: " << should_write);
            WriterObserver::addObserver(this);

        }

    StatisticsManager::StatisticsManager(const VdoParams& params_, bool should_write_)
        :   should_write(should_write_) {

            VDO_INFO_MSG("Results dir: " << RESULTS_DIR << " and writing set to: " << should_write);
            system_node_["params"] = params_;
            params = params_;
            WriterObserver::addObserver(this);

        }

    void StatisticsManager::logScene(const Scene& scene) {
        map_node_["scenes"].push_back(scene);
    }

    void StatisticsManager::logOdom(const Odometry& odom) {
        additional_["odom_gt"].push_back(odom);
    }


    void StatisticsManager::printStatistics() {
        if (!system_node_["params"].IsDefined()) {
            VDO_ERROR_MSG("Cannot print params becuase they are not set");
            return;
        }

        std::stringstream out;
        out << "Statistics\n";

        out << "fx: " << params.fx << "\n";
        out << "fy: " << params.fy << "\n";
        out << "cx: " << params.cx << "\n";

        std::string out_string = out.str();

        VDO_INFO_MSG(out_string);
    }

    
    void StatisticsManager::implWrite() {
        if (should_write) {
            std::string system_file = RESULTS_DIR + "system.yaml";
            std::string map_file = RESULTS_DIR + "map.yaml";
            std::string additional_file = RESULTS_DIR + "add.yaml";

            std::ofstream fout(system_file);
            fout << system_node_;
            VDO_INFO_MSG("Written data to " << system_file);

            std::ofstream fout_map(map_file);
            fout_map << map_node_;
            VDO_INFO_MSG("Written data to " << map_file);

            std::ofstream fout_add(additional_file);
            fout_add << additional_;
            VDO_INFO_MSG("Written data to " << additional_file);

        }

    }

    Logger::Logger(const std::string& name) {

        VDO_INFO_MSG("Making logger: " << name);
        file_path = RESULTS_DIR + name + ".txt";
        file.open (file_path);

        WriterObserver::addObserver(this);

    }

    void Logger::implWrite() {
        VDO_INFO_MSG("Written data to " << file_path);
        file.close();
    } 

    void Logger::addLog(const std::string& info) {
        file << info << "\n";
    }

    // void operator<<(std::ostream& os, Logger& logger) {
    //     std::stringstream ss;
    //     ss << os.rdbuf();
    //     std::string str = ss.str();
    //     logger.file << str << "\n";
    // }

    // void Logger::operator<<(const std::string& info) {

    // }

}


