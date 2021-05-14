#ifndef VDO_SLAM_STATISTICS_H
#define VDO_SLAM_STATISTICS_H

#include "vdo_slam/Params.h"
#include "vdo_slam/Scene.h"
#include "vdo_slam/utils/VdoUtils.h"
#include "vdo_slam/definitions.h"

#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <iostream>
#include <memory>


namespace VDO_SLAM {


    const std::string RESULTS_DIR = std::string(__VDO_SLAM_DIR__) + "output_results/";


    class StatisticsBase {
        public:
            StatisticsBase() {};
            virtual ~StatisticsBase() = default;

            virtual void implWrite() = 0;

        protected:
            YAML::Node base_node_;

    };

    class WriterObserver {

        public: 
            static void write();
            // static void addObserver(std::shared_ptr<StatisticsBase> stats);
            static void addObserver(StatisticsBase* stats);

        private:
            // static std::vector<std::shared_ptr<StatisticsBase>> statistics_obsevers;
            static std::vector<StatisticsBase*> statistics_obsevers;

    };

    //TODO: make logger unique (singleton) so only one can be created with the same
    //name to avoid overwriting
    class Logger : public StatisticsBase, public std::enable_shared_from_this< Logger> {

        public:
            Logger(const std::string& name);
            ~Logger() = default;

            //this type of operatr woudl be cool but more complicated than i thought to get working.
            //easily solution for now
            // void operator<<(const std::string& info) {
            //     file << info << "\n";
            // }

            // void operator<<(const double info) {
            //     file << std::to_string(info) << "\n";
            // }

            void addLog(const std::string& info);

        private:
            std::string file_path;
            std::ofstream file;
            void implWrite() override;


    };


    class StatisticsManager : public StatisticsBase, public std::enable_shared_from_this< StatisticsManager> {

        public:

            StatisticsManager(bool should_write_);
            StatisticsManager(const VdoParams& params_, bool should_write_);
            ~StatisticsManager() = default;

            void logScene(const Scene& scene);

            //for now this is just the gt odom we get from the bagfiles
            void logOdom(const Odometry& odom);

            void printStatistics();

        private:
            void implWrite() override;

            //node to store system level information such as params, frequency etc
            YAML::Node system_node_;
            //node to store Scene/SceneObject information
            YAML::Node map_node_;
            //currently we will just use this for gt odom data so we can better do RMSE 
            YAML::Node additional_;

            bool should_write;

            //we save an extra params here so we dont have to write an decode function
            //when we want to print it
            VdoParams params;




    };

    typedef std::shared_ptr<StatisticsManager> StatisticsManagerPtr;
    typedef std::unique_ptr<StatisticsManager> StatisticsManagerUniquePtr;



}



#endif