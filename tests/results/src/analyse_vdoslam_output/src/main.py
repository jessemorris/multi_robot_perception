from loader import OutputResultsLoader


if __name__ == "__main__":
    results_loader = OutputResultsLoader("/home/jesse/Code/src/ros/src/multi_robot_perception/VDO_SLAM/output_results")
    print(results_loader.getMapDict())