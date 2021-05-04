import yaml



def load_from_file(file_path):
    try:
        print("Loading config from {}".format(file_path))
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except Exception as e:
        print("Could not load file from {}".format((file_path)))
        raise e



class OutputResultsLoader:

    def __init__(self, results_folder: str):
        self._results_folder = results_folder
        self._gt_odom_dict = {}
        self._map_dict = {}
        
        print("Loading from root folder: {}".format(self._results_folder))
        self._odom_gt_file = self._results_folder + "/add.yaml"
        self._map_file = self._results_folder + "/map.yaml"

        self._gt_odom_dict = load_from_file(self._odom_gt_file)
        self._map_dict = load_from_file(self._map_file)


    def getOdomGtDict(self):
        return self._gt_odom_dict

    def getMapDict(self):
        return self._map_dict
