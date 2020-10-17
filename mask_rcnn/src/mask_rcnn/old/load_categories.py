
cat_2017 = '../../data/annotations/instances_val2017.json'

import sys, getopt
import json

json_file = cat_2017    
if json_file is not None:
    with open(json_file,'r') as COCO:
        js = json.loads(COCO.read())
        print(json.dumps(js['categories']))