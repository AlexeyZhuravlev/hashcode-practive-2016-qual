import os
import pandas as pd
import numpy as np

def analyze_file(f):
    pass

def analyze():
    for filename in os.listdir("test"):
        if filename.endswith(".in"):
            with open(os.path.join('test', filename)) as f:
                print("-" * 30)
                print("Stats for {}".format(filename))
                analyze_file(f)

if __name__ == "__main__":
    analyze()
