import os

import pandas as pd
from matplotlib import pyplot as plt


PATH = r"C:\Users\Ben2020\Documents\GitHub\pressure_transducer\code\scripts\test_data"

def read_csvs(path_to_dir):
    if not path_to_dir.endswith("\\"):
        path_to_dir += "\\"

    datasets = []
    for file_name in os.listdir(path_to_dir):
        head, extension = os.path.splitext(file_name)
        
        if extension == ".csv":
            data = {}
            data["sensorType"], data["startDate"], data["startTime"] = head.split("-")

            with open(path_to_dir + file_name, "r") as file:
                settings = file.readline().strip()
                values = file.readline().strip()
                
                for s, v in zip(settings.split(","), values.split(",")):
                    data[s] = v

            data["data"] = pd.read_csv(path_to_dir + file_name, header=2)
            datasets.append(data)
    return datasets



def main():
    datasets = read_csvs(PATH)

    print(datasets[0]["data"].describe())

    datasets[0]["data"].plot(x="time", y="pressure")

    plt.show()
    


if __name__ == "__main__":
    main()