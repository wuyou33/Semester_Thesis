import CF_functions as cff
import csv
from matplotlib import pyplot as plt

# Decode binary log file
log = cff.decode('log_data/log_thrust_est_4')
csv_file = 'csv_data/log_thrust_est_4.csv'

# Begin writing to csv
with open(csv_file, mode='w') as file:
    wrtr = csv.writer(file, delimiter=',')

    # Extract key names as a list
    keys = list(log.keys())

    # Write first row with names
    wrtr.writerow(keys)

    for cols in  range(len(list(log['tick']))):
        row_list = []

        # Extract a value from every key in a row and append it to the list
        for key in keys:
            row_list.append(log[key][cols])

        # Write row to csv file
        wrtr.writerow(row_list)
