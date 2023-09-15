import csv
import math
import matplotlib.pyplot as plt

# Sample of ramp.csv:
# 58
# 57
# 56
# 57
# 57
# 58
# 57
# 58
# 59
# 59


def plot_data(data):
    # Data is sampled every 1 us, time units should be in milliseconds
    time = [x * 1e-3 for x in range(len(data))]
    plt.plot(time, data)

    # Zero the time axis
    plt.xlim(0, max(time) + 2)

    plt.xlabel("Time (ms)")
    plt.ylabel("ADC Value")
    #plt.show()
    plt.savefig("ramp.png")


def main():
    with open(file="ramp.csv", mode="r") as in_file:
        # Read data into a list
        data = in_file.readlines()

        # Convert strings to integers
        data = [int(x) for x in data]

    # Find the maximum value
    max_value = max(data)

    # Find the minimum value
    min_value = min(data)

    # Find the range
    range_value = max_value - min_value + 1

    # Calculate the number of ADC levels
    num_levels = 2 ** 8

    # Calculate the resolution
    res = math.ceil(math.log2(range_value))

    # Print the results
    print("Maximum value: {}".format(max_value))
    print("Minimum value: {}".format(min_value))
    print("Range: {}".format(range_value))
    print("Number of levels: {}".format(num_levels))
    print("ENOB: {}".format(res))

    # Plot the data
    plot_data(data)


if __name__ == '__main__':
    main()
