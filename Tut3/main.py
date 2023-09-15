import math
import matplotlib.pyplot as plt


def calculate_dc_offset_error():
    with open(file="zero.csv", mode="r") as in_file:
        # Read data into a list
        data = in_file.readlines()

        # Convert strings to integers
        data = [int(x) for x in data]

    # Calculate the average
    avg = sum(data) / len(data)

    # Calculate the DC offset error
    dc_offset_error = round(abs(avg - 0), 3)

    # Calculate DC offset error in volts
    dc_offset_error_volts = round(dc_offset_error * 0.0109, 3)

    print("DC offset error: {}".format(dc_offset_error))
    print("DC offset error (volts): {} V".format(dc_offset_error_volts))

    # Change size of plot
    plt.rcParams["figure.figsize"] = (20, 10)

    # Data is sampled every 1 us, time units should be in milliseconds
    time = [x * 1e-3 for x in range(len(data))]

    plt.plot(time, data)
    plt.xlim(0, 1.2)  # Zero the time axis and zoom in

    plt.xlabel("Time (ms)")
    plt.ylabel("ADC Value")
    # plt.show()
    plt.savefig("zero.png")
    plt.close()


def calculate_resolution():
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
    range_value = max_value - min_value

    # Calculate the number of ADC levels
    num_levels = 2 ** 8

    # Calculate the resolution
    res = round(math.log2(range_value), 3)

    # Print the results
    print("Maximum value: {}".format(max_value))
    print("Minimum value: {}".format(min_value))
    print("Range: {}".format(range_value))
    print("Number of levels: {}".format(num_levels))
    print("Resolution: {}".format(res))

    # Data is sampled every 1 us, time units should be in milliseconds
    time = [x * 1e-3 for x in range(len(data))]
    plt.plot(time, data)

    # Zero the time axis
    plt.xlim(0, max(time) + 2)

    plt.xlabel("Time (ms)")
    plt.ylabel("ADC Value")
    # plt.show()
    plt.savefig("ramp.png")
    plt.close()


def main():
    calculate_resolution()

    print()

    calculate_dc_offset_error()


if __name__ == '__main__':
    main()
