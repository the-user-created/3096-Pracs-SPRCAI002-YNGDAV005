import csv
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.fft import fft

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


def calculate_dc_offset_error():
    with open(file="zero.csv", mode="r") as in_file:
        # Read data into a list
        data = in_file.readlines()

        # Convert strings to integers
        data = [int(x) for x in data]

    # Calculate the average
    avg = sum(data) / len(data)

    # Calculate the DC offset error
    dc_offset_error = abs(avg - 0)

    print("DC offset error: {}".format(dc_offset_error))

    # Change size of plot
    plt.rcParams["figure.figsize"] = (20, 10)

    # Data is sampled over approximately 1.1 ms, time units should be in milliseconds
    time = [x * 1.1e-4 for x in range(len(data))]
    # Reduce the data into every 6th value
    data = data[::6]
    time = time[::6]
    plt.plot(time, data)

    plt.xlabel("Time (ms)")
    plt.ylabel("ADC Value")
    # plt.show()
    plt.savefig("zero.png")


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


def calculate_sfdr():
    # Load the data from freq1.csv
    data = np.genfromtxt('freq1.csv', delimiter=',')

    # Sample rate (in Hz)
    sample_rate = 1000000  # Replace with your actual sample rate

    # Perform the FFT
    fft_result = np.fft.fft(data)
    fft_freq = np.fft.fftfreq(len(data), 1 / sample_rate)

    # Keep only the positive frequencies (single-sided spectrum)
    positive_freq_indices = np.where(fft_freq >= 0)
    fft_freq = fft_freq[positive_freq_indices]
    fft_result = fft_result[positive_freq_indices]

    # Calculate the magnitude spectrum
    magnitude_spectrum = np.abs(fft_result)

    # Plot the single-sided frequency spectrum
    plt.figure(figsize=(10, 4))
    plt.plot(fft_freq, magnitude_spectrum)
    plt.title('Single-Sided Frequency Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.grid(True)
    # Only plot up to 2 * target_frequency
    target_frequency = 80000  # 80 kHz
    plt.xlim(0, 2 * target_frequency)  # Limit the plot to positive frequencies
    plt.show()

    # Find the index corresponding to 80 kHz in the frequency array
    index_80kHz = np.argmin(np.abs(fft_freq - target_frequency))

    # Identify the amplitude (magnitude) of the signal component at 80 kHz
    signal_amplitude = magnitude_spectrum[index_80kHz]

    # Find the index corresponding to the largest spurious component (excluding DC and 80 kHz)
    spurious_amplitude = np.max(magnitude_spectrum[1:index_80kHz])  # Exclude DC and 80 kHz

    # Calculate the power of the signal component
    power_signal = signal_amplitude ** 2

    # Calculate the power of the largest spurious component
    power_spurious = spurious_amplitude ** 2

    # Calculate SFDR as the ratio of power_signal to power_spurious
    SFDR = power_signal / power_spurious

    # Optionally, convert SFDR to dB
    SFDR_dB = 10 * np.log10(SFDR)

    print(f"Signal Amplitude at 80 kHz: {signal_amplitude}")
    print(f"Largest Spurious Amplitude: {spurious_amplitude}")
    print(f"Power of Signal Component: {power_signal}")
    print(f"Power of Largest Spurious Component: {power_spurious}")
    print(f"SFDR: {SFDR}")
    print(f"SFDR (dB): {SFDR_dB}")


def main():
    calculate_dc_offset_error()
    # calculate_resolution()
    # calculate_sfdr()


if __name__ == '__main__':
    main()
