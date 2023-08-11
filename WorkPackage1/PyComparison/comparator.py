#!/usr/bin/python3
# Compares the results of the two algorithms


def main():
    # Open the files
    with open("result.txt", "r") as f:
        c = f.readlines()
    with open("result_py.txt", "r") as f:
        py = f.readlines()

    # Compare each element of the arrays
    num_diff = 0
    for i in range(len(c)):
        if eval(c[i]) != eval(py[i]):
            num_diff += 1

    print("There are {} differences of {} samples".format(num_diff, len(c)))
    print("Accuracy rating: {}%".format(100 - (num_diff / len(c)) * 100))


if __name__ == '__main__':
    main()
