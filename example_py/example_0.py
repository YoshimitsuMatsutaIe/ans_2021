
def do_example_0():
    x_list = [round(i / 10, 1) for i in range(0, 100)]
    y_list = [round((i**2 + i + 1), 1) for i in x_list]

    print(y_list)

if __name__ == '__main__':
    do_example_0()