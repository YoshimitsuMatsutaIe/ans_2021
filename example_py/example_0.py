x_list = [round(i / 10, 1) for i in range(0, 100)]
y_list = [round((i**2 + i + 1), 1) for i in x_list]

print(y_list)