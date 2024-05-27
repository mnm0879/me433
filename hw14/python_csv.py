import csv

t = [] # column 0
data1 = [] # column 1

fname = 'C:/Users/maxim/OneDrive/Documents/GitHub/me433/hw14/sigA.csv'

with open(fname) as f:
    # open the csv file
    reader = csv.reader(f)
    for row in reader:
        # read the rows 1 one by one
        t.append(float(row[0])) # leftmost column
        data1.append(float(row[1])) # second column
        # data2.append(float(row[2])) # third column

for i in range(len(t)):
    # print the data to verify it was read
    print(str(t[i]) + ", " + str(data1[i]))

