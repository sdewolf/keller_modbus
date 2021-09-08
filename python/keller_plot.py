import matplotlib.pyplot as plt
import csv

t = []
p = []
T = []

with open('/home/sdewolf/Data/2021.250.txt','r') as csvfile:
  plots = csv.reader(csvfile, delimiter = ',')
  for row in plots:
    t.append(row[0])
    p.append(float(row[1]))
    T.append(float(row[2]))

plt.figure(1)
plt.plot(p,color='k')
plt.xlabel('Time (minutes)')
plt.ylabel('Pressure (unknown)')

plt.figure(2)
plt.plot(T,color='k')
plt.xlabel('Time (minutes)')
plt.ylabel('Temperature ($^\circ$C)')

plt.show()

