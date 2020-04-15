
from pandas import read_csv;
import matplotlib
import matplotlib.pyplot as plt
import numpy as np  

df = read_csv('wp-2020-04-15-06-31-57.csv')
     
    
newDF = df.drop_duplicates();
a = newDF.to_numpy()

file = open('wp.csv', 'w')

a = a[370:1310]


a[710:900,:] = np.linspace(a[710,:],a[900,:],900-710)
a[120:300,:] = np.linspace(a[120,:],a[300,:],300-120)
a[400:515,:] = np.linspace(a[400,:],a[515,:],515-400)


fig, ax = plt.subplots()
ax.plot(a[:,0], a[:,1])

#a = a[::2]
for i in range(len(a)):
    file.write('%f, %f, %f, %f\n' %(a[i,0], a[i,1], a[i,2], a[i,3]) )



