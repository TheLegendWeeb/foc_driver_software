import math

resolution=1024
lookup_table=[]
for i in range(0,resolution):
    lookup_table.append(round(math.sin(i*(math.pi/2)/resolution),10))
with open("lookup_table.txt","w+") as f:
    f.write("".join(str(lookup_table)))