import math

resolution=1200
lookup_table=[]
_2PI  =6.2831853072
for i in range(0,resolution):
    lookup_table.append(round(math.sin(i*_2PI/resolution),10))
with open("lookup_table.txt","w+") as f:
    f.write("".join(str(lookup_table)))