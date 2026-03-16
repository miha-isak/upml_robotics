import typing,numpy 
values=numpy.array([-5,-4,-3,-2,-1,0,1,2,3,4,5])*4
def make(n:int,values:list[int]|numpy.ndarray)->typing.Generator[int]:
    for i in range(1<<n):
        out=sum(values[j] for j in range(n) if (i>>j)&1)
        yield out
a=make(11,values)
with open("sensors_pos.h",'wt')as f:
    f.write("short sensors_pos[] = {")
    for v in a:
        f.write(f"\t{v},\n")
    f.write("};\n")
