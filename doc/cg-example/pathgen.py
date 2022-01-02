#!/usr/bin/env python3 

V = [i for i in range(6)]
A = {
   1 : [2, 3],
   2 : [4, 5],
   3 : [2, 4, 5],
   4 : [5, 6],
   5 : [6],
   6 : []
}

T = {
   (1,2) :  10, 
   (1,3) :  3,
   (2,4) :  1,
   (2,5) :  3,
   (3,2) :  2,
   (3,4) :  7,
   (3,5) :  3,
   (4,5) :  1,
   (4,6) :  7,
   (5,6) :  2
}

pcount:int = 0
pdata:[str] = []

def dfs(path:[int], tm:int):
   global pcount
   global pdata
   if path[-1] == 6 and tm <= 14:
      for pos in range(1, len(path)):
         pdata.append("%d %d %d" % (pcount+1, path[pos-1], path[pos]))
      pcount += 1
   for n in A[path[-1]]:
      path.append(n)
      dfs(path, path[-1])
      path.pop(-1)

p = [1]
tm = 0
dfs(p, tm)

with open("P.txt", "w") as fid:
   fid.write("# Set of all feasible paths\n")
   fid.write("# <set ID>\n")
   for i in range(pcount):
      fid.write("%d\n" % (i+1))

with open("xp.txt", "w") as fid:
   fid.write("# Parameters that set which arc belongs to which path\n")
   fid.write("# <p,i,j> bool\n")
   for d in pdata:
      fid.write("%s 1\n" % d)
