import math


nRobots = 10
maxTransRobot = 7

#--------------------------------------------------------
def combinations(n,k):
    return math.factorial(n)/(math.factorial(k)*math.factorial(n-k))
#--------------------------------------------------------

nEvals = 0
for i in range(1,nRobots+1):
    nEvals += combinations(nRobots,i)*maxTransRobot**i

print('---------------------------------------------------------')
print (f'#evals for {nRobots} robots is {math.floor(nEvals):,}')
print('---------------------------------------------------------')
