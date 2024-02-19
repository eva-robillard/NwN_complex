from math import factorial

def combinations(n, r):
    return factorial(n) // factorial(r) // factorial(n-r)


for i in range(2,11):
    print(combinations(10,i))