# Generates labels for transitions of the system net
# :s_3(m,x1,n1,x2,n2,x3,n3)

# Example for n=3
# m:b(e0);
# x1:r(e1,cond1,cond2,t1);
# x2:r(e2,cond3,cond4,t2);
# x3:r(e3,cond5,cond6,t3);
# this:ev(ev);
# guard true==ev.gef(e0,e1,cond1,cond2,e2,cond3,cond4,e3,cond5,cond6);
# action ev.updateCap(cond1,cond2,cond3,cond4,cond5,cond6);
# this:fs(n1+"("+t1+"):"+e1+"-"+n2+"("+t2+"):"+e2+"-"+n3+"("+t3+"):"+e3+"\n")


# for [n1,n2] robots
n1 = 4
n2 = 11


# labels of transitions
# :s_3(m,x1,n1,x2,n2,x3,n3)
# this:s_3(m,x1,n1,x2,n2,x3,n3)
for i in range(n1,n2+1):
	syncText = ':s_' + str(i) + '(m'
	for j in range(1,i+1):
		syncText += ',x' + str(j) + ',n' + str(j)
	syncText += ')'
	print(syncText)

for i in range(n1,n2+1):
	syncText = 'this:s_' + str(i) + '(m'
	for j in range(1,i+1):
		syncText += ',x' + str(j) + ',n' + str(j)
	syncText += ')'
	print(syncText)

# print('--------------------------------')
# #condition variables cond_1,---,cond_n
# for i in range(n1,n2+1):
# 	conVars = 'cond_' + str(n1)
# 	for i in range(n1+1,n2+1):
# 		condVars += ',' + 'cond_' + str(i)

# labels for synchronizing transitions
# [x1,n1];[x2,n2];[x3,n3]
print('--------------------------------')
for i in range(n1,n2+1):
	syncText = '[x1,n1]'
	for j in range(2,i+1):
		sj = str(j)
		syncText += ';[x' + sj + ',n' + sj + ']'
	print(syncText)
print('--------------------------------')
for i in range(n1,n2+1):
	ist = str(i)
	transLabel = 'm:b(e0);\nthis:ev(ev);\n'
	ind = 1
	for j in range(1,i+1):
		js = str(j)
		transLabel += 'x' + js + ':r(e' + js + ',cond' + str(ind) + ',cond' + str(ind+1) + ',t' + js + ');\n'
		ind += 2

	ind = 1
	transLabel += 'guard true==ev.gef(e0'
	for j in range(1,i+1):
		js = str(j)
		transLabel += ',e' + js + ',cond' + str(ind) + ',cond' + str(ind+1)
		ind += 2
	transLabel += ');\n'

	transLabel += 'action ev.updateCap(cond1,cond2'
	ind = 3
	for j in range(2,i+1):
		transLabel += ',cond' + str(ind) + ',cond' + str(ind+1)
		ind += 2
	transLabel += ');\n'

	transLabel += 'this:fs('
	transLabel += 'n1+"("+t1+"):"+e1'
	for j in range(2,i+1):
		js = str(j)
		transLabel += '+"-"+n' + js + '+"("+t' + js + '+"):"+e' + js
		ind += 2
	transLabel += '+"\\n")' + '\n'


	print(transLabel)

