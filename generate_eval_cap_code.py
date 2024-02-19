# public static void updateCap(String pre1,String post1,String pre2,String post2) {
#         updateCap_many(pre1,post1,pre2,post2);
# }

# for [n1,n2] robots
n1 = 4
n2 = 11

cod = ''
for i in range(n1,n2+1):
    head = ['\tString pre'+str(j)+',String post'+str(j) for j in range(1,i+1)]
    pars = head[0]
    for p in head[1:]:
        pars += ',\n' + p
    pars += '\n'

    headInvoc = ['\t\tpre'+str(j)+',post'+str(j) for j in range(1,i+1)]
    parsInvoc = headInvoc[0]
    for p in headInvoc[1:]:
        parsInvoc += ',\n' + p
    parsInvoc += '\n'

    cod += 'public static void updateCap(\n' + pars + ') {\n';
    cod += '\tupdateCap_many(\n' + parsInvoc + '\t);\n'
    cod += '}\n\n'

print(cod)
