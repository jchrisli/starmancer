'''
    Generate study conditions with counter-balancing 
'''

'''
    Constants
'''

techniques = [['SH', 'JS'], ['JS', 'SH']]
target_size = [[['L', 'S'], ['L', 'S']], \
                [['L', 'S'], ['S', 'L']], \
                [['S', 'L'], ['S', 'L']], \
                [['S', 'L'], ['L', 'S']]]


L_latin_square = [range(9 - i, 9) + range(1, 9 - i) for i in range(0, 8)]
print(L_latin_square)
S_latin_square = [[ind + 16 for ind in row] for row in L_latin_square]

combinations = []

for t_seq in techniques:
    for s_seq in target_size:
        combinations.append(zip(t_seq, s_seq))
    
conditions = []

for cond in combinations:
    for i in range(0, len(cond)):
        technique = cond[i][0]
        tech_and_size = [technique + '_' + size for size in cond[i][1]]
        tech_size_rep = [[ts for r in range(1,5)] for ts in tech_and_size]
        tech_size_rep = [tsr for subtsr in tech_size_rep for tsr in subtsr]
        for tsr in tech_size_rep:
            conditions.append(tsr)
    
for i in range(0, 8):
    f = open('t1_p%s.txt' % (i + 1), 'wt')
    for j in range(0, 16):
        (task, size) = conditions[i * 8 + j].split('_')
        img_ind = j % 4
        if task == 'SH':
            img_ind += 4
        if size == 'S':
            img_num = S_latin_square[i][img_ind]
        else:
            # print('%s %s' % (i, img_ind))
            img_num = L_latin_square[i][img_ind]
        line = '_'.join([task, size, str(img_num)])
        f.write(line + '\n')
    f.close()