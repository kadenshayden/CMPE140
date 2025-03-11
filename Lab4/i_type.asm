.text 
addi t0, x0, 5
addi t1, t0, -10
addi t2, t1, 3
slli t3, t2, 2
srai t4, t3, 1
srli t4, t4, 1
xori t3, t3, -1
andi t5, t1, 0x7f0
ori t6, t4, 0x7f0
slti t0, t3, 0x123