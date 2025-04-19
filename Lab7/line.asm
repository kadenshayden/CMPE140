.data 
m:
.word -3
x:
.word 7
c:
.word 25
y:
.word -1 # random initial value

.text
# Initialize m , x, c and y
# m in t1, x in t2, c in t3, y in t4
#addi t1, x0, 3
lui     t0,%hi(m)
addi    t0, t0,%lo(m)
lw t1, 0(t0)

#addi t2, x0, -7
lui     t0,%hi(x)
addi    t0, t0,%lo(x)
lw t2, 0(t0)

#addi t3, x0, 10
lui     t0,%hi(c)
addi    t0, t0,%lo(c)
lw t3, 0(t0)

add t4, x0, x0
# Multiply without mul instruction
mul_m_x :
bge t1, x0, move_t1
sub t5, x0, t1   
jal x0, check_t2
move_t1 :
add t5, x0, t1  
check_t2 :
bge t2, x0, move_t2
sub t6, x0, t2  
jal x0, multiply
move_t2 :
add t6, x0, t2 
multiply :
beq t5, x0, rectify_sign
add t4, t4, t6
addi t5, t5, -1
jal x0, multiply
rectify_sign :
xor t0, t1, t2
bge t0, x0, add_c
sub t4, x0, t4
add_c :
add t4, t4, t3

lui     t0,%hi(y)
addi    t0, t0,%lo(y)
sw t4, 0(t0)



