.data
    n: .word 10
    
.text
.globl __start

FUNCTION:
    # Todo: Define your own function in HW1
    # You should store the output into a0
    #store some consts that would be used
    addi x12, x0, 3 #for multiplication in T
    addi x13, x0, 10 #for multiplication in T

    #store the previous instr
    addi sp, sp, -8
    sw x1, 0(sp)
    jal x1, t_function #call the recursive function
    #return
    lw x1, 0(sp)
    addi sp, sp, 8
    jalr x0, 0(x1)#when the recursive function is finished, jump to print result
    
t_function:
    #store the origianl value
    addi sp,sp,-16
    sw x1,0(sp)
    sw a0,8(sp)
    #we use word but not double here
    #set x7 = n-4
    addi x7,a0,-4
    #compare n-4>=0, if so jump to L1
    bge x7,x0,L1
    #, else the  result would be 1
    addi a0, x0, 3
    #pop the stack
    addi sp, sp, 16
    #jump back to caller
    jalr x0, 0(x1)
    
L1:
    srai a0, a0, 2 #divide a0 by 4 
    jal x1, t_function #call T(n-1)
    addi x6, a0, 0 #move the result of the leaf function to x6
    #restore previous value of x1 and a0
    lw x1, 0(sp)
    lw a0, 8(sp)
    addi sp, sp, 16
    mul x30, a0, x13 #store x30 with 10*a0 = 10n
    #T(n/4)*3
    mul x6, x6, x12
    #a0 = 3T(n/4) + 10n
    add a0, x6, x30
    addi a0, a0, 3
    jalr x0, 0(x1)
    

# Do NOT modify this part!!!
__start:
    la   t0, n
    lw   a0, 0(t0)
    jal  x1,FUNCTION
    la   t0, n
    sw   a0, 4(t0)
    addi a0,x0,10
    ecall