# MIPS Pipeline Processor

An implementation of the MIPS pipeline processor.

## Supported Commands

**Arithmetic/Logical Instructions:** add, sub, and, or, slt, addi, andi

**Memory Reference Instruction:** lw, sw

**Control Flow Instructions:** j, beq, bne

## Pipeline Hazards

The processor can identify and handle (Forwarding and Stall Insertion) data and control hazards.

### Examples of data hazards

Data dependency between beq/bne and add:

```
add R1,R2,R3
beq R1,R2,L1
```

Data dependency between beq/bne and lw:

```
lw R1, 100(R0)
beq R1,R2,L1
```
