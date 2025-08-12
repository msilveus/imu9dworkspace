/* STM32F401RET6U memory layout */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

/* Stack size: 8 KB */
_stack_size = 0x2000;

/* Heap size: 16 KB (customizable) */
_heap_size = 0x4000;

/* Set the top of the stack (stack grows down) */
_estack = ORIGIN(RAM) + LENGTH(RAM);

/* Place .stack at top of RAM */
_stack_start = _estack;
_stack_end = _stack_start - _stack_size;

/* Heap begins after .bss/.data and ends before the stack */
_heap_start = ORIGIN(RAM);
_heap_end = _stack_end;

PROVIDE(_estack = _estack);
PROVIDE(_stack_start = _stack_start);
PROVIDE(_stack_end = _stack_end);
PROVIDE(_heap_start = _heap_start);
PROVIDE(_heap_end = _heap_end);
