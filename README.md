## STM32H7xx MCUs Startup Code

Most IDEs provide us an Assembly code startup file for all STM32 microcontrollers. And this startup file depends on the ARM CMSIS library, so you cannot develop your own standalone application that can access the internal registers directly using their addresses and does not depend on any library.

This is an independent, easy to read Startup Code written in "C" for STM32H743xx microcontrollers that works with the ARM GCC tool chain. It comes with a compatible linker script file and a demonstration example. You can easily use this repository as an eclipse template project and port it for your STM32H7xx device.

If you don't know about the Startup Code, you can read the:

###Theory on Startup Code

Startup code is a small block of assembly language code that prepares the way for the execution of software written in a high-level language. Each high-level language has its own set of expectations about the runtime environment. For example, C both utilize an implicit stack. Space for the stack has to be allocated and initialized before software written in either language can be properly executed. The location and contents of this file are usually described in the documentation supplied with the compiler. 

Startup code usually consists of the following actions : 

- Initialize the stacks.
- Disable all interrupts.
- Copy any initialized data from ROM to RAM.
- Zero the uninitialized data area.
- Allocate space for and initialize the stack.
- Enable interrupts.
- Initialize the heap.
- Call constructors.
- Initialize the .bss section to zero.
- If compiled with FULL_LIBRARY, get the command line from the host using debug_getargs and set registers to supply argc and argv to main.
- Call main.

 On return from main or when exit is called :

- If compiled with FULL_LIBRARY, call destructors.
- If compiled with FULL_LIBRARY, call atexit functions.
- If compiled with FULL_LIBRARY, call debug_exit while supplying the return result from main.
- Wait in exit loop.

The instruction from the main code will be executed only in the event that the high-level language program exits Depending on the nature of the embedded system, you might want to use these instructions to halt the processor, reset the entire system, or transfer control to a debugging tool.

Because the startup code is not inserted automatically, the programmer must usually assemble it himself and include the resulting object file among the list of input files to the linker. Every microcontroller has its own startup code, so before creating a project make sure you have the necessary startup code for the specific microcontroller. [[more](http://eagerlearning.org/microcontrollers/theory/startup-code/)]

