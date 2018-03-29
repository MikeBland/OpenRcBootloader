/*
*****************************************************************************
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**
*****************************************************************************
*/


/*
 * In this linker script there is no heap available.
 * The stack start at the end of the ram segment.
 */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x04000000, LENGTH = 256K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 48K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */

  /* The program code and other data goes into FLASH */
  .text :
  {
      _stext = .;          /* Provide the name for the start of this section */
      
      CREATE_OBJECT_SYMBOLS
      KEEP(*(.vectors))
      
		*(.version)
      
      . = ALIGN(4);        /* Align the start of the text part */
      *(.text)
      *(.text.*)
 
      . = ALIGN(4);        /* Align the start of the exidx part */
      *(.ARM.exidx)
      *(.ARM.exidx.*)

      . = ALIGN(4);        /* Align the start of the rodata part */
      *(.rodata)
      *(.rodata.*)
      
      . = ALIGN(4);        /* Align the end of the section */
   } > RAM
   _etext = .;        /* define a global symbols at end of code */

   /*
    * The ".data" section is used for initialized data
    * and for functions (.fastrun) which should be copied 
    * from flash to ram. This functions will later be
    * executed from ram instead of flash.
    */
   .data : AT (_etext)
   {
      . = ALIGN(4);        /* Align the start of the section */
      _sdata = .;          /* Provide the name for the start of this section */
      
      *(.data)
      *(.data.*)
      
      . = ALIGN(4);        /* Align the start of the fastrun part */
      *(.fastrun)
      *(.fastrun.*)
      
      . = ALIGN(4);        /* Align the end of the section */
   } > RAM
   _edata = .;             /* Provide the name for the end of this section */
   

   /*
    * The ".bss" section is used for uninitialized data.
    * This section will be cleared by the startup code.
    */
   .bss :
   {
      . = ALIGN(4);        /* Align the start of the section */
      _sbss = .;           /* Provide the name for the start of this section */
      
      *(.bss)
      *(.bss.*)
      
      . = ALIGN(4);        /* Align the end of the section */
   } > RAM = 0
   _ebss = .;              /* Provide the name for the end of this section */

     /* 
    * The ".stack" section is our stack.
    * Here this section starts at the end of the ram segment.
    */
   _estack = ORIGIN(RAM) + LENGTH(RAM);
  
}
