#ifndef __MK60_IT_H__
#define __MK60_IT_H__

/*                          Redefine the interrupt vector table
 *  First cancel the default interrupt vector element macro definition        #undef  VECTOR_xxx
 *  Redefine to the interrupt function you wrote      #define VECTOR_xxx    xxx_IRQHandler
 *  For example£º
 *       #undef  VECTOR_003                         First unmap the interrupt function address macro definition in the interrupt vector table
 *       #define VECTOR_003    HardFault_Handler    Redefine hardware access interrupt service function
 */







#endif  //__MK60_IT_H__
