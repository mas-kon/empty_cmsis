#line 1 "USER\\src\\led_init.c"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"













































 



 



 
    






   


 
  


 






 
#line 113 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
   


 





 
   




 
#line 137 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



 

#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"









































 



 



 
    









 



 








 
  


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,      
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  FPU_IRQn                    = 81       
} IRQn_Type;



 

#line 1 ".\\CMSIS\\inc\\core_cm4.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 ".\\CMSIS\\inc\\core_cm4.h"

















 




 



 

 













#line 120 ".\\CMSIS\\inc\\core_cm4.h"



 
#line 135 ".\\CMSIS\\inc\\core_cm4.h"

#line 209 ".\\CMSIS\\inc\\core_cm4.h"

#line 1 ".\\CMSIS\\inc\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 ".\\CMSIS\\inc\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 ".\\CMSIS\\inc\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 ".\\CMSIS\\inc\\cmsis_armcc.h"











 


#line 54 ".\\CMSIS\\inc\\core_cmInstr.h"

 
#line 84 ".\\CMSIS\\inc\\core_cmInstr.h"

   

#line 211 ".\\CMSIS\\inc\\core_cm4.h"
#line 1 ".\\CMSIS\\inc\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 ".\\CMSIS\\inc\\core_cmFunc.h"

 
#line 84 ".\\CMSIS\\inc\\core_cmFunc.h"

 

#line 212 ".\\CMSIS\\inc\\core_cm4.h"
#line 1 ".\\CMSIS\\inc\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 ".\\CMSIS\\inc\\core_cmSimd.h"

 
#line 88 ".\\CMSIS\\inc\\core_cmSimd.h"

 






#line 213 ".\\CMSIS\\inc\\core_cm4.h"
















 
#line 256 ".\\CMSIS\\inc\\core_cm4.h"

 






 
#line 272 ".\\CMSIS\\inc\\core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 ".\\CMSIS\\inc\\core_cm4.h"

#line 1550 ".\\CMSIS\\inc\\core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 183 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
#line 1 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\system_stm32f4xx.h"



































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 184 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
#line 185 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;     
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1;
  volatile uint32_t MACDBGR;
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;





 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
  uint32_t      RESERVED1;   
  uint32_t      RESERVED2;   
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED3;   
  volatile uint32_t ECCR3;       
} FSMC_Bank2_3_TypeDef;



 

typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];   
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;                  
  volatile uint32_t CLKCR;                  
  volatile uint32_t ARG;                    
  volatile uint32_t CMD;                    
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;                 
  volatile uint32_t DLEN;                   
  volatile uint32_t DCTRL;                  
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;                    
  volatile uint32_t MASK;                   
  uint32_t      RESERVED0[2];           
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];          
  volatile uint32_t FIFO;                   
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved40[48];            
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct 
{
  volatile uint32_t DCFG;             
  volatile uint32_t DCTL;             
  volatile uint32_t DSTS;             
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;          
  volatile uint32_t DOEPMSK;          
  volatile uint32_t DAINT;            
  volatile uint32_t DAINTMSK;         
  uint32_t  Reserved20;           
  uint32_t Reserved9;             
  volatile uint32_t DVBUSDIS;         
  volatile uint32_t DVBUSPULSE;       
  volatile uint32_t DTHRCTL;          
  volatile uint32_t DIEPEMPMSK;       
  volatile uint32_t DEACHINT;         
  volatile uint32_t DEACHMSK;         
  uint32_t Reserved40;            
  volatile uint32_t DINEP1MSK;        
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;       
} USB_OTG_DeviceTypeDef;



 
typedef struct 
{
  volatile uint32_t DIEPCTL;            
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;            
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;           
  volatile uint32_t DIEPDMA;            
  volatile uint32_t DTXFSTS;            
  uint32_t Reserved18;              
} USB_OTG_INEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;



 



 
#line 935 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 





 
#line 975 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 992 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1029 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 



 






 

 



#line 1059 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"






 



   
#line 1153 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1194 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1248 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
  
 
#line 1298 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1354 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1416 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 
#line 1487 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1537 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1587 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1626 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 
#line 1654 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1710 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 1751 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1759 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
 
#line 1797 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 1825 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1875 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 1888 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 1901 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1915 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1929 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 1988 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
#line 2000 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 2007 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 2014 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2043 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
 
#line 2062 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2073 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2087 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2101 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2118 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2129 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2143 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2157 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2174 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

   
#line 2185 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2199 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2213 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2227 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2238 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2252 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2266 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2280 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2291 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2305 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2319 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
#line 2328 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2417 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2506 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2595 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2684 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
#line 2783 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2881 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 2979 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3077 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3175 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3273 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3371 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3469 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3567 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3665 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3763 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3861 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 3959 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4057 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4155 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4253 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4351 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4449 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4547 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4645 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4743 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4841 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 4939 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5037 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5135 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5233 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5331 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5429 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 





 





 




 
 
 
 
 


 

 
#line 5471 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 5478 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 5492 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 5508 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 5515 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 5529 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 5536 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5544 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 
#line 5582 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5590 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5598 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 5616 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 5657 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5668 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5685 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 5692 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5709 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 


 
#line 5728 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 






 
#line 5752 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 


 
#line 5769 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5783 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5791 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5799 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 5813 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 5892 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 5918 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  
#line 5937 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  
#line 5999 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  
#line 6061 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  
#line 6123 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  
#line 6185 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 





 
 
 
 
 
 
#line 6277 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6305 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6376 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6401 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6472 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6543 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6614 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6685 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 6703 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 6725 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6748 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6781 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6789 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 6830 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
                                             
 
#line 6847 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 6860 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













#line 6909 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6917 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













#line 6966 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 6974 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













#line 7023 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7031 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













#line 7080 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7089 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7097 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7109 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7117 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7125 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7133 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7148 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7156 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7168 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7176 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7184 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7192 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7207 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7215 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7227 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7235 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7243 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7251 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7266 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7274 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7286 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7294 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7302 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7310 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7325 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7333 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7345 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7353 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7368 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7376 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7388 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7396 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7411 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7419 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7431 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7439 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7454 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7462 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7474 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7482 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 7499 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











#line 7517 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7525 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7532 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7543 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











#line 7561 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7569 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7576 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7587 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











#line 7605 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7613 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7620 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7643 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7666 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7689 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7702 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7714 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7726 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7738 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7751 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7763 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7775 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7787 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7800 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7812 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7824 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7836 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7849 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7861 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7873 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7885 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7898 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7910 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7922 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7934 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7947 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7959 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7971 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 7983 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 7996 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 8008 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 8020 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 8032 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
 
 
 
 
 
#line 8129 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8211 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8261 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8279 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8361 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8411 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8493 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8543 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8593 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8611 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8661 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 8678 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8776 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8810 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 8862 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 8919 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 8961 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9019 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9061 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9111 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
 
 
 
 
 
#line 9161 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9172 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9188 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 



#line 9223 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 
#line 9235 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 9284 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9310 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9321 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 





 
 
 
 
 
 




 
#line 9345 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 9358 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
 
 
 
 
 
#line 9382 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9389 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9408 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 


 
#line 9434 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 


 
 
 
 
 
 
#line 9450 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9459 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9471 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9496 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9507 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9520 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 9534 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9542 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 










 










 
#line 9574 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9584 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9592 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 9606 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 9622 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 










#line 9640 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9647 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 9673 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9695 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 9714 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 
#line 9762 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9773 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 





 
#line 9849 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 9884 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 


 
#line 9949 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 9959 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 10035 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10082 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10132 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 10151 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10162 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 10238 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10279 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10290 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 10303 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10335 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10349 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10363 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 10370 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
 
 
 
 
 
#line 10384 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10401 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 10449 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10493 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10563 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10613 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10621 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 10634 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10704 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10774 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 10792 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10835 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10865 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 10893 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10941 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10953 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 10965 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 





 
 
 
 
 
 






 
#line 11092 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 11105 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 










#line 11143 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 




 




 
#line 11197 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11205 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11218 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 11297 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11338 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11412 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
 
 
 
 


 


 
#line 11443 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11450 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11481 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11504 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11533 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 






























#line 11591 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11602 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 





 



 


 
#line 11634 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
#line 11646 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11659 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11672 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11685 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11699 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11712 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11725 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11738 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11751 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11765 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11778 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11791 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11804 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11817 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11831 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11843 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11855 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11867 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



 
#line 11879 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11887 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 11909 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

















 
#line 11936 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11943 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11968 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 11976 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 11983 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





#line 11995 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 12008 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12055 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12093 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12119 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 






#line 12133 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 12140 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











#line 12157 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 12164 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 







#line 12184 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 12198 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 






#line 12212 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 12219 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











#line 12236 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 12243 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 







#line 12263 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 12277 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12324 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 




 




 
#line 12377 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 12402 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12412 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 12421 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 12438 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 
 
 
 
 
 
#line 12476 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 12489 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12536 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12559 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"











 
#line 12607 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12620 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 
 
 
 
 
 
#line 12641 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 12649 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"





 
#line 12665 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 12673 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"






 







 





 
 
 
 
 
 
#line 12705 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12719 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 12778 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 


 
#line 12797 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 
 
 
 
#line 12862 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12906 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 12946 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 12984 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 12992 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

  




 








 

  
#line 13031 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13111 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13128 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13136 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 13166 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 13191 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 13216 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
 
 

 
#line 13245 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13256 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13267 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13278 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13289 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 




 




 
 
 

 
#line 13352 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 13371 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 13389 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 13402 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 
#line 13425 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
 
 

 
#line 13492 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 
#line 13526 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
   
#line 13619 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13669 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13716 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13730 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 




 
 
 
 
 
 
#line 13787 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 13798 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 13809 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 13820 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













 
#line 13843 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13863 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13877 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 13899 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 13912 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




#line 13929 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 13950 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 14014 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14031 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


#line 14047 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14073 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14089 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14101 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 14129 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14209 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14289 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14297 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 14316 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14324 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 14337 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 14351 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14359 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14367 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 14380 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







#line 14394 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14402 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14410 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 
#line 14434 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




#line 14456 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14467 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14475 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14491 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14507 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 14520 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14540 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14548 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 14582 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14611 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14620 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14628 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 14669 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14677 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14691 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14700 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14726 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




#line 14745 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"













#line 14777 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 14790 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14801 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

#line 14813 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14848 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14883 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 14918 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 14930 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"
 
#line 14945 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 




 




 
#line 14968 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 15009 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15029 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 

#line 15038 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"







 
#line 15055 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"



  



 



 

 




 


 


 


 


 
#line 15102 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15113 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 


 



 



 


 



 





 
#line 15158 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15172 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15182 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15190 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15198 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 



 
#line 15210 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15220 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15228 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15236 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15244 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15256 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15266 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 



 
#line 15278 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 




 
#line 15341 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 
#line 15353 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 





 
#line 15367 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"

 





 





 
#line 15387 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"


 



 



 


 


 


 






#line 15420 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f407xx.h"




















 
 
 
 
 
 
 
 


 



 



 



 









 
#line 152 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"
#line 195 "C:\\Keil_v5\\ARM\\PACK\\Keil\\STM32F4xx_DFP\\2.11.0\\Drivers\\CMSIS\\Device\\ST\\STM32F4xx\\Include\\stm32f4xx.h"



 



  
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0U, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 












 



 
  



 
#line 2 "USER\\src\\led_init.c"
#line 1 ".\\USER\\inc\\led_init.h"



#line 5 ".\\USER\\inc\\led_init.h"

















void LED_Init(void);

#line 3 "USER\\src\\led_init.c"

void LED_Init(void){
	 
	((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR |= (0x1U << (3U));
	
	 
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		&= 	~(0x2U << (24U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		|= 	 (0x1U << (24U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OTYPER		&= 	~(0x1U << (12U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->PUPDR 		&= 	~(0x3U << (24U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	|= 	 (0x1U << (24U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	&= 	~(0x2U << (24U));

	 
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		&= 	~(0x2U << (26U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		|= 	 (0x1U << (26U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OTYPER		&= 	~(0x1U << (13U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->PUPDR 		&= 	~(0x3U << (26U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	|= 	 (0x1U << (26U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	&= 	~(0x2U << (26U));	

	 
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		&= 	~(0x2U << (28U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		|= 	 (0x1U << (28U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OTYPER		&= 	~(0x1U << (14U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->PUPDR 		&= 	~(0x3U << (28U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	|= 	 (0x1U << (28U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	&= 	~(0x2U << (28U));	
	
	 
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		&= 	~(0x2U << (30U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->MODER 		|= 	 (0x1U << (30U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OTYPER		&= 	~(0x1U << (15U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->PUPDR 		&= 	~(0x3U << (30U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	|= 	 (0x1U << (30U));
	((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0C00U))->OSPEEDR 	&= 	~(0x2U << (30U));		
}
