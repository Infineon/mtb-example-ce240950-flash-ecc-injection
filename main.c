/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Flash ECC Injection Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <inttypes.h>
#include "mtb_hal.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Test value */
#define CORRECT_DATA                   (0x0123456789abcdefull)

/* The 128-bit code word used for calculating the parity has this static 64-bit component */
#define CODEWORD_UPPER_64_BIT          (0x0000000000000108ull)

/* Number of bytes in a 64-bit word */
#define BYTES_PER_64_BIT_WORD          8

/* Fault interrupt priority */
#define IRQ_PRIORITY                   2

/* Shift value for CPU IRQ number ('intSrc' of cy_stc_sysint_t consists of CPU IRQ number and system IRQ number) */
#define CPU_IRQ_NUMBER_SHIFT           16

/* Interval value */
#define INTERVAL_MS                    (1)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Data type for 128-bit variable */
typedef struct
{
    uint64_t u64[2];
} Uint128Type;


/* Create a global volatile variable where the test access will store its data to prevent compiler optimization */
volatile uint64_t g_flashReadData;

/* Flag that is set in the fault IRQ handler and checked/cleared at other places */
bool g_faultIrqOccurred = false;


/* Variable that will end up as constant data in flash and will be used for the error injection test */
const uint64_t FLASH_TEST_VAR = CORRECT_DATA;

/* For the Retarget -IO (Debug UART) usage */
static cy_stc_scb_uart_context_t    UART_context;           /** UART context */
static mtb_hal_uart_t               UART_hal_obj;           /** Debug UART HAL object  */

/*******************************************************************************
* Function Name: handleFaultIrq
********************************************************************************
* Summary:
* This is the fault struct interrupt handles 
* It will occur on a code flash correctable ECC fault.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
static void handleFaultIrq(void)
{
    uint32_t faultAddress;
    uint32_t faultInfo;

    printf("Fault IRQ Handler entered!\r\n");

    /* Get fault specific data from the registers */
    faultAddress = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA0);
    faultInfo = Cy_SysFault_GetFaultData(FAULT_STRUCT0, CY_SYSFAULT_DATA1);
    cy_en_SysFault_source_t errorSource = Cy_SysFault_GetErrorSource(FAULT_STRUCT0);

    /* Check and display the fault information */
    if (errorSource == CY_SYSFAULT_FLASHC_MAIN_C_ECC)
    {
        printf("- Address:       0x%" PRIx32 "\r\n", (uint32_t)CY_FLASH_LG_SBM_BASE + faultAddress);
        printf("- ECC syndromes: 0x%" PRIx32 "(4x syndromes based on 256-bit aligned flash access)\r\n", faultInfo);

        if (g_flashReadData == CORRECT_DATA)
        {
            printf("TEST OK!\r\n");
        }
        else
        {
            printf("TEST ERROR: Incorrect data read!\r\n");
        }
    }
    else
    {
        printf("TEST ERROR: Unexpected fault source (0x%" PRIx32 ") detected!\r\n", (uint32_t)errorSource);
    }

    /* Set flag so that test code can check that the IRQ has occurred */
    g_faultIrqOccurred = true;

    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_ClearInterrupt(FAULT_STRUCT0);
}

/*******************************************************************************
* Function Name: Cy_SysLib_ProcessingFault
********************************************************************************
* Summary:
* This is the Processing Fault 
* It will be called by the PDL fault handler.
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
void Cy_SysLib_ProcessingFault(void)
{
    printf("Hard Fault Handler entered!\r\n");

    /* PDL fault handler has populated the global cy_faultFrame variable with fault information. The test access to
       flash with a non-correctable ECC fault injected should result in a precise error and the Bus Fault Address
       Register (BFAR) should be valid and hold the address of the test variable */
    if ((cy_faultFrame.cfsr.cfsrBits.precisErr == 1) &&
            (cy_faultFrame.cfsr.cfsrBits.bfarValid == 1) &&
            (cy_faultFrame.bfar == (uint32_t)&FLASH_TEST_VAR))
    {
        printf("TEST OK!\r\n");
    }
    else
    {
        printf("TEST ERROR: Unknown Hard Fault!\r\n");
    }

    /* The fault struct could be used additionally to gather even more information about the fault */

    /* Do not return from the fault */
    for (;;)
    {
    }        
}

/*******************************************************************************
* Function Name: initFaultHandling
********************************************************************************
* Summary:
* This Initialize fault handling to generate an IRQ on a Code Flash correctable 
* ECC fault 
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/

static void initFaultHandling(void)
{
    /* Only IRQ is needed as fault reaction in this example */
    cy_stc_SysFault_t faultStructCfg =
    {
        .ResetEnable   = false,
        .OutputEnable  = false,
        .TriggerEnable = false,
    };

    /* Set up fault struct and enable interrupt for Code Flash correctable ECC fault */
    Cy_SysFault_ClearStatus(FAULT_STRUCT0);
    Cy_SysFault_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFAULT_FLASHC_MAIN_C_ECC);
    Cy_SysFault_SetInterruptMask(FAULT_STRUCT0);
    if (Cy_SysFault_Init(FAULT_STRUCT0, &faultStructCfg) != CY_SYSFAULT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Set up the interrupt processing */
    cy_stc_sysint_t irqCfg =
    {
        .intrSrc = ((NvicMux3_IRQn << CPU_IRQ_NUMBER_SHIFT) | cpuss_interrupts_fault_0_IRQn),
        .intrPriority = IRQ_PRIORITY,
    };
    if (Cy_SysInt_Init(&irqCfg, &handleFaultIrq) != CY_SYSINT_SUCCESS)
    {
        CY_ASSERT(0);
    }
    NVIC_EnableIRQ((IRQn_Type) NvicMux3_IRQn);
}

/*******************************************************************************
* Function Name: do128BitAnd
********************************************************************************
* Summary:
* It do bitwise AND operation on two 128-bit values
*
* Parameters:
*  a - 1st value
*  b - 2nd value
*
* Return:
*  Uint128Type - result of AND operation
*
*******************************************************************************/
static Uint128Type do128BitAnd(Uint128Type a, Uint128Type b)
{
    Uint128Type result128;
    result128.u64[0] = a.u64[0] & b.u64[0];
    result128.u64[1] = a.u64[1] & b.u64[1];
    return result128;
}

/*******************************************************************************
* Function Name: do128BitXorReduction
********************************************************************************
* Summary:
* It do reduction XOR operation on 128-bit value 
* ECC fault 
*
* Parameters:
*  data128 - value
*
* Return:
*  uint8_t - XOR reduction result
*
*******************************************************************************/
static uint8_t do128BitXorReduction(Uint128Type data128)
{
    /* Use Brian Kernighan algorithm to count the number of '1' bits in the provided 128-bit value */
    uint8_t countOneBits = 0;
    for (uint8_t word = 0; word < 2; word++)
    {
        for (/* No init needed */; data128.u64[word] != 0; countOneBits++)
        {
            /* Clear the least significant bit set */
            data128.u64[word] &= (data128.u64[word] - 1); 
        }
    }

    /* If the number of bits set is even, the XOR reduction results in '0', or '1' otherwise */
    return ((countOneBits % 2) == 0) ? 0 : 1;
}

/*******************************************************************************
* Function Name: getParityForValue
********************************************************************************
* Summary:
* This calculate the Code Flash ECC parity for the provided 64-bit value
*
* Parameters:
*  value64 - value for which ECC parity shall calculated
*
* Return:
*  uint8_t - ECC parity
*
*******************************************************************************/
static uint8_t getParityForValue(uint64_t value64)
{
    /* Constants for ECC parity calculation as per the Architecture TRM */
    static const Uint128Type ECC_P[8] =
    {
        {{0x44844a88952aad5bull, 0x01bfbb75be3a72dcull}},
        {{0x1108931126b3366dull, 0x02df76f9dd99b971ull}},
        {{0x06111c2238c3c78eull, 0x04efcf9f9ad5ce97ull}},
        {{0x9821e043c0fc07f0ull, 0x08f7ecf6ed674e6cull}},
        {{0xe03e007c00fff800ull, 0x10fb7baf6ba6b5a6ull}},
        {{0xffc0007fff000000ull, 0x20fdb7cef36cab5bull}},
        {{0xffffff8000000000ull, 0x40fedd7b74db55abull}},
        {{0xd44225844ba65cb7ull, 0x807f000007ffffffull}},
    };

    /* The 128-bit code word which is the basis for the ECC parity calculation is constructed as follows */
    Uint128Type codeWord128;
    codeWord128.u64[0] = value64;
    codeWord128.u64[1] = CODEWORD_UPPER_64_BIT;

    /* Calculate each ECC parity bit individually according to the Architecture TRM */
    uint8_t parity = 0;
    for (uint32_t cnt = 0; cnt < (sizeof(ECC_P) / sizeof(ECC_P[0])); cnt++)
    {
        parity |= (do128BitXorReduction(do128BitAnd(codeWord128, ECC_P[cnt])) << cnt);
    }

    return parity;
}

/*******************************************************************************
* Function Name: injectParity
********************************************************************************
* Summary:
* This enable the injection of the provided ECC parity value for the provided 
* address 
*
* Parameters:
*  flashAddr - target flash address for which ECC parity shall be injected
*  parity - parity value to be injected
*
* Return:
*  none
*
*******************************************************************************/
static void injectParity(uint32_t flashAddr, uint8_t parity)
{
    uint32_t wordAddr;

    /* The register expects a relative 64-bit word address/index */
    wordAddr = (flashAddr - CY_FLASH_LG_SBM_BASE) / BYTES_PER_64_BIT_WORD;

    /* Set the parity and target address */
    FLASHC_ECC_CTL = _VAL2FLD(FLASHC_ECC_CTL_PARITY, parity) |
                     _VAL2FLD(FLASHC_ECC_CTL_WORD_ADDR, wordAddr);

    /* Enable ECC injection */
    CY_REG32_CLR_SET(FLASHC_FLASH_CTL, FLASHC_FLASH_CTL_MAIN_ECC_INJ_EN, 1);

    /* Read back register to ensure that the previous register writes got effective */
    FLASHC_FLASH_CTL;
}

/*******************************************************************************
* Function Name: executeTestAccess
********************************************************************************
* Summary:
* This makes a test access to our target variable in flash with all required 
* steps before and after the access
*
* Parameters:
*  none
*
* Return:
*  none
*
*******************************************************************************/
static void executeTestAccess(void)
{
    /* Initialize the read data variable which might contain data from the previous access */
    g_flashReadData = 0;
    /* Clear flag that is used to detect the IRQ occurrence */
    g_faultIrqOccurred = false;
    /* Invalidate the respective address in the D-Cache to ensure that the test access hits the flash controller */
    SCB_InvalidateDCache_by_Addr((void*)&FLASH_TEST_VAR, sizeof(FLASH_TEST_VAR));
    /* Make the test access. The address of the test variable is used to prevent compiler optimization */
    g_flashReadData = *((volatile const uint64_t*)&FLASH_TEST_VAR);
    /* Add a data synchronization barrier to enforce the ordering of the test access with subsequent data accesses */
    __DSB();
}
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Will hold the correct parity for the configured test value */
    uint8_t correctParity;

    /* Initialize the device and board peripherals */
    if (cybsp_init() != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    SCB_DisableICache();
    SCB_DisableDCache();

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    /* Debug UART init */
    result = (cy_rslt_t)Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    Cy_SCB_UART_Enable(UART_HW);

    /* Setup the HAL UART */
    result = mtb_hal_uart_setup(&UART_hal_obj, &UART_hal_config, &UART_context, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    result = cy_retarget_io_init(&UART_hal_obj);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** Flash ECC Error Injection Code Example ******************\r\n\r\n");

    initFaultHandling();

    correctParity = getParityForValue(FLASH_TEST_VAR);

    printf("Info about test variable in flash\r\n");
    printf("- Address:            0x%" PRIx32 "\r\n", (uint32_t)&FLASH_TEST_VAR);
    printf("- Correct ECC Parity: 0x%02x \r\n", correctParity);
    printf("\r\n");

    printf("Test step 1: Inject correct parity to prove correctness of ECC parity calculation\r\n");    

    injectParity((uint32_t)&FLASH_TEST_VAR, correctParity);
    executeTestAccess();
    if (g_faultIrqOccurred)
    {
        printf("TEST ERROR: Unexpected fault occurred!\r\n");
    }
    else if (g_flashReadData != CORRECT_DATA)
    {
        printf("TEST ERROR: Incorrect data read!\r\n");
    }
    else
    {
        printf("TEST OK!\r\n");
    }
    printf("\r\n");

    printf("Test step 2: Inject parity with 1-bit error to test correctable ECC fault\r\n");

    injectParity((uint32_t)&FLASH_TEST_VAR, correctParity ^ 0x01); /* Flip 1 bit */
    executeTestAccess();
    Cy_SysLib_Delay(INTERVAL_MS);
    if (g_faultIrqOccurred == false)
    {
        printf("TEST ERROR: Fault IRQ has not occurred!\r\n");
    }
    printf("\r\n");

    printf("Test step 3: Inject parity with 2-bit error to test non-correctable ECC fault\r\n");

    injectParity((uint32_t)&FLASH_TEST_VAR, correctParity ^ 0x03); /* Flip 2 bits */
    executeTestAccess();

    /* A non-correctable ECC error in the Code Flash results in a bus error and hence a CPU HardFault exception
       unless MAIN_ERR_SILENT bit in FLASHC_FLASH_CTL would be set. Optionally, the fault struct can catch a
       non-correctable ECC fault as well, but this is not demonstrated in this example because the HardFault
       exception has the higher priority anyway. */

    printf("TEST ERROR: Execution flow should not have reached this point!\r\n");

    for (;;)
    {
    }
}

/* [] END OF FILE */
