/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the PSoC 4: MSCLP CAPSENSE&trade; Low Power
 *              Example for ModusToolbox.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * $ Copyright 2021-2023 Cypress Semiconductor $
 *******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/


 
 //HAY QUE INCLUIR LAS FUNCIONALIDADES DEL IIR Y DEL C2C

#include "cy_pdl.h"
#include "cybsp.h"
#include "cycfg.h"
#include "cycfg_capsense.h"


//Archivos necesarios para el clock
#include "cycfg_peripherals.h"
#include "cy_wdt.h"

void mcwdt_isr(void)
{
    /* Clear interrupt */
    Cy_WDT_ClearInterrupt();
    
    /* Tu código aquí */
}

/*******************************************************************************
 * User configurable Macros
 ********************************************************************************/
/*Enables the Runtime measurement functionality used to for processing time measurement */
//Esto hay que activarlo para hacer las pruebas
#define ENABLE_RUN_TIME_MEASUREMENT     (0u)


#define ENABLE_LP_TIME_MEASUREMENT      (1u)

/* Enable this, if Tuner needs to be enabled */
#define ENABLE_TUNER                    (1u)

/*Enable PWM controlled LEDs... acá prenden los leds*/
// #define ENABLE_PWM_LED                  (1u)

/* 128Hz Refresh rate in Active mode */
#define ACTIVE_MODE_REFRESH_RATE        (128u)


/* TIEMPO PARA PASAR A ALR SI NO HAY ACTIVIDAD*/
#define ACTIVE_MODE_TIMEOUT_SEC         (10u)




/* SCAN TIME DEL ACTIVE MODE*/
#define ACTIVE_MODE_FRAME_SCAN_TIME     (46u)



/* TIEMPO DE PROCESAMIENTO*/
#define ACTIVE_MODE_PROCESS_TIME        (23u)


//REFRESH RATE DE 4HZ EN WOT

#define WOT_MODE_REFRESH_RATE           (4u)

// #define ALR_MODE_PROCESS_TIME        (23u)

#define WOT_MODE_FRAME_SCAN_TIME        (20u)  

#define DEEP_SLEEP_DEACT_TIME           (1800000000u)



/*******************************************************************************
 * Macros
 ********************************************************************************/
#define CAPSENSE_MSC0_INTR_PRIORITY     (3u)

#define CY_ASSERT_FAILED                (0u)

/* EZI2C interrupt priority must be higher than CAPSENSE&trade; interrupt. */
#define EZI2C_INTR_PRIORITY             (2u)

#define ILO_FREQ                        (40000u) // 25 HZ el oscilador interno
#define TIME_IN_US                      (1000000u)

#define MINIMUM_TIMER                   (TIME_IN_US / ILO_FREQ)
#if ((TIME_IN_US / ACTIVE_MODE_REFRESH_RATE) > (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))

#define ACTIVE_MODE_TIMER           (TIME_IN_US / ACTIVE_MODE_REFRESH_RATE - \
        (ACTIVE_MODE_FRAME_SCAN_TIME + ACTIVE_MODE_PROCESS_TIME))
#elif
#define ACTIVE_MODE_TIMER           (MINIMUM_TIMER)
#endif

//WOT REFRESH RATE
#if ((TIME_IN_US / WOT_MODE_REFRESH_RATE) > (WOT_MODE_FRAME_SCAN_TIME + WOT_MODE_PROCESS_TIME))

#define WOT_MODE_TIMER                  (TIME_IN_US / WOT_MODE_REFRESH_RATE - WOT_MODE_FRAME_SCAN_TIME)

#else
#define WOT_MODE_TIMER                  (MINIMUM_TIMER)
#endif

#define ACTIVE_MODE_TIMEOUT             (ACTIVE_MODE_REFRESH_RATE * ACTIVE_MODE_TIMEOUT_SEC)


// TIMEOUT EN WOT

#define WOT_MODE_TIMEOUT_SEC            (16u) //16 usegundos

// Timeout establecido para un caso de 8 sensores 21 raw counts
#define WOT_MODE_TIMEOUT                ((TIME_IN_US / WOT_MODE_REFRESH_RATE)*WOT_MODE_TIMEOUT_SEC)


#define TIMEOUT_RESET                   (0u)

#if ENABLE_RUN_TIME_MEASUREMENT
#define SYS_TICK_INTERVAL           (0x00FFFFFF)
#define TIME_PER_TICK_IN_US         ((float)1/CY_CAPSENSE_CPU_CLK)*TIME_IN_US
#endif


//Logica incorrecta
// LOW POWER TIME MEASUREMENT USING ILO AT 40KHZ
// #if ENABLE_LP_TIME_MEASUREMENT
// // En torno a 7 minutos para overflow
// #define SYS_TICK_INTERVAL           (0x00FFFFFF)
// #define TIME_PER_TICK_IN_US        ((float)1/CY_SYSCLK_ILO_FREQ)*TIME_IN_US
// #endif

/* Touch status of the proximity sensor */
#define TOUCH_STATE                     (3u)

/* Proximity status of the proximity sensor */
#define PROX_STATE                      (1u)




// WATCHDOGTIMER DEFINITIONS
#define 	CY_MCWDT_CTR0   (1UL << CY_MCWDT_CTR0_Pos)




// Flag del vaciado de memoria
bool mem_flag = false;



/*****************************************************************************
 * Finite state machine states for device operating states
 *****************************************************************************/
typedef enum
{
    WOT_MODE = 0x01u,
    ACTIVE_MODE = 0x02u,    /* Active mode - All the sensors are scanned in this state  * with highest refresh rate */
    DEACT_MODE = 0X03u        /* Wake on Touch (WoT) mode - Low Power sensors are scanned* in this state with lowest refresh rate */
} APPLICATION_STATE;



/*******************************************************************************
 * Function Prototypes
 ********************************************************************************/
static void initialize_capsense(void);
static void capsense_msc0_isr(void);

static void ezi2c_isr(void);
static void initialize_capsense_tuner(void);

#if ENABLE_RUN_TIME_MEASUREMENT
static void init_sys_tick();
static void start_runtime_measurement();
static uint32_t stop_runtime_measurement();
#endif


volatile bool wdt_flag = false;
volatile uint32_t wdt_ticks = 0;

// Función del contador WDT WatchDog Timer
void WDT_IRQHandler(void){
    Cy_WDT_ClearInterrupt();
    wdt_ticks++;
     if (wdt_ticks >= 1800u)
    {
        wdt_ticks = 0;
        wdt_flag = true;   // Aquí prendes tu flag
    }
}


/* Deep Sleep Callback function */
void register_callback(void);
cy_en_syspm_status_t deep_sleep_callback(cy_stc_syspm_callback_params_t *callbackParams,
        cy_en_syspm_callback_mode_t mode);



        

/*******************************************************************************
 * Global Definitions
 *******************************************************************************/

/* Variables holds the current low power state [ACTIVE, ALR or WOT] */
APPLICATION_STATE capsense_state;

cy_stc_scb_ezi2c_context_t ezi2c_context;

/* Callback parameters for custom, EzI2C */

/* Callback parameters for EzI2C */
cy_stc_syspm_callback_params_t ezi2cCallbackParams =
{
        .base       = SCB1,
        .context    = &ezi2c_context
};

/* Callback parameters for custom callback */
cy_stc_syspm_callback_params_t deepSleepCallBackParams = {
        .base       =  NULL,
        .context    =  NULL
};

/* Callback declaration for EzI2C Deep Sleep callback */
cy_stc_syspm_callback_t ezi2cCallback =
{
        .callback       = (Cy_SysPmCallback)&Cy_SCB_EZI2C_DeepSleepCallback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &ezi2cCallbackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 0
};

/* Callback declaration for Custom Deep Sleep callback */
cy_stc_syspm_callback_t deepSleepCb =
{
        .callback       = &deep_sleep_callback,
        .type           = CY_SYSPM_DEEPSLEEP,
        .skipMode       = 0UL,
        .callbackParams = &deepSleepCallBackParams,
        .prevItm        = NULL,
        .nextItm        = NULL,
        .order          = 2
};

//ELIMINAR
static cy_stc_syspm_callback_params_t capsenseCbParams = {
    .base = NULL,
    .context = &cy_capsense_context  // puntero a tu estructura de contexto CapSense
};

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  - initial setup of device
 *  - initialize CAPSENSE&trade;
 *  - initialize tuner communication
 *  - scan touch input continuously at 3 different power modes
 *  - LED for touch indication
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    uint32_t capsense_state_timeout;
    uint32_t interruptStatus;
    // Variables WOT
    mem_flag = false;

    #if ENABLE_RUN_TIME_MEASUREMENT
    static uint32_t active_processing_time;
    #endif

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    #if ENABLE_RUN_TIME_MEASUREMENT
    init_sys_tick();
    #endif

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize EZI2C */
    initialize_capsense_tuner();

    // #if ENABLE_PWM_LED
    // /* Initialize PWM block */
    // (void)Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
    // /* Enable the initialized PWM */
    // Cy_TCPWM_Enable_Multiple(CYBSP_PWM_HW, CYBSP_PWM_MASK);
    // /* Then start the PWM */
    // Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);
    // #endif

    /* Register callbacks */
    register_callback();

    /* Define initial state of the device and the corresponding refresh rate*/
    // capsense_state = ACTIVE_MODE;

    //Inicializamos en WoT mode
    capsense_state = WOT_MODE;
    capsense_state_timeout = WOT_MODE_TIMEOUT;

    /* Initialize MSC CAPSENSE&trade; */
    initialize_capsense();

    /* Measures the actual ILO frequency and compensate MSCLP wake up timers */
    Cy_CapSense_IloCompensate(&cy_capsense_context);

    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
    Cy_CapSense_ConfigureMsclpTimer(WOT_MODE_TIMER, &cy_capsense_context);


    // Configuraciones del WDT

    Cy_WDT_Unlock();

    Cy_WDT_SetIgnoreBits(0u);
    Cy_WDT_SetMatch(40000u);

    Cy_WDT_ClearInterrupt();
    // NVIC_EnableIRQ(WDT_IRQHandler);
    // NVIC_EnableIRQ(WCO_IRQn);
    Cy_WDT_Enable();
    uint32_t conteo = Cy_WDT_GetCount();
    CY_WDT_Lock();

    //Fin configuraciones WDT

    for (;;)
    {
        switch(capsense_state)
        {
            /* Wake On Touch Mode */
            case WOT_MODE :
                

                Cy_CapSense_ScanAllLpSlots(&cy_capsense_context);

                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    Cy_SysPm_CpuEnterDeepSleep();

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();
                }

                Cy_SysLib_ExitCriticalSection(interruptStatus);

                if (Cy_CapSense_IsAnyLpWidgetActive(&cy_capsense_context))
                {
                    capsense_state = ACTIVE_MODE;
                    //Frecuencia de muestreo de active mode
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;

                    // #if ENABLE_PWM_LED
                    // /* Initialize PWM block */
                    // (void)Cy_TCPWM_PWM_Init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config);
                    // /* Enable the initialized PWM */
                    // Cy_TCPWM_Enable_Multiple(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    // /* Then start the PWM */
                    // Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_HW, CYBSP_PWM_MASK);
                    // #endif

                    /* Configure the MSCLP wake up timer as per the ACTIVE mode refresh rate */
                    Cy_CapSense_ConfigureMsclpTimer(ACTIVE_MODE_TIMER, &cy_capsense_context);
                }

                //Preguntar si es necesario insertar aquí una critical section
                //Revisar el ultimo address pero igual dejar un timeout

                else if(wdt_flag){
                    mem_flag = true;
                    capsense_state = ACTIVE_MODE;
                    //Frecuencia de muestreo de active mode
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;
                    // active_processing_time = 0;
                }

                else
                {
                    //TIMEOUT DEL WOT
                    // wot_timeout--;
                    // if(wot_timeout == TIMEOUT_RESET)
                    // {
                    //     capsense_state = ACTIVE_MODE;
                    //     mem_flag = true;
                   Cy_CapSense_ConfigureMsclpTimer(WOT_MODE_TIMER, &cy_capsense_context);

                    // }
                    

                    /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                }

                break;
                /* End of "WAKE_ON_TOUCH_MODE" */




            case ACTIVE_MODE:

                Cy_CapSense_ScanAllSlots(&cy_capsense_context);

                // Deshabilita, interrumpe y retorna un valor que indica si
                // la interrupcion estaba previamente habilitada (0) o si no (1)
                // Entra en una sección crítica que deshabilita las interrupciones
                // globales
                interruptStatus = Cy_SysLib_EnterCriticalSection();

                while (Cy_CapSense_IsBusy(&cy_capsense_context))
                {
                    
                    Cy_SysPm_CpuEnterDeepSleep();
                    // Cy_SysPm_CpuEnterSleep();

                    //Rehabilita las interrupciones si se llamo previamente
                    //el enterCriticalSection

                    Cy_SysLib_ExitCriticalSection(interruptStatus);

                    /* This is a place where all interrupt handlers will be executed */
                    interruptStatus = Cy_SysLib_EnterCriticalSection();
                }

                // Run time Measurement

                #if ENABLE_RUN_TIME_MEASUREMENT
                active_processing_time=0;
                start_runtime_measurement();
                #endif
                 //Salimos de seccion critica
                Cy_SysLib_ExitCriticalSection(interruptStatus);



                /*Aqui se:
                -Aplican los filtros
                -Actualizan los umbrales para el auto-tuning
                -Actualizan los baselines
                -Actualizan los estados de salida de sensores y widgets
                No llamar sin escaneos previos*/

                Cy_CapSense_ProcessAllWidgets(&cy_capsense_context);

                /* Scan, process and check the status of the all Active mode sensors */
                // aqui se detectan los toques
                if(Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
                {
                    capsense_state_timeout = ACTIVE_MODE_TIMEOUT;
                    
                }

                else if(mem_flag){
                    // Vaciar la memoria


                    capsense_state = WOT_MODE;
                    //Frecuencia de muestreo de active mode
                    capsense_state_timeout = WOT_MODE_TIMEOUT;
                    mem_flag = false;


                }


                else
                {
                    capsense_state_timeout--;

                    if(TIMEOUT_RESET == capsense_state_timeout)
                    {
                        capsense_state = WOT_MODE;
                        capsense_state_timeout = WOT_MODE_TIMEOUT;

                        /* Configure the MSCLP wake up timer as per the ALR mode refresh rate */
                        Cy_CapSense_ConfigureMsclpTimer(WOT_MODE_TIMER, &cy_capsense_context);
                    }
                }

                #if ENABLE_RUN_TIME_MEASUREMENT
                active_processing_time=stop_runtime_measurement();
                #endif

                // #if ENABLE_PWM_LED
                // led_control();
                // #endif


                // CONDICION DE PASO A WOT

                break;
                /* End of ACTIVE_MODE */

                /*START OF DEACTIVE_MODE*/
                case DEACT_MODE:
                // Comando para apagar el sistema
                // Cy_CapSense_DeepSleepCallback(&capsenseCbParams, CY_SYSPM_CHECK_READY);
                Cy_SysPm_CpuEnterDeepSleep();
                // Cy_SysPm_CpuEnterDeepSleep();

    
                // if (active_processing_time >= DEEP_SLEEP_DEACT_TIME){
                 if (wdt_flag){
                    capsense_state = WOT_MODE;
                    capsense_state_timeout = WOT_MODE_TIMEOUT;
                    Cy_CapSense_ConfigureMsclpTimer(WOT_MODE_TIMER, &cy_capsense_context);
                 }
                break;


          
                

                

            default:
                /**  Unknown power mode state. Unexpected situation.  **/
                CY_ASSERT(CY_ASSERT_FAILED);
                break;
        }


        #if ENABLE_TUNER
        /* Establishes synchronized communication with the CAPSENSE&trade; Tuner tool */
        Cy_CapSense_RunTuner(&cy_capsense_context);
        #endif
    }
}

/*******************************************************************************
 * Function Name: initialize_capsense
 ********************************************************************************
 * Summary:
 *  This function initializes the CAPSENSE&trade; and configures the CAPSENSE&trade;
 *  interrupt.
 *
 *******************************************************************************/
static void initialize_capsense(void)
{
    cy_capsense_status_t status = CY_CAPSENSE_STATUS_SUCCESS;

    /* CAPSENSE&trade; interrupt configuration MSCLP 0 */
    const cy_stc_sysint_t capsense_msc0_interrupt_config =
    {
            .intrSrc = CY_MSCLP0_LP_IRQ,
            .intrPriority = CAPSENSE_MSC0_INTR_PRIORITY,
    };

    /* Capture the MSC HW block and initialize it to the default state. */
    status = Cy_CapSense_Init(&cy_capsense_context);

    if (CY_CAPSENSE_STATUS_SUCCESS == status)
    {
        /* Initialize CAPSENSE&trade; interrupt for MSC 0 */
        Cy_SysInt_Init(&capsense_msc0_interrupt_config, capsense_msc0_isr);
        NVIC_ClearPendingIRQ(capsense_msc0_interrupt_config.intrSrc);
        NVIC_EnableIRQ(capsense_msc0_interrupt_config.intrSrc);

        status = Cy_CapSense_Enable(&cy_capsense_context);
    }

    if(status != CY_CAPSENSE_STATUS_SUCCESS)
    {
        /* This status could fail before tuning the sensors correctly.
         * Ensure that this function passes after the CAPSENSE&trade; sensors are tuned
         * as per procedure give in the Readme.md file */
    }
}

/*******************************************************************************
 * Function Name: capsense_msc0_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from CAPSENSE&trade; MSC0 block.
 *
 *******************************************************************************/
static void capsense_msc0_isr(void)
{
    Cy_CapSense_InterruptHandler(CY_MSCLP0_HW, &cy_capsense_context);
}

/*******************************************************************************
 * Function Name: initialize_capsense_tuner
 ********************************************************************************
 * Summary:
 *  EZI2C module to communicate with the CAPSENSE&trade; Tuner tool.
 *
 *******************************************************************************/
static void initialize_capsense_tuner(void)
{
    cy_en_scb_ezi2c_status_t status = CY_SCB_EZI2C_SUCCESS;

    /* EZI2C interrupt configuration structure */
    const cy_stc_sysint_t ezi2c_intr_config =
    {
            .intrSrc = CYBSP_EZI2C_IRQ,
            .intrPriority = EZI2C_INTR_PRIORITY,
    };

    /* Initialize the EzI2C firmware module */
    status = Cy_SCB_EZI2C_Init(CYBSP_EZI2C_HW, &CYBSP_EZI2C_config, &ezi2c_context);

    if(status != CY_SCB_EZI2C_SUCCESS)
    {
        CY_ASSERT(CY_ASSERT_FAILED);
    }

    Cy_SysInt_Init(&ezi2c_intr_config, ezi2c_isr);
    NVIC_EnableIRQ(ezi2c_intr_config.intrSrc);

    /* Set the CAPSENSE&trade; data structure as the I2C buffer to be exposed to the
     * master on primary slave address interface. Any I2C host tools such as
     * the Tuner or the Bridge Control Panel can read this buffer but you can
     * connect only one tool at a time.
     */
    #if ENABLE_TUNER
    Cy_SCB_EZI2C_SetBuffer1(CYBSP_EZI2C_HW, (uint8_t *)&cy_capsense_tuner,
            sizeof(cy_capsense_tuner), sizeof(cy_capsense_tuner),
            &ezi2c_context);
    #endif
    Cy_SCB_EZI2C_Enable(CYBSP_EZI2C_HW);

}

/*******************************************************************************
 * Function Name: ezi2c_isr
 ********************************************************************************
 * Summary:
 *  Wrapper function for handling interrupts from EZI2C block.
 *
 *******************************************************************************/
static void ezi2c_isr(void)
{
    Cy_SCB_EZI2C_Interrupt(CYBSP_EZI2C_HW, &ezi2c_context);
}

#if ENABLE_RUN_TIME_MEASUREMENT
/*******************************************************************************
 * Function Name: init_sys_tick
 ********************************************************************************
 * Summary:
 *  initializes the system tick with highest possible value to start counting down.
 *
 *******************************************************************************/
static void init_sys_tick()
{
    Cy_SysTick_Init (CY_SYSTICK_CLOCK_SOURCE_CLK_CPU ,SYS_TICK_INTERVAL);
}
#endif

#if ENABLE_RUN_TIME_MEASUREMENT
/*******************************************************************************
 * Function Name: start_runtime_measurement
 ********************************************************************************
 * Summary:
 *  Initializes the system tick counter by calling Cy_SysTick_Clear() API.
 *******************************************************************************/
static void start_runtime_measurement()
{
    Cy_SysTick_Clear();
}

/*******************************************************************************
 * Function Name: stop_runtime_measurement
 ********************************************************************************
 * Summary:
 *  Reads the system tick and converts to time in microseconds(us).
 *
 *  Returns:
 *  runtime - in microseconds(us)
 *******************************************************************************/
#endif

#if ENABLE_RUN_TIME_MEASUREMENT
static uint32_t stop_runtime_measurement()
{
    uint32_t ticks;
    uint32_t runtime;
    ticks=Cy_SysTick_GetValue();
    ticks= SYS_TICK_INTERVAL - Cy_SysTick_GetValue();
    runtime=ticks*TIME_PER_TICK_IN_US;
    return runtime;
}
#endif

// #if ENABLE_PWM_LED
// uint32_t proxSensorStatus;
// uint32_t proxLedBrightness = 0u;
// uint16_t proxMaxRawCount = 0u;
// uint16_t maxDiffCount = 0u;
/*******************************************************************************
 * Function Name: led_control
 ********************************************************************************
 * Summary:
 *  Control LED3 in the kit to show the proximity status
 *  LED3 brightness changes based on proximity distance using PWM(CYBSP_PWM)
 *
 *  Control LED2(CYBSP_USER_LED) in the kit to show the touch status
 *  LED2 turns ON when a touch is detected
 *******************************************************************************/
// void led_control()
// {
//     /* Both LEDs are turned off when proximity/touch is not detected */
//     Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, 0);
//     Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_OFF);

//     if(Cy_CapSense_IsAnyWidgetActive(&cy_capsense_context))
//     {
//         proxSensorStatus = Cy_CapSense_IsProximitySensorActive(CY_CAPSENSE_PROXIMITY0_WDGT_ID, CY_CAPSENSE_PROXIMITY0_SNS0_ID, &cy_capsense_context);
//         proxMaxRawCount = cy_capsense_tuner.widgetContext[CY_CAPSENSE_PROXIMITY0_WDGT_ID].maxRawCount;
//         maxDiffCount = proxMaxRawCount - cy_capsense_tuner.sensorContext[CY_CAPSENSE_PROXIMITY0_SNS0_ID].bsln;
//         proxLedBrightness = ((uint32_t)(cy_capsense_tuner.sensorContext[CY_CAPSENSE_PROXIMITY0_SNS0_ID].diff*CYBSP_PWM_config.period0)/maxDiffCount);

//         if(proxSensorStatus == PROX_STATE)
//         {
//             /* LED Turns ON and brightness changes based on proximity distance */
//             Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, proxLedBrightness);
//         }
//         else if(proxSensorStatus == TOUCH_STATE)
//         {
//             /* LED Turns ON when touch is detected */
//             Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_NUM, CYBSP_LED_ON);
//             /* LED Turns ON and brightness changes based on proximity distance */
//             Cy_TCPWM_PWM_SetCompare0(CYBSP_PWM_HW, CYBSP_PWM_NUM, proxLedBrightness);
//         }
//     }
// }
// #endif

/*******************************************************************************
 * Function Name: register_callback
 ********************************************************************************
 *
 * Summary:
 *  Register Deep Sleep callbacks for EzI2C, SPI components
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void register_callback(void)
{
    /* Register EzI2C Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&ezi2cCallback);

    /* Register Deep Sleep callback */
    Cy_SysPm_RegisterCallback(&deepSleepCb);
}

/*******************************************************************************
 * Function Name: deep_sleep_callback
 ********************************************************************************
 *
 * Summary:
 * Deep Sleep callback implementation. Waits for the completion of SPI transaction.
 * And change the SPI GPIOs to highZ while transition to deep-sleep and vice-versa
 *
 * Parameters:
 *  callbackParams: The pointer to the callback parameters structure cy_stc_syspm_callback_params_t.
 *  mode: Callback mode, see cy_en_syspm_callback_mode_t
 *
 * Return:
 *  Entered status, see cy_en_syspm_status_t.
 *
 *******************************************************************************/
cy_en_syspm_status_t deep_sleep_callback(
        cy_stc_syspm_callback_params_t *callbackParams, cy_en_syspm_callback_mode_t mode)
{
    cy_en_syspm_status_t ret_val = CY_SYSPM_FAIL;

    switch (mode)
    {
        case CY_SYSPM_CHECK_READY:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_CHECK_FAIL:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_BEFORE_TRANSITION:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        case CY_SYSPM_AFTER_TRANSITION:

            ret_val = CY_SYSPM_SUCCESS;
            break;

        default:
            /* Don't do anything in the other modes */
            ret_val = CY_SYSPM_SUCCESS;
            break;
    }
    return ret_val;
}

/* [] END OF FILE */
