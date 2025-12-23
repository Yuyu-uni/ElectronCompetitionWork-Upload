/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gMotorBackup;
DL_TimerG_backupConfig gCacBackup;
DL_TimerA_backupConfig gFollowingBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_Motor_init();
    SYSCFG_DL_Pid_init();
    SYSCFG_DL_Cac_init();
    SYSCFG_DL_Following_init();
    SYSCFG_DL_TJC_init();
    SYSCFG_DL_ADC12_0_init();
    /* Ensure backup structures have no valid state */
	gMotorBackup.backupRdy 	= false;
	gCacBackup.backupRdy 	= false;
	gFollowingBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(Motor_INST, &gMotorBackup);
	retStatus &= DL_TimerG_saveConfiguration(Cac_INST, &gCacBackup);
	retStatus &= DL_TimerA_saveConfiguration(Following_INST, &gFollowingBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(Motor_INST, &gMotorBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(Cac_INST, &gCacBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(Following_INST, &gFollowingBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(Motor_INST);
    DL_TimerG_reset(Pid_INST);
    DL_TimerG_reset(Cac_INST);
    DL_TimerA_reset(Following_INST);
    DL_UART_Main_reset(TJC_INST);
    DL_ADC12_reset(ADC12_0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(Motor_INST);
    DL_TimerG_enablePower(Pid_INST);
    DL_TimerG_enablePower(Cac_INST);
    DL_TimerA_enablePower(Following_INST);
    DL_UART_Main_enablePower(TJC_INST);
    DL_ADC12_enablePower(ADC12_0_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C0_IOMUX,GPIO_Motor_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C0_PORT, GPIO_Motor_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_Motor_C1_IOMUX,GPIO_Motor_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_Motor_C1_PORT, GPIO_Motor_C1_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_TJC_IOMUX_TX, GPIO_TJC_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_TJC_IOMUX_RX, GPIO_TJC_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutputFeatures(LED1_PIN_22_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalOutput(Ain_PIN_0_IOMUX);

    DL_GPIO_initDigitalOutput(Ain_PIN_1_IOMUX);

    DL_GPIO_initDigitalOutput(Bin_PIN_2_IOMUX);

    DL_GPIO_initDigitalOutput(Bin_PIN_3_IOMUX);

    DL_GPIO_initDigitalInputFeatures(Follow_one_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_two_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_three_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_four_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_five_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_six_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_seven_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(Follow_eight_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(Gray_Address_FPIN_0_IOMUX);

    DL_GPIO_initDigitalOutput(Gray_Address_FPIN_1_IOMUX);

    DL_GPIO_initDigitalOutput(Gray_Address_FPIN_2_IOMUX);

    DL_GPIO_clearPins(GPIOB, LED1_PIN_22_PIN |
		Ain_PIN_0_PIN |
		Ain_PIN_1_PIN |
		Bin_PIN_2_PIN |
		Bin_PIN_3_PIN |
		Gray_Address_FPIN_0_PIN |
		Gray_Address_FPIN_1_PIN |
		Gray_Address_FPIN_2_PIN);
    DL_GPIO_enableOutput(GPIOB, LED1_PIN_22_PIN |
		Ain_PIN_0_PIN |
		Ain_PIN_1_PIN |
		Bin_PIN_2_PIN |
		Bin_PIN_3_PIN |
		Gray_Address_FPIN_0_PIN |
		Gray_Address_FPIN_1_PIN |
		Gray_Address_FPIN_2_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gMotorClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gMotorConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 10000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_Motor_init(void) {

    DL_TimerA_setClockConfig(
        Motor_INST, (DL_TimerA_ClockConfig *) &gMotorClockConfig);

    DL_TimerA_initPWMMode(
        Motor_INST, (DL_TimerA_PWMConfig *) &gMotorConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(Motor_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(Motor_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(Motor_INST);


    
    DL_TimerA_setCCPDirection(Motor_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );
    DL_TimerA_enableShadowFeatures(Motor_INST);


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPidClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Pid_INST_LOAD_VALUE = (10 ms * 4000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gPidTimerConfig = {
    .period     = Pid_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Pid_init(void) {

    DL_TimerG_setClockConfig(Pid_INST,
        (DL_TimerG_ClockConfig *) &gPidClockConfig);

    DL_TimerG_initTimerMode(Pid_INST,
        (DL_TimerG_TimerConfig *) &gPidTimerConfig);
    DL_TimerG_enableInterrupt(Pid_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(Pid_INST_INT_IRQN, 1);
    DL_TimerG_enableClock(Pid_INST);





}

/*
 * Timer clock configuration to be sourced by LFCLK /  (4096 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   20.48 Hz = 4096 Hz / (8 * (199 + 1))
 */
static const DL_TimerG_ClockConfig gCacClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_LFCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 199U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Cac_INST_LOAD_VALUE = (19 * 20.48 Hz) - 1
 */
static const DL_TimerG_TimerConfig gCacTimerConfig = {
    .period     = Cac_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_ONE_SHOT_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_Cac_init(void) {

    DL_TimerG_setClockConfig(Cac_INST,
        (DL_TimerG_ClockConfig *) &gCacClockConfig);

    DL_TimerG_initTimerMode(Cac_INST,
        (DL_TimerG_TimerConfig *) &gCacTimerConfig);
    DL_TimerG_enableInterrupt(Cac_INST , DL_TIMERG_INTERRUPT_LOAD_EVENT);
	NVIC_SetPriority(Cac_INST_INT_IRQN, 0);
    DL_TimerG_enableClock(Cac_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 4000000 Hz / (8 * (3 + 1))
 */
static const DL_TimerA_ClockConfig gFollowingClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 3U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * Following_INST_LOAD_VALUE = (1ms * 1000000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gFollowingTimerConfig = {
    .period     = Following_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_Following_init(void) {

    DL_TimerA_setClockConfig(Following_INST,
        (DL_TimerA_ClockConfig *) &gFollowingClockConfig);

    DL_TimerA_initTimerMode(Following_INST,
        (DL_TimerA_TimerConfig *) &gFollowingTimerConfig);
    DL_TimerA_enableInterrupt(Following_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(Following_INST);





}



static const DL_UART_Main_ClockConfig gTJCClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gTJCConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_TJC_init(void)
{
    DL_UART_Main_setClockConfig(TJC_INST, (DL_UART_Main_ClockConfig *) &gTJCClockConfig);

    DL_UART_Main_init(TJC_INST, (DL_UART_Main_Config *) &gTJCConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(TJC_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(TJC_INST, TJC_IBRD_32_MHZ_115200_BAUD, TJC_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(TJC_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(TJC_INST_INT_IRQN, 0);


    DL_UART_Main_enable(TJC_INST);
}

/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void)
{
    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_initSingleSample(ADC12_0_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_SOFTWARE,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH1,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC12_0_INST,4);
    DL_ADC12_enableConversions(ADC12_0_INST);
}

