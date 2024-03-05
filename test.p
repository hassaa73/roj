#include "Motor_Control_Component.h"
#define FTM_MOTOR FTM0
#define FTM_CHANNEL_DC_MOTOR kFTM_Chnl_0
#define FTM_CHANNEL_SERVO_MOTOR kFTM_Chnl_3

QueueHandle_t motorQueue;
QueueHandle_t angleQueue;

void setupMotorComponent()
{
	setupMotorPins();

	setupDCMotor();
	setupServo();
	BaseType_t status;


		for(volatile int i = 0U; i < 1000000; i++)
			__asm("NOP");
    /*************** Motor Task ***************/
	//Create Motor Queue
	motorQueue = xQueueCreate(5, sizeof(int));

	//Create Motor Task
	status = xTaskCreate(motorTask, "motor", 200, (void*)motorQueue, 2, NULL);
	if (status != pdPASS)
	{
		PRINTF("task creation failed!.\r\n");
		while(1);
	}

    /*************** Position Task ***************/
	//Create Angle Queue
	angleQueue = xQueueCreate(5, sizeof(int));

	//Create Position Task
	status = xTaskCreate(positionTask, "motor", 200, (void*)angleQueue, 2, NULL);
		if (status != pdPASS)
		{
			PRINTF("task creation failed!.\r\n");
			while(1);
		}

}

void setupMotorPins()
{

    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);

    CLOCK_EnableClock(kCLOCK_PortC);
    //DC MOTOR
    PORT_SetPinMux(PORTC, 1U,  kPORT_MuxAlt4);

    //Servo motor
    PORT_SetPinMux(PORTA, 6U, kPORT_MuxAlt3);


//    PORT_SetPinMux(PORTC, 15U,  kPORT_MuxAlt3);
//    PORT_SetPinMux(PORTC, 14U,  kPORT_MuxAlt3);
//    PORT_SetPinMux(PORTC, 13U,  kPORT_MuxAlt3);
//    PORT_SetPinMux(PORTC, 12U,  kPORT_MuxAlt3);


    //Configure PWM pins for DC and Servo motors
}

void setupDCMotor()
{
	//Initialize PWM for DC motor
	//DC
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = FTM_CHANNEL_DC_MOTOR;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTOR, &ftmInfo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U,
			CLOCK_GetFreq(kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}

void setupServo() {
	//Initialize PWM for Servo motor
	//SERVO
	ftm_config_t ftmInfo_servo;
	ftm_chnl_pwm_signal_param_t ftmParam_servo;
	ftm_pwm_level_select_t pwmLevel_servo = kFTM_HighTrue;
	ftmParam_servo.chnlNumber = FTM_CHANNEL_SERVO_MOTOR;
	ftmParam_servo.level = pwmLevel_servo;
	ftmParam_servo.dutyCyclePercent = 7;
	ftmParam_servo.firstEdgeDelayPercent = 0U;
	ftmParam_servo.enableComplementary = false;
	ftmParam_servo.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo_servo);
	ftmInfo_servo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(FTM_MOTOR, &ftmInfo_servo);
	FTM_SetupPwm(FTM_MOTOR, &ftmParam_servo, 1U, kFTM_EdgeAlignedPwm, 50U,
			CLOCK_GetFreq(kCLOCK_BusClk));
	FTM_StartTimer(FTM_MOTOR, kFTM_SystemClock);
}

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;

	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTORS));

	mod = FTM_MOTORS->MOD;
	if(dutyCycle == 0U)
	{
		/* Signal stays low */
		cnv = 0;
	}
	else
	{
		cnv = mod * dutyCycle;
		/* For 100% duty cycle */
		if (cnv >= mod)
		{
			cnv = mod + 1U;
		}
	}

	FTM_MOTORS->CONTROLS[channel].CnV = cnv;
}

void motorTask(void* pvParameters)
{
	BaseType_t status;

	//Motor task implementation
	QueueHandle_t queue = (QueueHandle_t) pvParameters;
//	dutyCycle = inputSpeed * 0.025f / 100.0f + 0.0615;
//	updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dutyCycle);

	while(1){
		status = xQueueReceive(queue, (void*)int, portMAX_DELAY);

	}
}

void positionTask(void* pvParameters)
{
	//Position task implementation
}
