#include "stm32f10x.h"
#include <stdio.h>
#include "LSM303DLHC/inc/LSM303DLHC.h"
#include "iNemo_Led/iNemoLed.h"
#include "STM32F1_VirtualCom/Source/inc/STM32F1_VC_General.h"
#include "iNEMO_Button.h"
#include "L3Gx/inc/L3Gx.h"
#include "iNEMO_AHRS_v121/inc/iNEMO_AHRS.h"
#include "tm_inemo_nrf24l01.h"

//------------------------ Definizione Variabili Globali ------------------------------

uint8_t dataOut[32],dataRec[64];


typedef struct{
		float time;
		float q0;
		float q1;
		float q2;
		float q3;
} Frame;

Frame send_data;

Frame data_received;
long timer = 0;
int i = 0;
/**
 * @brief Accelerometer sensor init structure
 */
LSMAccInit LSMAccInitStructure;


/**
* @brief Accelerometer high pass filter init structure
*/
LSMAccFilterInit LSMAccFilterInitStructure;


/**
* @brief Magnetometer sensor init structure
*/
LSMMagInit LSMMagInitStructure;


/**
 * @brief Gyroscopic sensor init structure
 */
L3GInit L3GInitStructure;


/**
 * @brief Acceleration and Magnetic field values
 */
float fAccXYZ[3], fMagXYZ[3];


/**
 * @brief Gyroscope data
 */
float fGyroXYZ[3];

/**
 * @brief AHRS filter input data structure
 */
iNEMO_SENSORDATA xSensorData;

iNEMO_EULER_ANGLES xEulerAngles={0};

iNEMO_QUAT  xQuat={0};

FlagStatus xTim2Raised = RESET;

uint8_t iter=0;
// ----------------------------------------------------------------------------

// -----------------------------Definizione Funzioni---------------------------
void prvFindFactors(u32 n, uint16_t *a, uint16_t *b)
{
	/** This function is copied from the ST STR7 library and is
	 * copyright STMicroelectronics.  Reproduced with permission.
	*/

	uint16_t b0;
	uint16_t a0;
	long err, err_min=n;


	*a = a0 = ((n-1)/0xffff) + 1;
	*b = b0 = n / *a;

	for (; *a < 0xffff-1; (*a)++)
	{
		*b = n / *a;
		err = (long)*a * (long)*b - (long)n;
		if (abs(err) > (*a / 2))
		{
			(*b)++;
			err = (long)*a * (long)*b - (long)n;
		}
		if (abs(err) < abs(err_min))
		{
			err_min = err;
			a0 = *a;
			b0 = *b;
			if (err == 0) break;
		}
	}

	*a = a0;
	*b = b0;
}
/**
 * @brief Configures the timer 2
 */
void iNemoTimerConfig(void)
{
  unsigned short a;
  unsigned short b;
  unsigned long n;
  /* This value is the frequency interrupts in Hz */
  unsigned char frequency = 50;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable timer clocks */
  RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

  TIM_TimeBaseStructInit( &TIM_TimeBaseStructure );

  /* Time base configuration for timer 2 - which generates the interrupts. */
  n = SystemCoreClock/frequency;

  prvFindFactors( n, &a, &b );
  TIM_TimeBaseStructure.TIM_Period = b - 1;
  TIM_TimeBaseStructure.TIM_Prescaler = a - 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );
  TIM_ARRPreloadConfig( TIM2, ENABLE );

  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 13;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init( &NVIC_InitStructure );

}


/**
 * @brief Start and stop the timer used by the iNemo Data Task.
 * @param command start the timer if ENABLE, stop the timer if DISABLE.
 * @retval None.
 */
void Enable_Timer(FunctionalState command) {
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TIM_ITConfig( TIM2, TIM_IT_Update, command );
	TIM_Cmd(TIM2, command);
}


/**
 * @brief This function handles TIM2 global interrupt request by resuming the
 * iNemoData task.
 */
void TIM2_IRQHandler(void)
{
  if(TIM_GetITStatus(TIM2, TIM_IT_Update))
  {
    xTim2Raised=SET;

    /* Clear the IRQ bit */
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  }

}

/*
void EXTI15_10_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line10)!= RESET)
  {
	  printf("Pulsante Premuto \n");
	  i++;
	  // inserire routine che fermi il ciclo al click del pulsante.
	  if(i==2)
		  {
		  	  int j;
		      for(j=0;j<10;j++) endofdata[j] = 0;

		  	  iNEMO_Led_Off(LED1);
		  	  i=0;
		  	  timer = 0;
		  }
  }

  EXTI_ClearITPendingBit(EXTI_Line10);

}
*/

// --------------------------------------------------------------------------------------
int main(void)
{
	TM_NRF24L01_Init(15, 32);

				//Set 2MBps data rate and -18dBm output power
				TM_NRF24L01_SetRF(TM_NRF24L01_DataRate_2M, TM_NRF24L01_OutputPower_0dBm);

				 uint8_t MyAddress[] = {
					0xE7,
					0xE7,
					0xE7,
					0xE7,
					0xE7
				  };
				  // Receiver address
				  uint8_t TxAddress[] = {
					0x7E,
					0x7E,
					0x7E,
					0x7E,
					0x7E
				  };

				//Set my address, 5 bytes
				TM_NRF24L01_SetMyAddress(MyAddress);
				//Set TX address, 5 bytes
				TM_NRF24L01_SetTxAddress(TxAddress);


	  for(uint32_t i=0; i<0xFFFFF;i++);

	  /* Initialize the iNemo LED */
	  iNEMO_Led_Init(LED1);
	  iNEMO_Button_Init(BUTTON_KEY, BUTTON_MODE_EXTI);
	  iNEMO_Button_Interrupt_Cmd(BUTTON_KEY,ENABLE);
	  printf("Led e Bottone Inizializzato \n");
	  Stm32f1VCInit();

	  printf("VCP Inizializzato \n");
	  iNemoTimerConfig();

	  printf("Timer Inizializzato \n");



	  delay_ms(300);

	  printf("Entro nel ciclone \n");

	  while(1)
	  {

		     uint8_t size = 0;
		     uint8_t* ptrdata = dataRec;

		     TM_NRF24L01_ReceivePacket(ptrdata, &size);

		     printf("Dato ricevuto \n");
		     if(size==20) // attenzione che qui ora è 20!
		     {
		    	     memcpy(&data_received.time,ptrdata,4);
		    	     memcpy(&data_received.q0,ptrdata+4,4);
		    	     memcpy(&data_received.q1,ptrdata+8,4);
		    	     memcpy(&data_received.q2,ptrdata+12,4);
		    	     memcpy(&data_received.q3,ptrdata+16,4);
		    	    //memcpy(&data_received.pitch,ptrdata+20,4);
		    	    // memcpy(&data_received.yaw,ptrdata+24,4);

		    	     for(i=0;i<20;i++) printf("%x", ptrdata[i]);
		    	 	 printf("\n");
			  		 /*
			  		 data_received.accy[i] = dataRec[i+8];
			  		 data_received.accz[i] = dataRec[i+12];
			  		 data_received.roll[i] = dataRec[i+16];
			  		 data_received.pitch[i] = dataRec[i+20];
			  		 data_received.yaw[i] = dataRec[i+24];
			  	  	  */
		    	 iNEMO_Led_Toggle(LED1);
		    	 /* Inizio stampa su VCOM per adeguarsi al programma MATLAB */
				 timer = timer + 11;
			     Stm32f1VCPrintf(" %.3f ", data_received.time);
			     Stm32f1VCPrintf(" %.3f ", data_received.q0);
			     Stm32f1VCPrintf(" %.3f ", data_received.q1);
			     Stm32f1VCPrintf(" %.3f ", data_received.q2);
			     //Stm32f1VCPrintf(" %.3f ", data_received.roll);
			     //Stm32f1VCPrintf(" %.3f ", data_received.pitch);
			     Stm32f1VCPrintf(" %.3f ; \n", data_received.q3);

			     //Stm32f1VCPrintf("\n");
				 //Stm32f1VCPrintf("Attitude(deg): R=%.3f, P=%.3f, Y=%.3f\n\n\r", xEulerAngles.m_fRoll* 180.0f / 3.141592f, xEulerAngles.m_fPitch * 180.0f / 3.141592f, xEulerAngles.m_fYaw * 180.0f / 3.141592f);
				 Stm32f1VCSendData();
		     }
		     else
		     {
			     Stm32f1VCPrintf("end");
			     Stm32f1VCPrintf("\n");
			     //Stm32f1VCPrintf("\n");
				 //Stm32f1VCPrintf("Attitude(deg): R=%.3f, P=%.3f, Y=%.3f\n\n\r", xEulerAngles.m_fRoll* 180.0f / 3.141592f, xEulerAngles.m_fPitch * 180.0f / 3.141592f, xEulerAngles.m_fYaw * 180.0f / 3.141592f);
				 Stm32f1VCSendData();
		     }

		     /* Toggle the iNemo led */
		     //delay_ms(10);


		 // }

	  }

}



// ----------------------------------------------------------------------------
void delay_ms(uint32_t ms) {
 	ms *= 75000000UL / 1000 / 6;

 	asm volatile(" mov r0, %[ms] \n\t"
 			"1: subs r0, #1 \n\t"
 			" bhi 1b \n\t"
 			:
 			: [ms] "r" (ms)
 			: "r0");
 }

