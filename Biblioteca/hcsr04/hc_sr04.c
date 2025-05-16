/*!
 * @note	El TIM1 se considera como un timer de prestaciones avanzadas y para
 *			esta tarea esta "sobrecapacitado" su uso. Se prefiere utilizar timers
 *			de proposito general como TIM2, TIM3 o TIM4.
 */
#include "tim.h"
#include "hc_sr04.h"


/*!
 * @note	El siguiente codigo se sugiere a fin de
 *			1) reducir la latencia de la interrupción (tiempo que tarda el 
 *				Microcontrolador en atender la IRQ) sin usar HAL
 *			2) Mejorar su biblioteca con la adición de una bandera que especificia
 *				si el sensor no pudo completar una medicion.
 *			3) Automatizar el proceso de lectura del sensor tanto de Echo como de Trigger
 *			4) Realizar mediciones continuas a la velocidad máxima del sensor (40Hz/25ms).
 *
 * Esto asume que el TIM3 se configuro como:
 * 1. Periodo de desbordamiento: 40Hz (25ms)
 * 2. TIM3.Channel_1 como Output Compare (Trigger), con CCR1 establecido a los 10us
 * 3. TIM3.Channel_2 como Input Capture (Echo), rising edge
 *
 * De esta forma, las señales de Trigger y Echo son controladas y/o generadas
 * automaticamente cada 25ms:
 * #TIM3.Channel_1 (Trigger) es 1, cuando TIM3 < 10us, de otra forma es 0
 * #TIM3.Channel_2 (Echo) detecta y captura el valor del timer cuando ocurre
 *	una transición ascendente  y descendete en el registro TIM3->CCR2
 */

/* ucSensorFlags
 * 	Bit		B7	B6	B5	B4	B3	B2	B1	B0
 * 	Field	-	-	-	-	-	-	EF	DON
 *
 * 	EF: Error flag
 * 	DON: Measure is done
 */
volatile uint8_t ucSensorFlags = 0x00;
volatile uint32_t ulEchoRisingTime = 0;
volatile uint32_t ulEchoPulseWidth = 0;

void HCSR04_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); // Activa OC para generar Trigger automaticamente
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // Activa IC para capturar Echo automaticamente
}

/*
 * Se define esta función para avisar al while principal cuando:
 * 1) Hay una nueva medicion (1)
 * 2) No se completo la medición (-1)
 * 3) No hay medición nueva (0)
*/
int HCSR04_Available(void)
{
	int retVal = 0;
	if(ucSensorFlags & 0x01)
	{
		// Nueva medicion lista
		retVal = 1;

		// Clear flag
		__disable_irq();
		ucSensorFlags &= ~0x01;
		__enable_irq();
	}
	else if(ucSensorFlags & 0x02)
	{
		// Error en la medicion
		retVal = -1;

		// Clear flag
		__disable_irq();
		ucSensorFlags &= ~0x02;
		__enable_irq();
	}
	
	return retVal; // No hay medicion nueva
}

// Si hay una nueva medicion, convertirla a distancia en mm
uint16_t HCSR04_Get_Distance(void)
{
	uint32_t ulEchoPulseWidthCopy = 0;

	// Race condition
	__disable_irq();
	ulEchoPulseWidthCopy = ulEchoPulseWidth; // Sus unidades son microsegundos
	__enable_irq();


	// velocidad del sonido: 342.2 m/s = 342.2*100 cm/s = 342.2*100*10 mm/s
	// = 342200 mm/s = 342200 / 1000000 [mm/us] = 3422 / 10000 [mm/us]
	return (uint16_t)((ulEchoPulseWidthCopy * 3422UL) / 10000UL);
}

void TIM3_IRQHandler(void)
{
	/*
	 * A nivel interrupción, la función HAL_TIM_PeriodElapsedCallback
	 * se ejecuta dentro de esta función, comprobandose previamente la
	 * la bandera UIF, tal y como se muestra a continuación.
	 */
	/* Update flag == HAL_TIM_PeriodElapsedCallback */
	if(TIM3->SR & TIM_SR_UIF)
	{
		TIM3->SR &= ~TIM_SR_UIF;
		
		// Forza a que el flanco a detectar en IC sea ascendente
		TIM3->CCER &= ~TIM_CCER_CC1P;
		
		// Si se desborda el timer y el sensor no registro ninguna distancia
		// notificar al while principal con la bandera ucSensorFlags en bit 1
		if(!(ucSensorFlags & 0x01)) ucSensorFlags |= 0x02;
	}
	
	
	/*
	 * A nivel interrupción, la función HAL_TIM_IC_CaptureCallback
	 * se ejecuta dentro de esta función, comprobandose previamente la
	 * la bandera TIM_SR_CCxIF, tal y como se muestra a continuación.
	 */
	/* Input capture Interrupt == HAL_TIM_IC_CaptureCallback */
	if(TIM3->SR & TIM_SR_CC2IF)
	{
		TIM3->SR &= ~TIM_SR_CC2IF; // Limpiar bandera

		// Verificar cual flanco esta activo
		if((TIM3->CCER & TIM_CCER_CC2P) == 0) /* Flanco ascendente */
		{
			// Guarda cuenta de timer en el cambio de flanco
			ulEchoRisingTime = TIM3->CCR2;

			// Configurar flanco descendete
			TIM3->CCER |= TIM_CCER_CC2P;
		}
		else
		{
			// Calcular el ancho de pulso
			ulEchoPulseWidth = (TIM3->CCR2 - ulEchoRisingTime);

			// Bandera de nuevo dato
			ucSensorFlags |= 0x01;

			// Configurar flanco ascendente
			TIM3->CCER &= ~TIM_CCER_CC1P;
		}
	}
}
