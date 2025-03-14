uint16_t ADC_Read(uint32_t rank, uint32_t channel)
{
	/* Configure channel */
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = channel;
	sConfig.Rank = rank;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Convert the Channel */
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint16_t adcval = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adcval;
}
