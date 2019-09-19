void TIM16_IRQHandler(void) {
	if ((TIM16->SR & TIM_SR_UIF) != 0) {
		if (step_num > 0) {
			step_num--;
						
			DRV8884_STEP_HI;
			uint32_t wait = 50;
			while (wait--);
			DRV8884_STEP_LO;
			
		} else {
			DRV8884_disable();
		}
		
		TIM16->SR &= ~TIM_SR_UIF;
	}
}