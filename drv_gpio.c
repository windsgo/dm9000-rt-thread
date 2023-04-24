/* 在drv_gpio.c中修改原来的中断函数（以GPIOG_6引脚中断为例）如下 */
/* In drv_gpio.c Modify the original isr handled as follow */

void EXTI9_5_IRQHandler(void)
{
    extern void rt_dm9000_isr(void);
    rt_interrupt_enter();

    /* handle dm9000 interrupt */
    while (rt_pin_read(GET_PIN(G, 6)) == PIN_LOW)
    {
        rt_dm9000_isr();
    }
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_6);

    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    // HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    rt_interrupt_leave();
}