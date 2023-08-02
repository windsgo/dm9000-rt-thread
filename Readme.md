# dm9000a rt-thread Driver for stm32f103ZET6 

also have branch support stm32f407IGT6

## Pay Attention

1. This driver is written for `stm32f103ZET6-atk-warshipv3` board, 
on operating system `rt-thread`.

    The sdk-bsp of the board using rt-thread-stdio is on:
    [https://github.com/RT-Thread-Studio/sdk-bsp-stm32f103-atk-warshipv3](https://github.com/RT-Thread-Studio/sdk-bsp-stm32f103-atk-warshipv3)

    You can buy the board on:
    [https://item.taobao.com/item.htm?&id=20428448343](https://item.taobao.com/item.htm?&id=20428448343)

2. The isr handler need to be modified too, see details in `drv_gpio.c`

3. Note to change the address of the dm9000a when you are not using it on `stm32f103ZET6-atk-warshipv3` board

## Question & Problem

- Please issue on github, or contact my email: 18221102427@163.com