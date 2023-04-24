#include "drv_dm9000.h"

#define DBG_TAG                        "dm9k"
#define DBG_LVL                        DBG_LOG
#include <rtdbg.h>

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#include <netif/ethernetif.h>

#include <stm32f1xx.h>
#include <stm32f1xx_hal_conf.h>

// #define DM9000_DEBUG
#ifdef DM9000_DEBUG
#define DM9000_TRACE    rt_kprintf
#else
#define DM9000_TRACE(...)
#endif

/* dm9000 reset pin : GPIOD PIN7, LOW is RESET */
#define DM9000_RST_0        rt_pin_write(GET_PIN(D, 7), PIN_LOW)
#define DM9000_RST_1        rt_pin_write(GET_PIN(D, 7), PIN_HIGH)

#define MAX_ADDR_LEN        6       /* max length of hw address */

#define DM9000_PHY          0x40    /* PHY address 0x01 */

enum DM9000_PHY_mode
{
    DM9000_10MHD = 0, DM9000_100MHD = 1,
    DM9000_10MFD = 4, DM9000_100MFD = 5,
    DM9000_AUTO  = 8, DM9000_1M_HPNA = 0x10
};

enum DM9000_TYPE
{
    TYPE_DM9000E,
    TYPE_DM9000A,
    TYPE_DM9000B
};

struct rt_dm9000_eth
{
    /* inherit from ethernet device */
    struct eth_device parent;

    enum DM9000_TYPE type;
    enum DM9000_PHY_mode mode;

    rt_uint8_t packet_cnt;                /* packet I or II */
    rt_uint16_t queue_packet_len;          /* queued packet (packet II) */

    /* interface address info. */
    rt_uint8_t  dev_addr[MAX_ADDR_LEN];     /* hw address   */
};

static struct rt_dm9000_eth dm9000_device;
static struct rt_semaphore sem_ack, sem_lock;

// 这个一定要放在全局作用域下
static SRAM_HandleTypeDef DM9000_Handler;           //DM9000句柄

/* --- */

static inline void dm9000_delay_ms(rt_uint32_t ms)
{
    rt_thread_mdelay(ms); return;
}

/* Read a byte from I/O port */
rt_inline rt_uint16_t dm9000_io_read(rt_uint16_t reg) {
    DM9000_IO = reg;
    return DM9000_DATA;
}

/* Write a byte to I/O port */
rt_inline void dm9000_io_write(rt_uint16_t reg, rt_uint16_t value) {
    DM9000_IO = reg;
    DM9000_DATA = value;
}

/* Get DeviceID of DM9000 */
static rt_uint32_t dm9000_get_device_id(void)
{
    rt_uint32_t value;
    value  = dm9000_io_read(DM9000_VIDL);
    value |= dm9000_io_read(DM9000_VIDH) << 8;
    value |= dm9000_io_read(DM9000_PIDL) << 16;
    value |= dm9000_io_read(DM9000_PIDH) << 24;
    return value;
}

/* Reset DM9000 */
static void dm9000_reset(void) {
    DM9000_TRACE("enter dm9000_reset\n");

    rt_pin_write(GET_PIN(D, 7), PIN_LOW); // hardware rst
    dm9000_delay_ms(10);

    rt_pin_write(GET_PIN(D, 7), PIN_HIGH);
    dm9000_delay_ms(100);  // hardware rst over

    dm9000_io_write(DM9000_GPCR, 0x01);
    dm9000_io_write(DM9000_GPR, 0);
    dm9000_io_write(DM9000_NCR, (0x02 | NCR_RST)); // soft rst

    do
    {
        dm9000_delay_ms(25);
    }while(dm9000_io_read(DM9000_NCR) & 1); // wait for soft rst over

    dm9000_io_write(DM9000_NCR,0);
    dm9000_io_write(DM9000_NCR, (0x02 | NCR_RST)); // soft rst again

    do
    {
        dm9000_delay_ms(25);
    }while (dm9000_io_read(DM9000_NCR) & 1);
}


/* Read a word from phyxcer */
rt_inline rt_uint16_t dm9000_phy_read(rt_uint16_t reg) {
    rt_uint16_t val;

    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);
    dm9000_io_write(DM9000_EPCR, 0x0C); /* Issue phyxcer read command */

    dm9000_delay_ms(100);       /* Wait read complete */

    dm9000_io_write(DM9000_EPCR, 0x00); /* Clear phyxcer read command */
    val = (dm9000_io_read(DM9000_EPDRH) << 8) | dm9000_io_read(DM9000_EPDRL);

    return val;
}

/* Write a word to phyxcer */
rt_inline void dm9000_phy_write(rt_uint16_t reg, rt_uint16_t value)
{
    /* Fill the phyxcer register into REG_0C */
    dm9000_io_write(DM9000_EPAR, DM9000_PHY | reg);

    /* Fill the written data into REG_0D & REG_0E */
    dm9000_io_write(DM9000_EPDRL, (value & 0xFF));
    dm9000_io_write(DM9000_EPDRH, ((value >> 8) & 0xFF));
    dm9000_io_write(DM9000_EPCR, 0x0A); /* Issue phyxcer write command */

    dm9000_delay_ms(500);       /* Wait write complete */

    dm9000_io_write(DM9000_EPCR, 0x00); /* Clear phyxcer write command */
}

/* Set PHY operationg mode */
rt_inline void dm9000_phy_mode_set(rt_uint8_t mode)
{
    rt_uint16_t phy_BMCR, phy_ANAR;
    switch(mode)
    {
        case DM9000_10MHD:
            phy_BMCR = 0X0000;
            phy_ANAR = 0X21;
            break;
        case DM9000_10MFD:
            phy_BMCR = 0X0100;
            phy_ANAR = 0X41;
            break;
        case DM9000_100MHD:
            phy_BMCR = 0X2000;
            phy_ANAR = 0X81;
            break;
        case DM9000_100MFD:
            phy_BMCR = 0X2100;
            phy_ANAR = 0X101;
            break;
        case DM9000_AUTO:
            phy_BMCR = 0X1000;
            phy_ANAR = 0X01E1;
            break;
    }

    dm9000_phy_write(DM9000_PHY_BMCR, phy_BMCR);
    dm9000_phy_write(DM9000_PHY_ANAR, phy_ANAR); /* Set PHY media mode */

    dm9000_io_write(DM9000_GPCR, 0x01); /* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0X00);  /* enable PHY */
}

/* interrupt service routine */
void rt_dm9000_isr(void)
{
    rt_uint16_t int_status;
    rt_uint16_t last_io;
    rt_uint32_t eint_pend;

    last_io = DM9000_IO;

    /* Disable all interrupts */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Got DM9000 interrupt status */
    int_status = dm9000_io_read(DM9000_ISR);    /* Got ISR */
    dm9000_io_write(DM9000_ISR, int_status);    /* Clear ISR status */

    DM9000_TRACE("dm9000 isr: int status %04x\n", int_status);

    /* receive overflow */
    if (int_status & ISR_ROS)
    {
        LOG_W("overflow, ISR:%02x", int_status);
    }

    if (int_status & ISR_ROOS)
    {
        LOG_W("overflow counter overflow, ISR:%02x", int_status);
    }

    /* Received the coming packet */
    if (int_status & ISR_PRS)
    {
        /* a frame has been received */
        eth_device_ready(&(dm9000_device.parent));
    }

    /* Transmit Interrupt check */
    if (int_status & ISR_PTS)
    {
        /* clear int_status */
        dm9000_io_write(DM9000_ISR, ISR_PTS);

        /* transmit done */
        int tx_status = dm9000_io_read(DM9000_NSR); /* Got TX status */

        if (tx_status & (NSR_TX2END | NSR_TX1END))
        {
            dm9000_device.packet_cnt --;
            if (dm9000_device.packet_cnt > 0)
            {
                DM9000_TRACE("dm9000 isr: tx second packet\n");

                /* transmit packet II */
                /* Set TX length to DM9000 */
                dm9000_io_write(DM9000_TXPLL, dm9000_device.queue_packet_len & 0xff);
                dm9000_io_write(DM9000_TXPLH, (dm9000_device.queue_packet_len >> 8) & 0xff);

                /* Issue TX polling command */
                dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */
            }

            /* One packet sent complete */
            /* clear tx isr */
            if (sem_ack.value != 0) {
                LOG_W("isr: trying to release sem_ack while its value > 0 / failed");
            } else {
                rt_sem_release(&sem_ack);
            }
        }
    }

    /* Re-enable interrupt mask */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    DM9000_IO = last_io;
}

static void dm9000_softrst_wait(rt_uint32_t ms)
{
    dm9000_io_write(DM9000_NCR, NCR_RST);
    do
    {
        rt_thread_mdelay(ms);
    } while (dm9000_io_read(DM9000_NCR) & 1); /* wait for soft rst over */

    /* initialize regs */

    /* GPIO0 on pre-activate PHY */
    dm9000_io_write(DM9000_GPR, 0x00);              /* REG_1F bit0 activate phyxcer */
    dm9000_io_write(DM9000_GPCR, GPCR_GEP_CNTL);    /* Let GPIO0 output */
    dm9000_io_write(DM9000_GPR, 0x00);               /* Enable PHY */

    /* Set PHY */
    dm9000_phy_mode_set(dm9000_device.mode);

    /* Program operating register */
    dm9000_io_write(DM9000_NCR, 0x0);   /* only intern phy supported by now */
    dm9000_io_write(DM9000_TCR, 0);     /* TX Polling clear */
    dm9000_io_write(DM9000_BPTR, 0x3f); /* Less 3Kb, 200us */
    dm9000_io_write(DM9000_FCTR, FCTR_HWOT(3) | FCTR_LWOT(8));  /* Flow Control : High/Low Water */
    dm9000_io_write(DM9000_FCR, 0x0);   /* SH FIXME: This looks strange! Flow Control */
    dm9000_io_write(DM9000_SMCR, 0);    /* Special Mode */
    dm9000_io_write(DM9000_NSR, NSR_WAKEST | NSR_TX2END | NSR_TX1END);  /* clear TX status */
    dm9000_io_write(DM9000_ISR, 0x0f);  /* Clear interrupt status */
    dm9000_io_write(DM9000_TCR2, 0x80); /* Switch LED to mode 1 */

    /* Activate DM9000 */
    dm9000_io_write(DM9000_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN); /* RX enable */
    dm9000_io_write(DM9000_IMR, IMR_PAR);
}

/* RT-Thread Device Interface */
/* initialize the interface */
static rt_err_t rt_dm9000_init(rt_device_t dev)
{
    LOG_I("Driver dm9000 init / start");
    int i, oft, lnk;
    rt_uint32_t dm9000_id;

    /* RESET device */
    dm9000_reset();
    dm9000_delay_ms(100);

    /* identfy DM9000 */
    dm9000_id = dm9000_get_device_id();
    LOG_I("dm9000 id: 0x%x", dm9000_id);
    if (dm9000_id != DM9000_ID) {
        LOG_E("dm9000 id error");
        return -RT_ERROR;
    }

    /* set mac address */
    for (i = 0, oft = DM9000_PAR; i < 6; ++i, ++oft)
        dm9000_io_write(oft, dm9000_device.dev_addr[i]);
    /* set multicast address */
    for (i = 0, oft = DM9000_MAR; i < 8; ++i, ++oft)
        dm9000_io_write(oft, 0xff);

    dm9000_softrst_wait(25); /* init regs here */

    if (dm9000_device.mode == DM9000_AUTO)
    {
        i = 0;
        while (!(dm9000_phy_read(1) & 0x20))
        {
            /* autonegation complete bit */
            rt_thread_delay( RT_TICK_PER_SECOND/10 );
            i++;
            if (i > 30 ) /* wait 3s */
            {
                LOG_E("could not establish link");
                return 0;
            }
        }
    }

    /* send a notify */
    eth_device_linkchange(&dm9000_device.parent, RT_TRUE);

    /* see what we've got */
    lnk = dm9000_phy_read(17) >> 12;
    rt_kprintf("operating at ");
    switch (lnk)
    {
    case 1:
        rt_kprintf("10M half duplex ");
        break;
    case 2:
        rt_kprintf("10M full duplex ");
        break;
    case 4:
        rt_kprintf("100M half duplex ");
        break;
    case 8:
        rt_kprintf("100M full duplex ");
        break;
    default:
        rt_kprintf("unknown: %d ", lnk);
        break;
    }
    rt_kprintf("mode\n");

    /* Enable TX/RX interrupt mask */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    LOG_I("Driver dm9000 init / end");
    return RT_EOK;
}

static rt_err_t rt_dm9000_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

static rt_err_t rt_dm9000_close(rt_device_t dev)
{
    /* RESET devie */
    dm9000_phy_write(0, 0x8000);    /* PHY RESET */
    dm9000_io_write(DM9000_GPR, 0x01);  /* Power-Down PHY */
    dm9000_io_write(DM9000_IMR, 0x80);  /* Disable all interrupt */
    dm9000_io_write(DM9000_RCR, 0x00);  /* Disable RX */

    return RT_EOK;
}

static rt_size_t rt_dm9000_read(rt_device_t dev, rt_off_t pos, void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_size_t rt_dm9000_write (rt_device_t dev, rt_off_t pos, const void* buffer, rt_size_t size)
{
    rt_set_errno(-RT_ENOSYS);
    return 0;
}

static rt_err_t rt_dm9000_control(rt_device_t dev, int cmd, void *args)
{
    switch (cmd)
    {
    case NIOCTL_GADDR:
        /* get mac address */
        if (args) rt_memcpy(args, dm9000_device.dev_addr, 6);
        else return -RT_ERROR;
        break;

    default :
        break;
    }

    return RT_EOK;
}

/* ethernet device interface */
/* transmit packet. */
rt_err_t rt_dm9000_tx( rt_device_t dev, struct pbuf* p)
{
//    LOG_D("enter rt_dm9000_tx, p->tot_len: %d\n", p->tot_len);
    DM9000_TRACE("rt_dm9000_tx: %d\n", p->tot_len);

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Move data to DM9000 TX RAM */
    DM9000_outb(DM9000_IO_BASE, DM9000_MWCMD);

    {
        /* q traverses through linked list of pbuf's
         * This list MUST consist of a single packet ONLY */
        struct pbuf *q;
        rt_uint16_t pbuf_index = 0;
        rt_uint8_t word[2], word_index = 0;

        q = p;
        /* Write data into dm9000a, two bytes at a time
         * Handling pbuf's with odd number of bytes correctly
         * No attempt to optimize for speed has been made */
        while (q)
        {
            if (pbuf_index < q->len)
            {
                word[word_index++] = ((u8_t*)q->payload)[pbuf_index++];
                if (word_index == 2)
                {
                    DM9000_outw(DM9000_DATA_BASE, (word[1] << 8) | word[0]);
                    word_index = 0;
                }
            }
            else
            {
                q = q->next;
                pbuf_index = 0;
            }
        }
        /* One byte could still be unsent */
        if (word_index == 1)
        {
            DM9000_outw(DM9000_DATA_BASE, word[0]);
        }
    }

//    /* Set TX length to DM9000 */
//    dm9000_io_write(DM9000_TXPLL, p->tot_len & 0xff);
//    dm9000_io_write(DM9000_TXPLH, (p->tot_len >> 8) & 0xff);
//
//    /* Issue TX polling command */
//    dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */

    if (dm9000_device.packet_cnt == 0)
    {
        DM9000_TRACE("dm9000 tx: first packet\n");

        dm9000_device.packet_cnt ++;
        /* Set TX length to DM9000 */
        dm9000_io_write(DM9000_TXPLL, p->tot_len & 0xff);
        dm9000_io_write(DM9000_TXPLH, (p->tot_len >> 8) & 0xff);

        /* Issue TX polling command */
        dm9000_io_write(DM9000_TCR, TCR_TXREQ); /* Cleared after TX complete */
    }
    else
    {
        DM9000_TRACE("dm9000 tx: second packet\n");

        dm9000_device.packet_cnt ++;
        dm9000_device.queue_packet_len = p->tot_len;
    }

    /* enable dm9000a all interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    /* wait ack */
    rt_sem_take(&sem_ack, RT_WAITING_FOREVER);

    DM9000_TRACE("rt_dm9000_tx done\n");

    return RT_EOK;
}

/* reception packet. */
struct pbuf *rt_dm9000_rx(rt_device_t dev)
{
    struct pbuf* p;
    rt_uint32_t rx_ready; /* first rx byte */
    rt_uint16_t rx_status, rx_len;
    rt_uint16_t* data;
    rt_uint8_t dummy_u8;
    rt_uint16_t dummy_u16; // used for dummy
    rt_int32_t len;

    /* init p pointer */
    p = RT_NULL;

    /* lock DM9000 device */
    rt_sem_take(&sem_lock, RT_WAITING_FOREVER);

    /* disable dm9000a interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR);

    /* Check packet ready or not */
    dm9000_io_read(DM9000_MRCMDX);              /* Dummy read */
    rx_ready = DM9000_inb(DM9000_DATA_BASE);      /* Got most updated data */
    if (rx_ready == 0x01)
    {
        /* A packet ready now  & Get status/length */
        DM9000_outb(DM9000_IO_BASE, DM9000_MRCMD);

        rx_status = DM9000_inw(DM9000_DATA_BASE) & 0xff00;
        rx_len = DM9000_inw(DM9000_DATA_BASE);

        DM9000_TRACE("dm9000 rx: status %04x len %d\n", rx_status, rx_len);

        /* error handle */
        if ((rx_status & 0xbf00) || (rx_len < 0x40) || (rx_len > DM9000_PKT_MAX))
        {
            LOG_E("rx error: status %04x, rx_len: %d", rx_status, rx_len);

            if (rx_status & 0x100)
            {
                LOG_E("rx fifo error");
            }
            if (rx_status & 0x200)
            {
                LOG_E("rx crc error");
            }
            if (rx_status & 0x8000)
            {
                LOG_E("rx length error");
            }
            if (rx_len > DM9000_PKT_MAX)
            {
                LOG_E("rx length too big");
            }

            /* software-reset and re-init */
            dm9000_softrst_wait(25);

            /* it issues an error, release pbuf */
            if (p != RT_NULL)
                pbuf_free(p);
            p = RT_NULL;
            goto _rx_end;
        }

        /* allocate buffer */
        p = pbuf_alloc(PBUF_LINK, rx_len, PBUF_RAM);
        if (p != RT_NULL)
        {
            RT_ASSERT(p->type == PBUF_RAM); /* set PBUF_RAM above */
            if (p->type == PBUF_RAM) {
                /* p is one large chunk */
                int i;

                RT_ASSERT(p->next == RT_NULL);
                RT_ASSERT(p->len == p->tot_len);

                data = (rt_uint16_t*)p->payload;
                len = p->len;

                while (len > 1) {
                    *data = DM9000_inw(DM9000_DATA_BASE);
                    data++;
                    len -= 2;
                }

                /* just read a byte, protect memory */
                if (len == 1) {
                    dummy_u8 = DM9000_inb(DM9000_DATA_BASE);
                    ((rt_uint8_t*)p->payload)[p->len - 1] = dummy_u8;
                }
            } else { /* p is not one large chunk */
                struct pbuf* q;
                rt_int32_t len;

                for (q = p; q != RT_NULL; q= q->next)
                {
                    data = (rt_uint16_t*)q->payload;
                    len = q->len;

                    while (len > 0)
                    {
                        *data = DM9000_inw(DM9000_DATA_BASE);
                        data ++;
                        len -= 2;
                    }
                }
            }
        }
        else /* pbuf allocate failed */
        {
            LOG_E("dm9000 rx: no pbuf, rx_len:%d", rx_len);
            len = rx_len;

            /* no pbuf, discard data from DM9000 */
            while (len > 1)
            {
                dummy_u16 = DM9000_inw(DM9000_DATA_BASE); /* dummy read 2 bytes */
                len -= 2;
            }

            /* len == 1, if remaining 1 byte not read */
            if (len == 1)
            {
                rt_uint8_t dummy_u8;

                dummy_u8 = DM9000_inb(DM9000_DATA_BASE); /* dummy read 1 byte */
            }
        }
    }
    else if (rx_ready > 0x01) /* error, stop interface and wait to reset */
    {
        LOG_E("dm9000 rx: rx error, stop device");

        dm9000_io_write(DM9000_ISR, 0x80);  /* Stop INT request */
        dm9000_io_write(DM9000_ISR, 0x0F);  /* Clear ISR status */
        dm9000_io_write(DM9000_RCR, 0x00);  /* Stop Rx Function */

        dm9000_softrst_wait(5); /* software-reset and re-init */
        goto _rx_end;
    }
    /*else rx_ready == 0x00, no message should be read */

_rx_end:

    /* clear packet received latch status */
    dm9000_io_write(DM9000_ISR, ISR_PRS);

    /* restore receive interrupt */
    dm9000_io_write(DM9000_IMR, IMR_PAR | IMR_PTM | IMR_PRM | ISR_ROS | ISR_ROOS);

    /* unlock DM9000 device */
    rt_sem_release(&sem_lock);

    return p;
}

int rt_hw_dm9000_init(void) {
    /* stm32 hal lib dm9000 specific init */
    rt_uint32_t temp;
    GPIO_InitTypeDef GPIO_Initure;
    FSMC_NORSRAM_TimingTypeDef FSMC_ReadWriteTim;

    __HAL_RCC_GPIOD_CLK_ENABLE();               //开启GPIOD时钟
    __HAL_RCC_GPIOE_CLK_ENABLE();               //开启GPIOE时钟
    __HAL_RCC_GPIOF_CLK_ENABLE();               //开启GPIOF时钟
    __HAL_RCC_GPIOG_CLK_ENABLE();               //开启GPIOG时钟

    GPIO_Initure.Pin=GPIO_PIN_7;                //PD7
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      //推挽输出
    GPIO_Initure.Pull=GPIO_PULLUP;              //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    GPIO_Initure.Pin=GPIO_PIN_6;                //PG6 DM9000中断引脚
    GPIO_Initure.Mode=GPIO_MODE_IT_FALLING;     //中断，下降沿
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);

    //PD0 1 4 5 8 8 9 10 14 15
    GPIO_Initure.Pin=GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|\
                     GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用推挽输出
    HAL_GPIO_Init(GPIOD,&GPIO_Initure);

    //PE7 8 9 10 11 12 13 14 15
    GPIO_Initure.Pin=GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|\
                     GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用推挽输出
    HAL_GPIO_Init(GPIOE,&GPIO_Initure);

    //PF13
    GPIO_Initure.Pin=GPIO_PIN_13;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用推挽输出
    HAL_GPIO_Init(GPIOF,&GPIO_Initure);

    //PG9
    GPIO_Initure.Pin=GPIO_PIN_9;
    GPIO_Initure.Mode=GPIO_MODE_AF_PP;          //复用推挽输出
    HAL_GPIO_Init(GPIOG,&GPIO_Initure);

    //中断线6=PG6
    HAL_NVIC_SetPriority(EXTI9_5_IRQn,1,0);     //抢占优先级为1，子优先级为0
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);           //使能中断线6

    DM9000_Handler.Instance=FSMC_NORSRAM_DEVICE;
    DM9000_Handler.Extended=FSMC_NORSRAM_EXTENDED_DEVICE;
    
    DM9000_Handler.Init.NSBank=FSMC_NORSRAM_BANK2;                      //使用NE2
    DM9000_Handler.Init.DataAddressMux=FSMC_DATA_ADDRESS_MUX_DISABLE;   //地址/数据线不复用
    DM9000_Handler.Init.MemoryType=FSMC_MEMORY_TYPE_SRAM;               //SRAM
    DM9000_Handler.Init.MemoryDataWidth=FSMC_NORSRAM_MEM_BUS_WIDTH_16;  //16位数据宽度
    DM9000_Handler.Init.BurstAccessMode=FSMC_BURST_ACCESS_MODE_DISABLE;     //是否使能突发访问,仅对同步突发存储器有效,此处未用到
    DM9000_Handler.Init.WaitSignalPolarity=FSMC_WAIT_SIGNAL_POLARITY_LOW;   //等待信号的极性,仅在突发模式访问下有用
    DM9000_Handler.Init.WaitSignalActive=FSMC_WAIT_TIMING_BEFORE_WS;    //存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT
    DM9000_Handler.Init.WriteOperation=FSMC_WRITE_OPERATION_ENABLE;     //存储器写使能
    DM9000_Handler.Init.WaitSignal=FSMC_WAIT_SIGNAL_DISABLE;            //等待使能位,此处未用到
    DM9000_Handler.Init.ExtendedMode=FSMC_EXTENDED_MODE_DISABLE;            //读写使用相同的时序
    DM9000_Handler.Init.AsynchronousWait=FSMC_ASYNCHRONOUS_WAIT_DISABLE;    //是否使能同步传输模式下的等待信号,此处未用到
    DM9000_Handler.Init.WriteBurst=FSMC_WRITE_BURST_DISABLE;            //禁止突发写
  
    //FMC读时序控制寄存器
    FSMC_ReadWriteTim.AddressSetupTime=0x00;        //地址建立时间（ADDSET）为1个HCLK 1/72M=13.8ns
    FSMC_ReadWriteTim.AddressHoldTime=0x00;         //地址保持时间（ADDHLD）模式A未用到
    FSMC_ReadWriteTim.DataSetupTime=0x03;           //数据保存时间为3个HCLK =4*13.8=55ns
    FSMC_ReadWriteTim.BusTurnAroundDuration=0X00;
    FSMC_ReadWriteTim.AccessMode=FSMC_ACCESS_MODE_A;//模式A
    
    HAL_SRAM_Init(&DM9000_Handler,&FSMC_ReadWriteTim,&FSMC_ReadWriteTim);

    /* general dm9000 init */
    rt_sem_init(&sem_ack, "tx_ack", 0, RT_IPC_FLAG_FIFO);   // 同步信号量，初始为0，发送tx后等待中断释放信号量表示tx完成
    rt_sem_init(&sem_lock, "eth_lock", 1, RT_IPC_FLAG_FIFO); // 互斥信号量，初始为1，用于保护tx和rx过程不冲突
    
    dm9000_device.type  = TYPE_DM9000A;
    dm9000_device.mode  = DM9000_AUTO;
    dm9000_device.packet_cnt = 0;
    dm9000_device.queue_packet_len = 0;

    /*
     * SRAM Tx/Rx pointer automatically return to start address,
     * Packet Transmitted, Packet Received
     */
    temp = *(volatile rt_uint16_t*)(0x1FFFF7E8);                //获取STM32的唯一ID的前24位作为MAC地址后三字节
    dm9000_device.dev_addr[0] = 0x02;
    dm9000_device.dev_addr[1] = 0x00;
    dm9000_device.dev_addr[2] = 0x00;
    dm9000_device.dev_addr[3] = (temp >> 16) & 0xFF;    //低三字节用STM32的唯一ID
    dm9000_device.dev_addr[4] = (temp >> 8) & 0xFFF;
    dm9000_device.dev_addr[5] = temp  &0xFF;

    dm9000_device.parent.parent.init       = rt_dm9000_init;
    dm9000_device.parent.parent.open       = rt_dm9000_open;
    dm9000_device.parent.parent.close      = rt_dm9000_close;
    dm9000_device.parent.parent.read       = rt_dm9000_read;
    dm9000_device.parent.parent.write      = rt_dm9000_write;
    dm9000_device.parent.parent.control    = rt_dm9000_control;
    dm9000_device.parent.parent.user_data  = RT_NULL;

    dm9000_device.parent.eth_rx  = rt_dm9000_rx;
    dm9000_device.parent.eth_tx  = rt_dm9000_tx;
    
    eth_device_init(&(dm9000_device.parent), "e0");
    eth_device_linkchange(&(dm9000_device.parent), RT_TRUE);


    return RT_EOK;
}

INIT_DEVICE_EXPORT(rt_hw_dm9000_init);

void dm9000a(void)
{
    rt_kprintf("\n");
    rt_kprintf("NCR   (%02X): %02x\n", DM9000_NCR,   dm9000_io_read(DM9000_NCR));
    rt_kprintf("NSR   (%02X): %02x\n", DM9000_NSR,   dm9000_io_read(DM9000_NSR));
    rt_kprintf("TCR   (%02X): %02x\n", DM9000_TCR,   dm9000_io_read(DM9000_TCR));
    rt_kprintf("TSRI  (%02X): %02x\n", DM9000_TSR1,  dm9000_io_read(DM9000_TSR1));
    rt_kprintf("TSRII (%02X): %02x\n", DM9000_TSR2,  dm9000_io_read(DM9000_TSR2));
    rt_kprintf("RCR   (%02X): %02x\n", DM9000_RCR,   dm9000_io_read(DM9000_RCR));
    rt_kprintf("RSR   (%02X): %02x\n", DM9000_RSR,   dm9000_io_read(DM9000_RSR));
    rt_kprintf("ORCR  (%02X): %02x\n", DM9000_ROCR,  dm9000_io_read(DM9000_ROCR));
    rt_kprintf("CRR   (%02X): %02x\n", DM9000_CHIPR, dm9000_io_read(DM9000_CHIPR));
    rt_kprintf("CSCR  (%02X): %02x\n", DM9000_CSCR,  dm9000_io_read(DM9000_CSCR));
    rt_kprintf("RCSSR (%02X): %02x\n", DM9000_RCSSR, dm9000_io_read(DM9000_RCSSR));
    rt_kprintf("ISR   (%02X): %02x\n", DM9000_ISR,   dm9000_io_read(DM9000_ISR));
    rt_kprintf("IMR   (%02X): %02x\n", DM9000_IMR,   dm9000_io_read(DM9000_IMR));
    rt_kprintf("pin_G_6: %d\n", rt_pin_read(GET_PIN(G, 6)));
    rt_kprintf("\n");
}

#ifdef RT_USING_FINSH
#include <finsh.h>
MSH_CMD_EXPORT(dm9000a, dm9000a register dump);
#endif
