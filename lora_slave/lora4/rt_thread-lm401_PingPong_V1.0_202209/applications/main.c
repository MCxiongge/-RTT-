/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-10     CaocoWang   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "ulog.h"
/* LM401_LoraWan Color led  */
#define LED_BLUE_PIN       GET_PIN(B,5)  /* defined the LED_BLUE pin: PB5 */
#define LED_GREEN_PIN      GET_PIN(B,4)
#define LED_RED_PIN        GET_PIN(B,3)

int uart1init(void);
rt_size_t len = 0;
rt_size_t rx_len = 0;
rt_device_t u1_dev;
rt_thread_t u1_th;
struct rt_semaphore sem;
struct serial_configure u1_configs = RT_SERIAL_CONFIG_9600 ;

rt_err_t rx_callback(rt_device_t dev, rt_size_t size)
{
    rx_len =size;
    rt_sem_release(&sem);
    return RT_EOK;
}
void serial_thread_entry(void *parameter)
{
  char buffer[1024];
 while (1)
 {
     rt_sem_take(&sem,RT_WAITING_FOREVER);
     rt_device_write(u1_dev, 0, buffer, sizeof(rx_len));
     rt_kprintf("%c",buffer);
 }

}
void read_data_entry(void *parameter)
{
    while(1)
    {
    }
}

int main(void)
{
    uart1init();
    /* set LED_BLUE pin mode to output */
    rt_pin_mode(LED_BLUE_PIN, PIN_MODE_OUTPUT);

    u1_th =rt_thread_create("u1_recv",serial_thread_entry, NULL, 4096, 10, 60);
           if (u1_th != RT_NULL)
                      {
                          rt_thread_startup(u1_th);
                      }
    while (1)
    {
        rt_pin_write(LED_BLUE_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_BLUE_PIN, PIN_LOW);
        rt_thread_mdelay(500);
        rt_device_write(u1_dev, 0,"123", sizeof("123"));
    }
}

int uart1init(void)
{
    rt_err_t ret = 0;
        u1_dev=rt_device_find("uart1");
        if (u1_dev == RT_NULL)
                {
                    LOG_E("rt_device_find[uart1] failed...\n");
                    return RT_ERROR;
                }
        ret=rt_device_open(u1_dev, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_DMA_RX);
        if(ret <0)
        {
            LOG_E("rt_device_open[uart1] failed...\n");
            return ret;
        }
        rt_device_control(u1_dev, RT_DEVICE_CTRL_CONFIG, (void *)&u1_configs);

        rt_device_set_rx_indicate(u1_dev,rx_callback);

        rt_sem_init(&sem,"rt_sem",0,RT_IPC_FLAG_FIFO);
        if(ret <0)
        {
            LOG_E("rt_sem_init failed[%d]...\n",ret);
            return ret;
        }
        return RT_EOK;
}
