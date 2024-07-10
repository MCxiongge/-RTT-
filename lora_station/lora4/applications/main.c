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
#define LOG_TAG                        "subghz"
#include "ulog.h"
#include<stdio.h>
#include<string.h>
#include<stdlib.h>

#include "subghz_phy_app.h"
#include "radio.h"
#include "app_version.h"

#include "lora-radio-timer.h"

//触发中断条件启动线程
#define EV_RADIO_INIT            0x0001
#define EV_RADIO_TX_START        0x0002
#define EV_RADIO_TX_DONE         0x0004
#define EV_RADIO_TX_TIMEOUT      0x0008
#define EV_RADIO_RX_DONE         0x0010
#define EV_RADIO_RX_TIMEOUT      0x0020
#define EV_RADIO_RX_ERROR        0x0040
#define EV_RADIO_ALL             (EV_RADIO_INIT | EV_RADIO_TX_START | EV_RADIO_TX_DONE | EV_RADIO_TX_TIMEOUT | EV_RADIO_RX_DONE | EV_RADIO_RX_TIMEOUT | EV_RADIO_RX_ERROR)



/* USER CODE BEGIN PTD */
//时序机状态分配
typedef enum
{
  RX,
  RX_TIMEOUT,
  RX_ERROR,
  TX,
  TX_TIMEOUT,
IDE,
//  TXing,
//  HAND_OVER
} States_t;
/* USER CODE END PTD */




/* LM401_LoraWan Color led  */

#define LED_BLUE_PIN       GET_PIN(B,5)  /* defined the LED_BLUE pin: PB5 */
#define LED_GREEN_PIN      GET_PIN(B,4)
#define LED_RED_PIN        GET_PIN(B,3)
char buffer[256];

/* Configurations */
/*Timeout*/
#define RX_TIMEOUT_VALUE              5000
#define TX_TIMEOUT_VALUE              5000
/* PING string*/
#define PING "PING"
/* PONG string*/
#define PONG "PONG"
/*Size of the payload to be sent*/
/* Size must be greater of equal the PING and PONG*/
#define MAX_APP_BUFFER_SIZE          255
#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
/* wait for remote to be in Rx, before sending a Tx frame*/
#define RX_TIME_MARGIN                200
/* Afc bandwidth in Hz */
#define FSK_AFC_BANDWIDTH             83333
/* LED blink Period*/
#define LED_PERIOD_MS                50

/* Private variables ---------------------------------------------------------*/
/* Radio events function pointer */
static RadioEvents_t RadioEvents;
/* USER CODE BEGIN PV */
static struct rt_event radio_event;
/*Ping Pong FSM states */
static States_t State = RX;
/* App Rx Buffer*/
static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
/* App Tx Buffer*/
static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
/* Last  Received Buffer Size*/
uint16_t RxBufferSize = 0;
/* Last  Received packer Rssi*/
int8_t RssiValue = 0;
/* Last  Received packer SNR (in Lora modulation)*/
int8_t SnrValue = 0;
/* Led Timers objects*/
static rtick_timer_event_t timerLed;
/* device state. Master: true, Slave: false*/
bool isMaster = true;
/* random delay to make sure 2 devices will sync*/
/* the closest the random delays are, the longer it will
   take for the devices to sync when started simultaneously*/
static int32_t random_delay;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/*!
 * @brief Function to be executed on Radio Tx Done event
 */
static void OnTxDone(void);

/**
  * @brief Function to be executed on Radio Rx Done event
  * @param  payload ptr of buffer received
  * @param  size buffer size
  * @param  rssi
  * @param  LoraSnr_FskCfo
  */
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);

/**
  * @brief Function executed on Radio Tx Timeout event
  */
static void OnTxTimeout(void);

/**
  * @brief Function executed on Radio Rx Timeout event
  */
static void OnRxTimeout(void);

/**
  * @brief Function executed on Radio Rx Error event
  */
static void OnRxError(void);

/* USER CODE BEGIN PFP */
/**
  * @brief  Function executed on when led timer elapses
  * @param  context ptr of LED context
  */
static void OnledEvent(void);

/**
  * @brief PingPong state machine implementation
  */
static void lora_ping_pong_thread_entry(void* parameter);
/* USER CODE END PFP */
/*!
 * lora radio test thread
 */
static rt_thread_t lora_radio_test_thread = RT_NULL;
uint8_t led_red_s  = 1;
uint8_t led_green_s  = 1;
/* Exported functions ---------------------------------------------------------*/

int uart1init(void);
int test_len = 0;
rt_size_t flag = 0;
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
 int i;
 while (1)
 {
     rt_sem_take(&sem,RT_WAITING_FOREVER);
     test_len = rt_device_read(u1_dev,0,&buffer,rx_len);
     flag=1;
     for(i=0;i<test_len;i++)
     {
     rt_kprintf("%c",buffer[i]);
     }
 }

}
void read_data_entry(void *parameter)
{
    while(1)
    {
    }
}
//执行数据接收任务
static void lora_ping_pong_thread_entry(void* parameter)
{
    int test_size2;
    int i;
     LOG_D( "On lora_ping_pong_thread_entry\n\r");
     rt_uint32_t ev = 0;
     rt_uint8_t buffer_test[10]={1,1,1,2,2,2,3,3,3,4};//测试
    while( 1 )
    {
        if (rt_event_recv(&radio_event, EV_RADIO_ALL,
                                        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
                                        RT_WAITING_FOREVER, &ev) == RT_EOK)

        {
         Radio.Rx(0);//开启全接收模式
          switch (State)
          {
          case RX://接收数据返回主机
              if(RxBufferSize > 0){
                  rt_device_write(u1_dev, 0, &BufferRx,RxBufferSize);
                  LOG_D("on\r\n");
                  LOG_D("payload. size=%d \n\r", RxBufferSize);
                  LOG_HEX("RX:",16,BufferRx,test_len);
              }
              else {
                  LOG_D( "ID no\n\r");
            }
              State=IDE;//如果为接收到数据则进行监听
              //rt_event_send(&radio_event, EV_RADIO_ide_FLAG);
                break;
          case IDE:LOG_D( "IDE break\n\r");

          case TX:
          case RX_TIMEOUT:
          case RX_ERROR:
          case TX_TIMEOUT:
              //Radio.Sleep();
               LOG_D( "IDE\n\r");
               State=IDE;
               break;
            default://State=RX;
                LOG_D( "default\n\r");
              break;
          }

      }
  }
}

int main(void)
{
    uart1init();
    /* set LED_BLUE pin mode to output */
    rt_pin_mode(LED_BLUE_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_GREEN_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_RED_PIN, PIN_MODE_OUTPUT);
    /* USER CODE BEGIN SubghzApp_Init_1 */
     LOG_I( "PING PONG");
     //hw_rtc_init();
     /* Print APP version*/
     LOG_I("APP_VERSION= V%X.%X.%X",
             (uint8_t)(__APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
             (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
             (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB2_SHIFT));

     /* Led Timers*/
     rt_pin_mode(LED_GREEN_PIN, PIN_MODE_OUTPUT);
     rt_pin_mode(LED_RED_PIN, PIN_MODE_OUTPUT);

     rt_pin_write(LED_GREEN_PIN, PIN_LOW);
     rt_pin_write(LED_RED_PIN, PIN_HIGH);
     TimerInit(&timerLed, OnledEvent);
     TimerSetValue(&timerLed, LED_PERIOD_MS);
     TimerStart(&timerLed);
       //return 0;
     /* USER CODE END SubghzApp_Init_1 */
     rt_event_init(&radio_event, "ev_lora_test", RT_IPC_FLAG_FIFO);
     /* Radio initialization */
     //中断事件注册
     RadioEvents.TxDone = OnTxDone;
     RadioEvents.RxDone = OnRxDone;
     RadioEvents.TxTimeout = OnTxTimeout;
     RadioEvents.RxTimeout = OnRxTimeout;
     RadioEvents.RxError = OnRxError;

     Radio.Init(&RadioEvents);

     /* USER CODE BEGIN SubghzApp_Init_2 */
     /* Radio Set frequency */
     Radio.SetChannel(RF_FREQUENCY);

     /* Radio configuration */
     LOG_D("---------------");
     LOG_I("LORA_MODULATION");
     LOG_I("LORA_BW=%d kHz", (1 << LORA_BANDWIDTH) * 125);
     LOG_I("LORA_SF=%d", LORA_SPREADING_FACTOR);

     Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                       LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                       LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                       true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);

     Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                       LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                       LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                       0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

     Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);

     /* LED initialization*/


     /*calculate random delay for synchronization*/
     random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
     /*fills tx buffer*/
     rt_memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);

     LOG_I("rand=%d", random_delay);
     /*starts reception*/
     //Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
    Radio.Rx(0);
     /*register task to to be run in while(1) after Radio IT*/
     //UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);


//接收事件注册完成
    lora_radio_test_thread = rt_thread_create("lora1",
                                                            lora_ping_pong_thread_entry,
                                                            RT_NULL,
                                                            4096,
                                                            3,
                                                            100);
        if (lora_radio_test_thread != RT_NULL)
        {
            rt_thread_startup(lora_radio_test_thread);
        }
        else
            LOG_E("lora radio test thread create failed!\n");

        u1_th =rt_thread_create("u1_recv",serial_thread_entry, NULL, 1024, 4, 60);
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


static void OnTxDone(void)
{
  /* USER CODE BEGIN OnTxDone */
  LOG_D( "OnTxDone\n\r");
  /* Update the State of the FSM*/
  State = TX;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
   rt_event_send(&radio_event, EV_RADIO_TX_DONE);
  /* USER CODE END OnTxDone */
}
//接收完成中断
static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
{
  /* USER CODE BEGIN OnRxDone */
  LOG_D( "OnRxDone\n\r");
#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
  LOG_D("RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
  /* Record payload Signal to noise ratio in Lora*/
  SnrValue = LoraSnr_FskCfo;
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
  LOG_D("RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
  SnrValue = 0; /*not applicable in GFSK*/
#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
  /* Update the State of the FSM*/
  State = RX;
  /* Clear BufferRx*/
  rt_memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
  /* Record payload size*/
  RxBufferSize = size;
  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
  {
    rt_memcpy(BufferRx, payload, RxBufferSize);
  }
  /* Record Received Signal Strength*/
  RssiValue = rssi;
  /* Record payload content*/
  LOG_D("payload. size=%d \n\r", size);
  LOG_HEX("RX:",16,BufferRx,size);
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
  //执行接收完成返回事件任务
  rt_event_send(&radio_event, EV_RADIO_RX_DONE);
  /* USER CODE END OnRxDone */
}

static void OnTxTimeout(void)
{
  /* USER CODE BEGIN OnTxTimeout */
  LOG_D( "OnTxTimeout\n\r");
  /* Update the State of the FSM*/
  State = TX_TIMEOUT;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    rt_event_send(&radio_event, EV_RADIO_TX_TIMEOUT);
  /* USER CODE END OnTxTimeout */
}

static void OnRxTimeout(void)
{
  /* USER CODE BEGIN OnRxTimeout */
  LOG_D( "OnRxTimeout\n\r");
  /* Update the State of the FSM*/
  State = RX_TIMEOUT;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    rt_event_send(&radio_event, EV_RADIO_RX_TIMEOUT);
  /* USER CODE END OnRxTimeout */
}

static void OnRxError(void)
{
  /* USER CODE BEGIN OnRxError */
  LOG_D( "OnRxError\n\r");
  /* Update the State of the FSM*/
  State = RX_ERROR;
  /* Run PingPong process in background*/
  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
    rt_event_send(&radio_event, EV_RADIO_RX_ERROR);
  /* USER CODE END OnRxError */
}

static void OnledEvent(void)
{
    static uint32_t i = 0;
    TimerSetValue(&timerLed, LED_PERIOD_MS);
    TimerStart(&timerLed);
    i++;
    if(i%2 == 0)
    {
        rt_pin_write(LED_GREEN_PIN, PIN_LOW);
        rt_pin_write(LED_RED_PIN, PIN_LOW);
    }
    else
    {
        rt_pin_write(LED_GREEN_PIN, PIN_HIGH);
        rt_pin_write(LED_RED_PIN, PIN_HIGH);
    }


}

/* USER CODE END PrFD */
