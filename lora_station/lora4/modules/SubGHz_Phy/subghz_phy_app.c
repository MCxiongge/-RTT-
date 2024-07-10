//
//
///* Includes ------------------------------------------------------------------*/
//#include <rtthread.h>
//#include <rtdevice.h>
//#include <board.h>
//#define LOG_TAG                        "subghz"
//#include "ulog.h"
//
//#include "subghz_phy_app.h"
//#include "radio.h"
//#include "app_version.h"
//
///* USER CODE BEGIN Includes */
////#include "stm32_timer.h"
//#include "lora-radio-timer.h"
////#include "utilities_def.h"
///* USER CODE END Includes */
//
///* External variables ---------------------------------------------------------*/
///* USER CODE BEGIN EV */
//
///* USER CODE END EV */
//
///* Private typedef -----------------------------------------------------------*/
//// Ping pong event 调制状态
//#define EV_RADIO_INIT            0x0001
//#define EV_RADIO_TX_START        0x0002
//#define EV_RADIO_TX_DONE         0x0004
//#define EV_RADIO_TX_TIMEOUT      0x0008
//#define EV_RADIO_RX_DONE         0x0010
//#define EV_RADIO_RX_TIMEOUT      0x0020
//#define EV_RADIO_RX_ERROR        0x0040
//#define EV_RADIO_ALL             (EV_RADIO_INIT | EV_RADIO_TX_START | EV_RADIO_TX_DONE | EV_RADIO_TX_TIMEOUT | EV_RADIO_RX_DONE | EV_RADIO_RX_TIMEOUT | EV_RADIO_RX_ERROR)
//
///* USER CODE BEGIN PTD */
//typedef enum
//{
//  RX,
//  RX_TIMEOUT,
//  RX_ERROR,
//  TX,
//  TX_TIMEOUT,
//	IDE,
//	HAND_OVER
//} States_t;
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//#define LED_GREEN       GET_PIN(B,4)  /* defined the LED_BLUE pin: PB5 */
//#define LED_RED       GET_PIN(B,3)
////#define LED_GREEN       GET_PIN(B,5)  /* defined the LED_BLUE pin: PB5 */
//
///* Configurations */
///*Timeout*/
//#define RX_TIMEOUT_VALUE              5000
//#define TX_TIMEOUT_VALUE              5000
///* PING string*/
//#define PING "PING"
///* PONG string*/
//#define PONG "PONG"
///*Size of the payload to be sent*/
///* Size must be greater of equal the PING and PONG*/
//#define MAX_APP_BUFFER_SIZE          255
//#if (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE)
//#error PAYLOAD_LEN must be less or equal than MAX_APP_BUFFER_SIZE
//#endif /* (PAYLOAD_LEN > MAX_APP_BUFFER_SIZE) */
///* wait for remote to be in Rx, before sending a Tx frame*/
//#define RX_TIME_MARGIN                200
///* Afc bandwidth in Hz */
//#define FSK_AFC_BANDWIDTH             83333
///* LED blink Period*/
//#define LED_PERIOD_MS                 200
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
///* Radio events function pointer */
//static RadioEvents_t RadioEvents;
///* USER CODE BEGIN PV */
//static struct rt_event radio_event;
///*Ping Pong FSM states */
//static States_t State = RX;
///* App Rx Buffer*/
//static uint8_t BufferRx[MAX_APP_BUFFER_SIZE];
///* App Tx Buffer*/
//static uint8_t BufferTx[MAX_APP_BUFFER_SIZE];
///* Last  Received Buffer Size*/
//uint16_t RxBufferSize = 0;
///* Last  Received packer Rssi*/
//int8_t RssiValue = 0;
///* Last  Received packer SNR (in Lora modulation)*/
//int8_t SnrValue = 0;
///* Led Timers objects*/
//static rtick_timer_event_t timerLed;
///* device state. Master: true, Slave: false*/
//bool isMaster = true;
///* random delay to make sure 2 devices will sync*/
///* the closest the random delays are, the longer it will
//   take for the devices to sync when started simultaneously*/
//static int32_t random_delay;
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
///*!
// * @brief Function to be executed on Radio Tx Done event
// */
//static void OnTxDone(void);
//
///**
//  * @brief Function to be executed on Radio Rx Done event
//  * @param  payload ptr of buffer received
//  * @param  size buffer size
//  * @param  rssi
//  * @param  LoraSnr_FskCfo
//  */
//static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo);
//
///**
//  * @brief Function executed on Radio Tx Timeout event
//  */
//static void OnTxTimeout(void);
//
///**
//  * @brief Function executed on Radio Rx Timeout event
//  */
//static void OnRxTimeout(void);
//
///**
//  * @brief Function executed on Radio Rx Error event
//  */
//static void OnRxError(void);
//
///* USER CODE BEGIN PFP */
///**
//  * @brief  Function executed on when led timer elapses
//  * @param  context ptr of LED context
//  */
//static void OnledEvent(void);
//
///**
//  * @brief PingPong state machine implementation
//  */
//static void lora_ping_pong_thread_entry(void* parameter);
///* USER CODE END PFP */
///*!
// * lora radio test thread
// */
//static rt_thread_t lora_radio_test_thread = RT_NULL;
//uint8_t led_red_s  = 1;
//uint8_t led_green_s  = 1;
///* Exported functions ---------------------------------------------------------*/
//static int SubghzApp_Init(void)
//{
//  /* USER CODE BEGIN SubghzApp_Init_1 */
//  LOG_I( "PING PONG");
//  //hw_rtc_init();
//  /* Print APP version*/
//  LOG_I("APP_VERSION= V%X.%X.%X",
//          (uint8_t)(__APP_VERSION >> __APP_VERSION_MAIN_SHIFT),
//          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB1_SHIFT),
//          (uint8_t)(__APP_VERSION >> __APP_VERSION_SUB2_SHIFT));
//
//  /* Led Timers*/
//  rt_pin_mode(LED_GREEN, PIN_MODE_OUTPUT);
//  rt_pin_mode(LED_RED, PIN_MODE_OUTPUT);
//
//  rt_pin_write(LED_GREEN, PIN_LOW);
//  rt_pin_write(LED_RED, PIN_HIGH);
//  TimerInit(&timerLed, OnledEvent);
//  TimerSetValue(&timerLed, LED_PERIOD_MS);
//  TimerStart(&timerLed);
//    //return 0;
//  /* USER CODE END SubghzApp_Init_1 */
//  rt_event_init(&radio_event, "ev_lora_test", RT_IPC_FLAG_FIFO);
//  /* Radio initialization */
//  RadioEvents.TxDone = OnTxDone;
//  RadioEvents.RxDone = OnRxDone;
//  RadioEvents.TxTimeout = OnTxTimeout;
//  RadioEvents.RxTimeout = OnRxTimeout;
//  RadioEvents.RxError = OnRxError;
//
//  Radio.Init(&RadioEvents);
//
//  /* USER CODE BEGIN SubghzApp_Init_2 */
//  /* Radio Set frequency */
//  Radio.SetChannel(RF_FREQUENCY);
//
//  /* Radio configuration */
//  LOG_D("---------------");
//  LOG_I("LORA_MODULATION");
//  LOG_I("LORA_BW=%d kHz", (1 << LORA_BANDWIDTH) * 125);
//  LOG_I("LORA_SF=%d", LORA_SPREADING_FACTOR);
//
//  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
//                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
//                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
//                    true, 0, 0, LORA_IQ_INVERSION_ON, TX_TIMEOUT_VALUE);
//
//  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
//                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
//                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
//                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
//
//  Radio.SetMaxPayloadLength(MODEM_LORA, MAX_APP_BUFFER_SIZE);
//
//  /* LED initialization*/
//
//
//  /*calculate random delay for synchronization*/
//  random_delay = (Radio.Random()) >> 22; /*10bits random e.g. from 0 to 1023 ms*/
//  /*fills tx buffer*/
//  rt_memset(BufferTx, 0x0, MAX_APP_BUFFER_SIZE);
//
//  LOG_I("rand=%d", random_delay);
//  /*starts reception*/
//  Radio.Rx(RX_TIMEOUT_VALUE + random_delay);
//  /*register task to to be run in while(1) after Radio IT*/
//  //UTIL_SEQ_RegTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), UTIL_SEQ_RFU, PingPong_Process);
//  lora_radio_test_thread = rt_thread_create("lora-ping-pong",
//                                                        lora_ping_pong_thread_entry,
//                                                        RT_NULL,
//                                                        8096,
//                                                        2,
//                                                        10);
//    if (lora_radio_test_thread != RT_NULL)
//    {
//        rt_thread_startup(lora_radio_test_thread);
//    }
//    else
//        LOG_E("lora radio test thread create failed!\n");
//  /* USER CODE END SubghzApp_Init_2 */
//    return 0;
//}
//
///* USER CODE BEGIN EF */
////INIT_APP_EXPORT(SubghzApp_Init);
//MSH_CMD_EXPORT(SubghzApp_Init,SubghzApp Shell Cmd);
///* USER CODE END EF */
//
///* Private functions ---------------------------------------------------------*/
//
//static void OnTxDone(void)
//{
//  /* USER CODE BEGIN OnTxDone */
//  LOG_D( "OnTxDone\n\r");
//  /* Update the State of the FSM*/
//  State = TX;
//  /* Run PingPong process in background*/
//  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
//   rt_event_send(&radio_event, EV_RADIO_TX_DONE);
//  /* USER CODE END OnTxDone */
//}
//
//static void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t LoraSnr_FskCfo)
//{
//  /* USER CODE BEGIN OnRxDone */
//  LOG_D( "OnRxDone\n\r");
//#if ((USE_MODEM_LORA == 1) && (USE_MODEM_FSK == 0))
//  LOG_D("RssiValue=%d dBm, SnrValue=%ddB\n\r", rssi, LoraSnr_FskCfo);
//  /* Record payload Signal to noise ratio in Lora*/
//  SnrValue = LoraSnr_FskCfo;
//#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
//#if ((USE_MODEM_LORA == 0) && (USE_MODEM_FSK == 1))
//  LOG_D("RssiValue=%d dBm, Cfo=%dkHz\n\r", rssi, LoraSnr_FskCfo);
//  SnrValue = 0; /*not applicable in GFSK*/
//#endif /* USE_MODEM_LORA | USE_MODEM_FSK */
//  /* Update the State of the FSM*/
//  State = RX;
//  /* Clear BufferRx*/
//  rt_memset(BufferRx, 0, MAX_APP_BUFFER_SIZE);
//  /* Record payload size*/
//  RxBufferSize = size;
//  if (RxBufferSize <= MAX_APP_BUFFER_SIZE)
//  {
//    rt_memcpy(BufferRx, payload, RxBufferSize);
//  }
//  /* Record Received Signal Strength*/
//  RssiValue = rssi;
//  /* Record payload content*/
//  LOG_D("payload. size=%d \n\r", size);
//  LOG_HEX("RX:",16,BufferRx,size);
////  for (int i = 0; i < PAYLOAD_LEN; i++)
////  {
////    LOG_D("%02X", BufferRx[i]);
////  }
//  /* Run PingPong process in background*/
//  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
//  rt_event_send(&radio_event, EV_RADIO_RX_DONE);
//  /* USER CODE END OnRxDone */
//}
//
//static void OnTxTimeout(void)
//{
//  /* USER CODE BEGIN OnTxTimeout */
//  LOG_D( "OnTxTimeout\n\r");
//  /* Update the State of the FSM*/
//  State = TX_TIMEOUT;
//  /* Run PingPong process in background*/
//  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
//    rt_event_send(&radio_event, EV_RADIO_TX_TIMEOUT);
//  /* USER CODE END OnTxTimeout */
//}
//
//static void OnRxTimeout(void)
//{
//  /* USER CODE BEGIN OnRxTimeout */
//  LOG_D( "OnRxTimeout\n\r");
//  /* Update the State of the FSM*/
//  State = RX_TIMEOUT;
//  /* Run PingPong process in background*/
//  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
//    rt_event_send(&radio_event, EV_RADIO_RX_TIMEOUT);
//  /* USER CODE END OnRxTimeout */
//}
//
//static void OnRxError(void)
//{
//  /* USER CODE BEGIN OnRxError */
//  LOG_D( "OnRxError\n\r");
//  /* Update the State of the FSM*/
//  State = RX_ERROR;
//  /* Run PingPong process in background*/
//  //UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_SubGHz_Phy_App_Process), CFG_SEQ_Prio_0);
//    rt_event_send(&radio_event, EV_RADIO_RX_ERROR);
//  /* USER CODE END OnRxError */
//}
////初始化状态为RX
///* USER CODE BEGIN PrFD */
//static void lora_ping_pong_thread_entry(void* parameter)
//{
//  //Radio.Sleep();
//     LOG_D( "On lora_ping_pong_thread_entry\n\r");
//     rt_uint32_t ev = 0;
//
//    while( 1 )
//    {
//        if (rt_event_recv(&radio_event, EV_RADIO_ALL,
//                                        RT_EVENT_FLAG_OR | RT_EVENT_FLAG_CLEAR,
//                                        RT_WAITING_FOREVER, &ev) == RT_EOK)
//        {
//         //  Radio.Rx(1500);
//          switch (State)
//          {
//            case RX:/*解析数据*/
////                if(RxBufferSize > 0){
////                 rt_device_write(u1_dev, 0, &BufferRx, sizeof(RxBufferSize));
////                 State=RX;
////                }
////                else if{
////
////                }
//							      //必须先解析完成
//							      //数据不正常 //一直等待接收
//							    State=IDE;
//
//						  //数据正常并解析完成开启接收模式
//						State=HAND_OVER;//进行采集数据
//						Radio.Rx(RX_TIMEOUT_VALUE);
//              break;
//            case TX:
//              //进行数据的发送Radio.Send();
//						//发送失败进行数据的释放，使用rt_memset();
//
//              break;
//						case HAND_OVER:
//							//获取数据进行任务调度，以防止在接收到数据时，数据丢失。
//							 //获取传感器的数据并且封装成函数
//						   //使用Radio.Send();必须先计算出要发生数据的大小或者数据长度小于255，剩余长度补0
//						   //获取成功
//						   State=TX;
//						   //获取失败，使用rt_memset()清0;再次进入空闲状态等待数据的接收
//					      State=IDE;
//							break;
//            case RX_TIMEOUT:
//							//判断是否需要进行休眠
//
////						break;
//            case RX_ERROR:
//							//判断是否需要进行休眠
//            case TX_TIMEOUT:
//              Radio.Sleep();
//							State=IDE;
//							break;
////              break;
//						case IDE:
//							State=IDE;
//							break;
//            default:
//              break;
//          }
//      }
//  }
//}
//
//static void OnledEvent(void)
//{
//    static uint32_t i = 0;
//    i++;
//    if(i%2 == 0)
//    {
//        rt_pin_write(LED_GREEN, PIN_LOW);
//        rt_pin_write(LED_RED, PIN_LOW);
//    }
//    else
//    {
//        rt_pin_write(LED_GREEN, PIN_HIGH);
//        rt_pin_write(LED_RED, PIN_HIGH);
//    }
//   //LOG_D( "OnledEvent");
//  //BSP_LED_Toggle(LED_GREEN);
//    TimerSetValue(&timerLed, LED_PERIOD_MS);
//    TimerStart(&timerLed);
//}
//
///* USER CODE END PrFD */
//
///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
