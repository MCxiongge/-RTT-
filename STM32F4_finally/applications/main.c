/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-05-15     RT-Thread    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include <msh.h>
#include "board.h"

#include <stdlib.h>
#include<posix/stdlib.h>
#include "stdio.h"
#include "paho_mqtt.h"
#include "serial.h"
#include "crc16.h"
#include "fal.h"
#include "easyflash.h"
int uart3init(void);
int uart4init(void);
#define LED_PIN_G GET_PIN(C,13)
//震动传感器 1为整数位，2为小数位其他传感器同理
uint16_t x_speed1;
uint16_t x_speed2;
uint16_t y_speed1;
uint16_t y_speed2;
uint16_t z_speed1;
uint16_t z_speed2;
uint16_t x_acceleration1;
uint16_t x_acceleration2;
uint16_t y_acceleration1;
uint16_t y_acceleration2;
uint16_t z_acceleration1;
uint16_t z_acceleration2;
uint16_t x_kurtosis1;
uint16_t x_kurtosis2;
uint16_t y_kurtosis1;
uint16_t y_kurtosis2;
uint16_t z_kurtosis1;
uint16_t z_kurtosis2;
uint16_t devicetemp1;
uint16_t devicetemp2;
//JTH—2D1温湿度变送器
uint16_t temp1;
uint16_t temp2;
uint16_t humi1;
uint16_t humi2;
//电流、电压传感器
uint16_t frequency1;
uint16_t frequency2;
uint16_t current_harmonics1;
uint16_t current_harmonics2;
uint16_t current1;
uint16_t current2;
uint16_t voltage1;
uint16_t voltage2;

rt_size_t len = 0;
rt_size_t len4 = 0;
rt_size_t rx_len = 0;
rt_size_t rx_len4 = 0;
rt_mailbox_t data;
char databuffer[256];
rt_device_t u3_dev;
rt_device_t u4_dev;
rt_thread_t u3_th;
rt_thread_t u4_th;
rt_thread_t modbus_sendcmd;
rt_thread_t screen_sendcmd;
rt_thread_t cloud_publish;
rt_thread_t data_handling;
rt_thread_t threshold_judgment;
struct rt_semaphore sem;
struct rt_semaphore sem1;
struct rt_semaphore sem2;
struct serial_configure u3_configs = RT_SERIAL_CONFIG_9600;
struct serial_configure u4_configs = RT_SERIAL_CONFIG_9600 ;
//阈值
uint8_t c_old_boot_times, c_new_boot_times[11] = {0};
uint32_t x_speed_threshold;
uint32_t y_speed_threshold;
uint32_t z_speed_threshold;
uint32_t x_acceleration_threshold;
uint32_t y_acceleration_threshold;
uint32_t z_acceleration_threshold;
uint32_t x_kurtosis_threshold;
uint32_t y_kurtosis_threshold;
uint32_t z_kurtosis_threshold;
uint32_t frequency_threshold;
uint32_t current_harmonics_threshold;
uint32_t current_threshold;
uint32_t voltage_threshold;
uint32_t devicetemp_threshold;

rt_err_t rx3_callback(rt_device_t u3_dev, rt_size_t size)//接收Lora主机传递过来的数据
{
    rx_len =size;
    rt_sem_release(&sem);
    return RT_EOK;
}

rt_err_t rx4_callback(rt_device_t u4_dev, rt_size_t size)//接收串口屏下发的信息
{
    rx_len4 =size;
    rt_sem_release(&sem1);
    return RT_EOK;
}

void cloud_publish_entry(void *parameter)//上报阿里云物模型
{
    char sendbuffer[1024]={0};
    while(1)
    {
    sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"37829753\",\"params\":{\"y_ac\":%d.%d,\"z_v\":%d.%d,\"y_v\":%d.%d,\"E_t\":%d.%d,\"y_k\":%d.%d,\"x_k\":%d.%d,\"z_k\":%d.%d,\"x_ac\":%d.%d,\"x_v\":%d.%d,\"z_ac\":%d.%d},\"version\":\"1.0.0\"}"
                                                                                                              ,y_acceleration1,y_acceleration2,z_speed1,z_speed2,y_speed1,y_speed2,devicetemp1,devicetemp2,y_kurtosis1,y_kurtosis2,x_kurtosis1,x_kurtosis2,z_kurtosis1,z_kurtosis2,x_acceleration1,x_acceleration2,x_speed1,x_speed2,z_acceleration1,z_acceleration1);
    msh_exec(sendbuffer, strlen(sendbuffer));
    memset(sendbuffer,0,sizeof(sendbuffer));

    rt_thread_mdelay(50);

    sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"382249982\",\"params\":{\"L_f\":%d.%d,\"Ch\":%d.%d,\"temperature\":%d.%d,\"E_c\":%d.%d,\"humidity\":%d.%d,\"E_v\":%d.%d},\"version\":\"1.0.0\"}"
                                                                                          ,frequency1,frequency2,current_harmonics1,current_harmonics2,temp1,temp2,current1,current2,humi1,humi2,voltage1,voltage2);
    msh_exec(sendbuffer, strlen(sendbuffer));
    memset(sendbuffer,0,sizeof(sendbuffer));
    rt_thread_mdelay(500);

    }
}

void serial3_thread_entry(void *parameter)//处理Lora主机传递过来的数据
{
    char buffer[1024];
  int High;
  int Low;
  int i=0;
  u16 crc16_rst;
 while (1)
 {
     rt_sem_take(&sem,RT_WAITING_FOREVER);
     len = rt_device_read(u3_dev,0,&buffer,rx_len);
     crc16_rst=crc_cal_value((unsigned char*)buffer,len-2);//crc校验，由于crclib中的查表法计算时容易爆栈，尝试多种方法都未能解决，故在crclib中增加了直接计算函数，规避了这一问题
     High=(crc16_rst>>8)&0xFF;
     Low =crc16_rst&0xFF;
        if(High==buffer[len-1] && Low==buffer[len-2])
        {
            switch(buffer[0])
            {
            case 0x01://传感器1：震动传感器
              i=i+1;
              x_speed1=((buffer[3]<< 8)|buffer[4])/100;
              x_speed2=((buffer[3]<< 8)|buffer[4])%100;
              rt_kprintf(" %d x_speed:%d.%d\r\n",i,x_speed1,x_speed2);
              y_speed1=((buffer[5]<< 8)|buffer[6])/100;
              y_speed2=((buffer[5]<< 8)|buffer[6])%100;
              rt_kprintf(" %d y_speed:%d.%d\r\n",i,y_speed1,y_speed2);
              z_speed1=((buffer[7]<< 8)|buffer[8])/100;
              z_speed2=((buffer[7]<< 8)|buffer[8])%100;
              rt_kprintf(" %d z_speed:%d.%d\r\n",i,z_speed1,z_speed2);
              devicetemp1=((buffer[9]<< 8)|buffer[10])/100;
              devicetemp2=((buffer[9]<< 8)|buffer[10])%100;
              rt_kprintf(" %d devicetemp:%d.%d\r\n",i,devicetemp1,devicetemp2);
              x_acceleration1=((buffer[11]<< 8)|buffer[12])/100;
              x_acceleration2=((buffer[11]<< 8)|buffer[12])%100;
              rt_kprintf(" %d x_acceleration:%d.%d\r\n",i,x_acceleration1,x_acceleration2);
              y_acceleration1=((buffer[13]<< 8)|buffer[14])/100;
              y_acceleration2=((buffer[13]<< 8)|buffer[14])%100;
              rt_kprintf(" %d y_acceleration:%d.%d\r\n",i,y_acceleration1,y_acceleration2);
              z_acceleration1=((buffer[15]<< 8)|buffer[16])/100;
              z_acceleration2=((buffer[15]<< 8)|buffer[16])%100;
              rt_kprintf(" %d z_acceleration:%d.%d\r\n",i,z_acceleration1,z_acceleration2);
              x_kurtosis1=((buffer[23]<< 8)|buffer[24])/100;
              x_kurtosis2=((buffer[23]<< 8)|buffer[24])%100;
              rt_kprintf(" %d x_kurtosis:%d.%d\r\n",i,x_kurtosis1,x_kurtosis2);
              y_kurtosis1=((buffer[31]<< 8)|buffer[32])/100;
              y_kurtosis2=((buffer[31]<< 8)|buffer[32])%100;
              rt_kprintf(" %d y_kurtosis:%d.%d\r\n",i,y_kurtosis1,y_kurtosis2);
              z_kurtosis1=((buffer[39]<< 8)|buffer[40])/100;
              z_kurtosis2=((buffer[39]<< 8)|buffer[40])%100;
              rt_kprintf(" %d z_kurtosis:%d.%d\r\n",i,z_kurtosis1,z_kurtosis2);
            case 0x02://传感器2：电流、电压传感器
                if(buffer[2]==0x02)
                {
              i=i+1;
              current_harmonics1=((buffer[3]<< 8)|buffer[4])/100;
              current_harmonics2=((buffer[3]<< 8)|buffer[4])%100;
              rt_kprintf(" %d current_harmonics:%d.%d\r\n",i,current_harmonics1,current_harmonics2);
                }
                if(buffer[2]==0x18)
                {
              i=i+1;
              voltage1=((buffer[3]<< 8)|buffer[4])/100;
              voltage2=((buffer[3]<< 8)|buffer[4])%100;
              rt_kprintf(" %d voltage:%d.%d\r\n",i,voltage1,voltage2);
              current1=((buffer[7]<< 8)|buffer[8])/1000;
              current2=((buffer[7]<< 8)|buffer[8])%1000;
              rt_kprintf(" %d current:%d.%d\r\n",i,current1,current2);
              frequency1=((buffer[25]<< 8)|buffer[26])/100;
              frequency2=((buffer[25]<< 8)|buffer[26])%100;
              rt_kprintf(" %d frequency:%d.%d\r\n",i,frequency1,frequency2);
                }
              break;
            case 0x03://传感器3：温湿度传感器
              i=i+1;
              temp1=((buffer[3]<< 8)|buffer[4])/10;
              temp2=((buffer[3]<< 8)|buffer[4])%10;
              rt_kprintf(" %d temp:%d.%d\r\n",i,temp1,temp2);
              humi1=((buffer[5]<< 8)|buffer[6])/10;
              humi2=((buffer[5]<< 8)|buffer[6])%10;
              rt_kprintf(" %d humi:%d.%d\r\n",i,humi1,humi1);
              break;
            default:
              break;
            }
        rt_sem_release(&sem2);//释放信号量给阈值判断线程
        }
        rt_thread_mdelay(100);
 }

}
void serial4_thread_entry(void *parameter)
{
    char buffer[100];
    char *buffer2;
    char bufferA[4]= {0};
    char bufferB[4]= {0};
    char bufferC[4]= {0};
    char bufferD[4]= {0};
    char bufferE[4]= {0};
    char bufferF[4]= {0};
    char bufferG[4]= {0};
    char bufferH[4]= {0};
    char bufferI[4]= {0};
    char bufferJ[4]= {0};
    char bufferK[4]= {0};
    char bufferL[4]= {0};
    char bufferM[4]= {0};
    char bufferN[4]= {0};
    int i;
    while(1)
    {
    rt_sem_take(&sem1,RT_WAITING_FOREVER);
    len4 = rt_device_read(u4_dev,0,&buffer,rx_len4);
    //串口屏将以A0123B3210B001.....的字母+固定四位数字的形式下发阈值
    for(i=0;i<sizeof(buffer);i++)
    {
     switch(buffer[i])
     {
     case 'A': x_speed_threshold=strtol(&buffer[i+1], &buffer2,10); /*将B前的四位数字提取出来*/ itoa(x_speed_threshold,bufferA,10);/*转换回字符串*/ ef_set_env("x_speed_threshold_env", bufferA); ef_save_env();/*设置并保存阈值*/   rt_kprintf("uart4get A:%d \r\n",x_speed_threshold); break;/*调试串口回显*/
     case 'B': y_speed_threshold=strtol(&buffer[i+1], &buffer2,10);        itoa(y_speed_threshold,bufferB,10);           ef_set_env("y_speed_threshold_env", bufferB); ef_save_env();  rt_kprintf("uart4get B:%d \r\n",y_speed_threshold); break;
     case 'C': z_speed_threshold=strtol(&buffer[i+1], &buffer2,10);        itoa(z_speed_threshold,bufferC,10);           ef_set_env("z_speed_threshold_env", bufferC); ef_save_env();  rt_kprintf("uart4get C:%d \r\n",z_speed_threshold); break;
     case 'D': x_acceleration_threshold=strtol(&buffer[i+1], &buffer2,10); itoa(x_acceleration_threshold,bufferD,10);    ef_set_env("x_acceleration_threshold_env", bufferD); ef_save_env();  rt_kprintf("uart4get D:%d \r\n",x_acceleration_threshold); break;
     case 'E': y_acceleration_threshold=strtol(&buffer[i+1], &buffer2,10); itoa(y_acceleration_threshold,bufferE,10);    ef_set_env("y_acceleration_threshold_env", bufferE); ef_save_env();  rt_kprintf("uart4get E:%d \r\n",y_acceleration_threshold); break;
     case 'F': z_acceleration_threshold=strtol(&buffer[i+1], &buffer2,10); itoa(z_acceleration_threshold,bufferF,10);    ef_set_env("z_acceleration_threshold_env", bufferF); ef_save_env();  rt_kprintf("uart4get F:%d \r\n",z_acceleration_threshold);break;
     case 'G': x_kurtosis_threshold=strtol(&buffer[i+1], &buffer2,10);     itoa(x_kurtosis_threshold,bufferG,10);        ef_set_env("x_kurtosis_threshold_env", bufferG); ef_save_env();  rt_kprintf("uart4get G:%d \r\n",x_kurtosis_threshold); break;
     case 'H': y_kurtosis_threshold=strtol(&buffer[i+1], &buffer2,10);     itoa(y_kurtosis_threshold,bufferH,10);        ef_set_env("y_kurtosis_threshold_env", bufferH); ef_save_env();  rt_kprintf("uart4get H:%d \r\n",y_kurtosis_threshold); break;
     case 'I': z_kurtosis_threshold=strtol(&buffer[i+1], &buffer2,10);     itoa(z_kurtosis_threshold,bufferI,10);        ef_set_env("z_kurtosis_threshold_env", bufferI); ef_save_env();  rt_kprintf("uart4get I:%d \r\n",z_kurtosis_threshold); break;
     case 'J': devicetemp_threshold=strtol(&buffer[i+1], &buffer2,10);     itoa(devicetemp_threshold,bufferJ,10);        ef_set_env("devicetemp_threshold_env", bufferJ); ef_save_env();  rt_kprintf("uart4get J:%d \r\n",devicetemp_threshold); break;
     case 'K': frequency_threshold=strtol(&buffer[i+1], &buffer2,10);      itoa(frequency_threshold,bufferK,10);         ef_set_env("frequency_threshold_env", bufferK); ef_save_env();   rt_kprintf("uart4get K:%d \r\n",frequency_threshold); break;
     case 'L': voltage_threshold=strtol(&buffer[i+1], &buffer2,10);      itoa(voltage_threshold,bufferL,10);         ef_set_env("voltage_threshold", bufferL); ef_save_env();   rt_kprintf("uart4get L:%d \r\n",voltage_threshold); break;
     case 'M': current_threshold=strtol(&buffer[i+1], &buffer2,10);      itoa(current_threshold,bufferM,10);         ef_set_env("current_threshold", bufferM); ef_save_env();   rt_kprintf("uart4get M:%d \r\n",current_threshold); break;
     case 'N': current_harmonics_threshold=strtol(&buffer[i+1], &buffer2,10);      itoa(current_harmonics_threshold,bufferN,10);         ef_set_env("current_harmonics_threshold", bufferN); ef_save_env();   rt_kprintf("uart4get N:%d \r\n",current_harmonics_threshold); break;
     default: break;
     }

    }
    memset(buffer,0,sizeof(buffer));
    rt_thread_mdelay(100);
    }
}
void send_data_entry(void *parameter)
{
      char screencmd[1024];
      while (1)
      {
        //记录展示
        snprintf(screencmd,sizeof(screencmd),"t1.txt=\"%d.%d\"\xff\xff\xfft2.txt=\"%d.%d\"\xff\xff\xfft3.txt=\"%d.%d\"\xff\xff\xfft4.txt=\"%d.%d\"\xff\xff\xfft5.txt=\"%d.%d\"\xff\xff\xfft6.txt=\"%d.%d\"\xff\xff\xff"
        ,x_speed1,x_speed2,y_speed1,y_speed2,z_speed1,z_speed2,x_acceleration1,x_acceleration2,y_acceleration1,y_acceleration2,z_acceleration1,z_acceleration2);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"t7.txt=\"%d.%d\"\xff\xff\xfft8.txt=\"%d.%d\"\xff\xff\xfft9.txt=\"%d.%d\"\xff\xff\xfft10.txt=\"%d.%d\"\xff\xff\xfft11.txt=\"%d.%d\"\xff\xff\xfft12.txt=\"%d.%d\"\xff\xff\xfft13.txt=\"%d.%d\"\xff\xff\xfft14.txt=\"%d.%d\"\xff\xff\xfft15.txt=\"%d\"\xff\xff\xfft16.txt=\"%d\"\xff\xff\xff"
        ,x_kurtosis1,x_kurtosis2,y_kurtosis1,y_kurtosis2,z_kurtosis1,z_kurtosis2,devicetemp1,devicetemp2,frequency1,frequency2,voltage1,voltage2,current1,current2,current_harmonics1,current_harmonics2,temp1,humi1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
      //曲线展示
        memset(screencmd,0,sizeof(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s0.id,0,%d\xff\xff\xff",x_speed1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s0.id,1,%d\xff\xff\xff",y_speed1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s0.id,2,%d\xff\xff\xff",z_speed1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s1.id,0,%d\xff\xff\xff",x_acceleration1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s1.id,1,%d\xff\xff\xff",y_acceleration1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s1.id,2,%d\xff\xff\xff",z_acceleration1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s2.id,0,%d\xff\xff\xff",x_kurtosis1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s2.id,1,%d\xff\xff\xff",y_kurtosis1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s2.id,2,%d\xff\xff\xff",z_kurtosis1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s4.id,0,%d\xff\xff\xff",frequency1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        snprintf(screencmd,sizeof(screencmd),"add s4.id,1,%d\xff\xff\xff",current_harmonics1);
        rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
        memset(screencmd,0,sizeof(screencmd));
      rt_thread_mdelay(300);
      }
}
void threshold_judgment_entry(void *parameter)
{
    char *A,*B,*C,*D,*E,*F,*G,*H,*I,*J,*K,*L,*M,*N;
    //获取储存在内部flash中的阈值
     A = ef_get_env("x_speed_threshold_env");
     x_speed_threshold = atol(A);
     B = ef_get_env("y_speed_threshold_env");
     y_speed_threshold = atol(B);
     C = ef_get_env("z_speed_threshold_env");
     z_speed_threshold = atol(C);
     D = ef_get_env("x_acceleration_threshold_env");
     x_acceleration_threshold = atol(D);
     E = ef_get_env("y_acceleration_threshold_env");
     y_acceleration_threshold = atol(E);
     F = ef_get_env("z_acceleration_threshold_env");
     z_acceleration_threshold = atol(F);
     G = ef_get_env("x_kurtosis_threshold_env");
     x_kurtosis_threshold = atol(G);
     H = ef_get_env("y_kurtosis_threshold_env");
     y_kurtosis_threshold = atol(H);
     I = ef_get_env("z_kurtosis_threshold_env");
     z_kurtosis_threshold = atol(I);
     J = ef_get_env("devicetemp_threshold_env");
     devicetemp_threshold = atol(J);
     K = ef_get_env("frequency_threshold_env");
     frequency_threshold = atol(K);
     L = ef_get_env("voltage_threshold");
     voltage_threshold = atol(L);
     M = ef_get_env("current_threshold");
     current_threshold = atol(M);
     N = ef_get_env("current_harmonics_threshold");
     current_harmonics_threshold = atol(N);
    int a=0,b=0,c=0,d=0,e=0,f=0,g=0,h=0,i=0,j=0,k=0,l=0,m=0,n=0;
    char screencmd[1024];
    char sendbuffer[1024];
    int x=0;
    //开机后在显示屏上显示储存的阈值
    snprintf(screencmd,sizeof(screencmd),"warn.tA.txt=\"%ld\"\xff\xff\xffwarn.tB.txt=\"%ld\"\xff\xff\xffwarn.tC.txt=\"%ld\"\xff\xff\xffwarn.tD.txt=\"%ld\"\xff\xff\xffwarn.tE.txt=\"%ld\"\xff\xff\xffwarn.tF.txt=\"%ld\"\xff\xff\xffwarn.tG.txt=\"%ld\"\xff\xff\xffwarn.tH.txt=\"%ld\"\xff\xff\xffwarn.tI.txt=\"%ld\"\xff\xff\xffwarn.tJ.txt=\"%ld\"\xff\xff\xffwarn.tK.txt=\"%ld\"\xff\xff\xffwarn.tL.txt=\"%ld\"\xff\xff\xffwarn.tM.txt=\"%ld\"\xff\xff\xffwarn.tN.txt=\"%ld\"\xff\xff\xff"
    ,x_speed_threshold,y_speed_threshold,z_speed_threshold,x_acceleration_threshold,y_acceleration_threshold,z_acceleration_threshold,x_kurtosis_threshold,y_kurtosis_threshold,z_kurtosis_threshold,devicetemp_threshold,frequency_threshold,voltage_threshold,current_threshold,current_harmonics_threshold);
    rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
    memset(screencmd,0,sizeof(screencmd));
    snprintf(screencmd,sizeof(screencmd),"warn.tA.txt=\"%ld\"\xff\xff\xffwarn.tB.txt=\"%ld\"\xff\xff\xffwarn.tC.txt=\"%ld\"\xff\xff\xffwarn.tD.txt=\"%ld\"\xff\xff\xffwarn.tE.txt=\"%ld\"\xff\xff\xffwarn.tF.txt=\"%ld\"\xff\xff\xffwarn.tG.txt=\"%ld\"\xff\xff\xffwarn.tH.txt=\"%ld\"\xff\xff\xffwarn.tI.txt=\"%ld\"\xff\xff\xffwarn.tJ.txt=\"%ld\"\xff\xff\xffwarn.tK.txt=\"%ld\"\xff\xff\xffwarn.tL.txt=\"%ld\"\xff\xff\xffwarn.tM.txt=\"%ld\"\xff\xff\xffwarn.tN.txt=\"%ld\"\xff\xff\xff"
    ,x_speed_threshold,y_speed_threshold,z_speed_threshold,x_acceleration_threshold,y_acceleration_threshold,z_acceleration_threshold,x_kurtosis_threshold,y_kurtosis_threshold,z_kurtosis_threshold,devicetemp_threshold,frequency_threshold,voltage_threshold,current_threshold,current_harmonics_threshold);
    rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
    memset(screencmd,0,sizeof(screencmd));
    snprintf(screencmd,sizeof(screencmd),"warn.tA.txt=\"%ld\"\xff\xff\xffwarn.tB.txt=\"%ld\"\xff\xff\xffwarn.tC.txt=\"%ld\"\xff\xff\xffwarn.tD.txt=\"%ld\"\xff\xff\xffwarn.tE.txt=\"%ld\"\xff\xff\xffwarn.tF.txt=\"%ld\"\xff\xff\xffwarn.tG.txt=\"%ld\"\xff\xff\xffwarn.tH.txt=\"%ld\"\xff\xff\xffwarn.tI.txt=\"%ld\"\xff\xff\xffwarn.tJ.txt=\"%ld\"\xff\xff\xffwarn.tK.txt=\"%ld\"\xff\xff\xffwarn.tL.txt=\"%ld\"\xff\xff\xffwarn.tM.txt=\"%ld\"\xff\xff\xffwarn.tN.txt=\"%ld\"\xff\xff\xff"
    ,x_speed_threshold,y_speed_threshold,z_speed_threshold,x_acceleration_threshold,y_acceleration_threshold,z_acceleration_threshold,x_kurtosis_threshold,y_kurtosis_threshold,z_kurtosis_threshold,devicetemp_threshold,frequency_threshold,voltage_threshold,current_threshold,current_harmonics_threshold);
    rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
//    memset(screencmd,0,sizeof(screencmd));
    sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"1698265518\",\"params\":{\"ts_v\":\"normal\",\"ts_ac\":\"normal\",\"ts_k\":\"normal\",\"ts_ch\":\"normal\"},\"version\":\"1.0.0\"}");
    msh_exec(sendbuffer, strlen(sendbuffer));
    memset(sendbuffer,0,sizeof(sendbuffer));
    //开机后调试串口显示阈值
     rt_kprintf("x_speed_threshold_env:%d \r\n",x_speed_threshold);
     rt_kprintf("y_speed_threshold_env:%d \r\n",y_speed_threshold);
     rt_kprintf("z_speed_threshold_env:%d \r\n",z_speed_threshold);
     rt_kprintf("x_acceleration_threshold_env:%d \r\n",x_acceleration_threshold);
     rt_kprintf("y_acceleration_threshold_env:%d \r\n",y_acceleration_threshold);
     rt_kprintf("z_acceleration_threshold_env:%d \r\n",z_acceleration_threshold);
     rt_kprintf("x_kurtosis_threshold_env:%d \r\n",x_kurtosis_threshold);
     rt_kprintf("y_kurtosis_threshold_env:%d \r\n",y_kurtosis_threshold);
     rt_kprintf("z_kurtosis_threshold_env:%d \r\n",z_kurtosis_threshold);
     rt_kprintf("devicetemp_threshold_env:%d \r\n",devicetemp_threshold);
     rt_kprintf("frequency_threshold_env:%d \r\n",frequency_threshold);
     rt_kprintf("voltage_threshold:%d \r\n",voltage_threshold);
     rt_kprintf("current_threshold:%d \r\n",current_threshold);
     rt_kprintf("current_harmonics_threshold:%d \r\n",current_harmonics_threshold);
     while(1)
     {
         rt_sem_take(&sem2,RT_WAITING_FOREVER);//信号量到来后开始阈值判断
         if(x_speed1>=x_speed_threshold)
         {
          a=a+1;//报警次数累加
          snprintf(screencmd,sizeof(screencmd),"ta.txt=\"%d\"\xff\xff\xff",a);
          rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(y_speed1>=y_speed_threshold)
         {
             b=b+1;
             snprintf(screencmd,sizeof(screencmd),"tb.txt=\"%d\"\xff\xff\xff",b);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(z_speed1>=z_speed_threshold)
         {
             c=c+1;
             snprintf(screencmd,sizeof(screencmd),"tc.txt=\"%d\"\xff\xff\xff",c);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(x_acceleration1>=x_acceleration_threshold)
         {
             d=d+1;
             snprintf(screencmd,sizeof(screencmd),"td.txt=\"%d\"\xff\xff\xff",d);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(y_acceleration1>=y_acceleration_threshold)
         {
             e=e+1;
             snprintf(screencmd,sizeof(screencmd),"te.txt=\"%d\"\xff\xff\xff",e);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(z_acceleration1>=z_acceleration_threshold)
         {
             f=f+1;
             snprintf(screencmd,sizeof(screencmd),"tf.txt=\"%d\"\xff\xff\xff",f);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(x_kurtosis1>=x_kurtosis_threshold)
         {
             g=g+1;
             snprintf(screencmd,sizeof(screencmd),"tg.txt=\"%d\"\xff\xff\xff",g);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(y_kurtosis1>=y_kurtosis_threshold)
         {
             h=h+1;
             snprintf(screencmd,sizeof(screencmd),"th.txt=\"%d\"\xff\xff\xff",h);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(z_kurtosis1>=z_kurtosis_threshold)
         {
             i=i+1;
             snprintf(screencmd,sizeof(screencmd),"ti.txt=\"%d\"\xff\xff\xff",i);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(devicetemp1>=devicetemp_threshold)
         {
             j=j+1;
             snprintf(screencmd,sizeof(screencmd),"tj.txt=\"%d\"\xff\xff\xff",j);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备温度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备温度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         }
         if(frequency1>=frequency_threshold)
         {
             k=k+1;
             snprintf(screencmd,sizeof(screencmd),"tk.txt=\"%d\"\xff\xff\xff",k);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备频率异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备频率异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         }
         if(voltage1>=voltage_threshold)
         {
             l=l+1;
             snprintf(screencmd,sizeof(screencmd),"tl.txt=\"%d\"\xff\xff\xff",l);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"交流电压异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"交流电压异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         }
         if(current1>=current_threshold)
         {
             m=m+1;
             snprintf(screencmd,sizeof(screencmd),"tm.txt=\"%d\"\xff\xff\xff",m);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }
         if(current_harmonics1>=current_harmonics_threshold)
         {
             n=n+1;
             snprintf(screencmd,sizeof(screencmd),"tn.txt=\"%d\"\xff\xff\xff",n);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
         }

         if(current1>=current_threshold||current_harmonics1>=current_harmonics_threshold)
         {
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备电流异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ch\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备电流异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ch\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
         }
         }
         else
         {
         sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ch\":\"normal\"},\"version\":\"1.0.0\"}");
         msh_exec(sendbuffer, strlen(sendbuffer));
         memset(sendbuffer,0,sizeof(sendbuffer));
        }
         if(x_speed1>=x_speed_threshold||y_speed1>=y_speed_threshold||z_speed1>=z_speed_threshold)
         {
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备速度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_v\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备速度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_v\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
         }
         }
         else
         {
         sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_v\":\"normal\"},\"version\":\"1.0.0\"}");
         msh_exec(sendbuffer, strlen(sendbuffer));
         memset(sendbuffer,0,sizeof(sendbuffer));
        }
         if(x_acceleration1>=x_acceleration_threshold||y_acceleration1>=y_acceleration_threshold||z_acceleration1>=z_acceleration_threshold)
         {
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备加速度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ac\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备加速度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ac\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
         }
         }
         else
         {
         sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_ac\":\"normal\"},\"version\":\"1.0.0\"}");
         msh_exec(sendbuffer, strlen(sendbuffer));
         memset(sendbuffer,0,sizeof(sendbuffer));
         }
         if(x_kurtosis1>=x_kurtosis_threshold||y_kurtosis1>=y_kurtosis_threshold||z_kurtosis1>=z_kurtosis_threshold)
         {
             if(x<7)
             {
             x=x+1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备峭度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_k\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
             }
             else
             {
             x=1;
             snprintf(screencmd,sizeof(screencmd),"address.w%d.txt=\"设备峭度异常\"+t0.txt\xff\xff\xff",x);
             rt_device_write(u4_dev, 0,screencmd,strlen(screencmd));
             sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_k\":\"alarm\"},\"version\":\"1.0.0\"}");
             msh_exec(sendbuffer, strlen(sendbuffer));
             memset(sendbuffer,0,sizeof(sendbuffer));
         }
         }
         else
         {
         sprintf(sendbuffer,"mqtt_publish /sys/ikhiECPnTEQ/l9Tx9wNv1E4Ux6zJVBiR/thing/event/property/post {\"method\":\"thing.service.property.set\",\"id\":\"952961143\",\"params\":{\"ts_k\":\"normal\"},\"version\":\"1.0.0\"}");
         msh_exec(sendbuffer, strlen(sendbuffer));
         memset(sendbuffer,0,sizeof(sendbuffer));
        }
         }
}

int main(void)
{
    char *cmd = "mqtt_start";
    msh_exec("mqtt_start", strlen(cmd));
    fal_init();
    easyflash_init();
    uart3init();
    uart4init();
    rt_sem_init(&sem2,"rt_sem2",0,RT_IPC_FLAG_FIFO);//阈值判断相关信号量
    rt_pin_write(LED_PIN_G, PIN_LOW);
        u3_th =rt_thread_create("u3_recv",serial3_thread_entry, NULL, 4096, 10, 60);//串口屏通讯串口
        if (u3_th != RT_NULL)
                   {
                       rt_thread_startup(u3_th);
                   }

        u4_th =rt_thread_create("u4_recv",serial4_thread_entry, NULL, 1024, 9, 60);//与Lora主机通讯串口
        if (u4_th != RT_NULL)
                   {
                       rt_thread_startup(u4_th);
                   }

       screen_sendcmd =rt_thread_create("screen",send_data_entry, NULL, 4096,16,50);//本地显示线程
       if (screen_sendcmd != RT_NULL)
                  {
                      rt_thread_startup(screen_sendcmd);
                  }

       cloud_publish =rt_thread_create("cloud",cloud_publish_entry, NULL, 4096,16,50);//上云线程
       if (cloud_publish != RT_NULL)
                  {
                     rt_thread_startup(cloud_publish);
                  }

       threshold_judgment =rt_thread_create("alarm",threshold_judgment_entry, NULL,4096,16,50);//告警线程
       if (threshold_judgment != RT_NULL)
                  {
                     rt_thread_startup(threshold_judgment);
                  }

    return RT_EOK;
}

int uart3init(void)
{
    rt_err_t ret = 0;
            u3_dev=rt_device_find("uart3");
            if (u3_dev == RT_NULL)
                    {
                        LOG_E("rt_device_find[uart3] failed...\n");
                        return -EINVAL;
                    }
            ret=rt_device_open(u3_dev, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_DMA_RX);
            if(ret <0)
            {
                LOG_E("rt_device_open[uart3] failed...\n");
                return ret;
            }
            rt_device_control(u3_dev, RT_DEVICE_CTRL_CONFIG, (void *)&u3_configs);

            rt_device_set_rx_indicate(u3_dev,rx3_callback);

            rt_sem_init(&sem,"rt_sem",0,RT_IPC_FLAG_FIFO);
            if(ret <0)
            {
                LOG_E("rt_sem_init failed[%d]...\n",ret);
                return ret;
            }
            return RT_EOK;
}

int uart4init(void)
{
    rt_err_t ret = 0;
            u4_dev=rt_device_find("uart4");
            if (u4_dev == RT_NULL)
                    {
                        LOG_E("rt_device_find[uart4] failed...\n");
                        return -EINVAL;
                    }
            ret=rt_device_open(u4_dev, RT_DEVICE_OFLAG_RDWR|RT_DEVICE_FLAG_DMA_RX);
            if(ret <0)
            {
                LOG_E("rt_device_open[uart4] failed...\n");
                return ret;
            }
            rt_device_control(u4_dev, RT_DEVICE_CTRL_CONFIG, (void *)&u4_configs);

            rt_device_set_rx_indicate(u4_dev,rx4_callback);

            rt_sem_init(&sem1,"rt_sem1",0,RT_IPC_FLAG_FIFO);
            if(ret <0)
            {
                LOG_E("rt_sem1_init failed[%d]...\n",ret);
                return ret;
            }
            return RT_EOK;
}
