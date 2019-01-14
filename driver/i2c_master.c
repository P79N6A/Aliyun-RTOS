/******************************************************************************
 * Copyright 2013-2014 Espressif Systems (Wuxi)
 *
 * FileName: i2c_master.c
 *
 * Description: i2c master API
 *
 * Modification history:
 *     2014/3/12, v1.0 create this file.
*******************************************************************************/
#include <stdint.h>
#include <string.h>
#include "c_types.h"
#include "esp8266/ets_sys.h"
#include "gpio.h"

#include "i2c_master.h"

LOCAL uint8 m_nLastSDA;
LOCAL uint8 m_nLastSCL;

uint8 IIC_TX_Buffer[]={0x03,0x00,0x04}; //读温湿度命令（无CRC校验）
uint8 IIC_RX_Buffer[10];

typedef union FLOAT_CONV
{
        float f;
        char c[4];
}FLOAT_CONV;

float BLEndianFloat(float fValue)
{
       FLOAT_CONV	d1, d2;

       d1.f = fValue;
       d2.c[0] = d1.c[3];
       d2.c[1] = d1.c[2];
       d2.c[2] = d1.c[1];
       d2.c[3] = d1.c[0];

       return d2.f;
}

extern void delay_ms(uint16 x);
/******************************************************************************
 * FunctionName : i2c_master_setDC
 * Description  : Internal used function -
 *                    set i2c SDA and SCL bit value for half clk cycle
 * Parameters   : uint8 SDA
 *                uint8 SCL
 * Returns      : NONE
*******************************************************************************/
LOCAL void ICACHE_FLASH_ATTR
i2c_master_setDC(uint8 SDA, uint8 SCL)
{
    SDA	&= 0x01;
    SCL	&= 0x01;
    m_nLastSDA = SDA;
    m_nLastSCL = SCL;
    ETS_INTR_LOCK();
    if ((0 == SDA) && (0 == SCL)) {
        I2C_MASTER_SDA_LOW_SCL_LOW();
    } else if ((0 == SDA) && (1 == SCL)) {
        I2C_MASTER_SDA_LOW_SCL_HIGH();
    } else if ((1 == SDA) && (0 == SCL)) {
        I2C_MASTER_SDA_HIGH_SCL_LOW();
    } else {
        I2C_MASTER_SDA_HIGH_SCL_HIGH();
    }
    ETS_INTR_UNLOCK();
}

/******************************************************************************
 * FunctionName : i2c_master_getDC
 * Description  : Internal used function -
 *                    get i2c SDA bit value
 * Parameters   : NONE
 * Returns      : uint8 - SDA bit value
*******************************************************************************/
LOCAL uint8 ICACHE_FLASH_ATTR
i2c_master_getDC(void)
{
    uint8 sda_out;
    ETS_INTR_LOCK();
    sda_out = GPIO_INPUT_GET(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO));
    ETS_INTR_UNLOCK();
    return sda_out;
}

/******************************************************************************
 * FunctionName : i2c_master_init
 * Description  : initilize I2C bus to enable i2c operations
 * Parameters   : NONE
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_init(void)
{
    uint8 i;

    i2c_master_setDC(1, 0);
    i2c_master_wait(5);

    // when SCL = 0, toggle SDA to clear up
    i2c_master_setDC(0, 0) ;
    i2c_master_wait(5);
    i2c_master_setDC(1, 0) ;
    i2c_master_wait(5);

    // set data_cnt to max value
    for (i = 0; i < 28; i++) {
        i2c_master_setDC(1, 0);
        i2c_master_wait(5);	// sda 1, scl 0
        i2c_master_setDC(1, 1);
        i2c_master_wait(5);	// sda 1, scl 1
    }

    // reset all
    i2c_master_stop();
    return;
}

/******************************************************************************
 * FunctionName : i2c_master_gpio_init
 * Description  : config SDA and SCL gpio to open-drain output mode,
 *                mux and gpio num defined in i2c_master.h
 * Parameters   : NONE
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_gpio_init(void)
{
//    ETS_GPIO_INTR_DISABLE() ;
//    ETS_INTR_LOCK();

    PIN_FUNC_SELECT(I2C_MASTER_SDA_MUX, I2C_MASTER_SDA_FUNC);
    PIN_FUNC_SELECT(I2C_MASTER_SCL_MUX, I2C_MASTER_SCL_FUNC);

    GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SDA_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SDA_GPIO));
    GPIO_REG_WRITE(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO)), GPIO_REG_READ(GPIO_PIN_ADDR(GPIO_ID_PIN(I2C_MASTER_SCL_GPIO))) | GPIO_PIN_PAD_DRIVER_SET(GPIO_PAD_DRIVER_ENABLE)); //open drain;
    GPIO_REG_WRITE(GPIO_ENABLE_ADDRESS, GPIO_REG_READ(GPIO_ENABLE_ADDRESS) | (1 << I2C_MASTER_SCL_GPIO));

    I2C_MASTER_SDA_HIGH_SCL_HIGH();

//    ETS_GPIO_INTR_ENABLE() ;
//    ETS_INTR_UNLOCK();

    i2c_master_init();
}

/******************************************************************************
 * FunctionName : i2c_master_start
 * Description  : set i2c to send state
 * Parameters   : NONE
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_start(void)
{
    i2c_master_setDC(1, m_nLastSCL);
    i2c_master_wait(5);
    i2c_master_setDC(1, 1);
    i2c_master_wait(5);	// sda 1, scl 1
    i2c_master_setDC(0, 1);
    i2c_master_wait(5);	// sda 0, scl 1
}

/******************************************************************************
 * FunctionName : i2c_master_stop
 * Description  : set i2c to stop sending state
 * Parameters   : NONE
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_stop(void)
{
    i2c_master_wait(5);

    i2c_master_setDC(0, m_nLastSCL);
    i2c_master_wait(5);	// sda 0
    i2c_master_setDC(0, 1);
    i2c_master_wait(5);	// sda 0, scl 1
    i2c_master_setDC(1, 1);
    i2c_master_wait(5);	// sda 1, scl 1
}

/******************************************************************************
 * FunctionName : i2c_master_setAck
 * Description  : set ack to i2c bus as level value
 * Parameters   : uint8 level - 0 or 1
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_setAck(uint8 level)
{
    i2c_master_setDC(m_nLastSDA, 0);
    i2c_master_wait(5);
    i2c_master_setDC(level, 0);
    i2c_master_wait(5);	// sda level, scl 0
    i2c_master_setDC(level, 1);
    i2c_master_wait(8);	// sda level, scl 1
    i2c_master_setDC(level, 0);
    i2c_master_wait(5);	// sda level, scl 0
    i2c_master_setDC(1, 0);
    i2c_master_wait(5);
}

/******************************************************************************
 * FunctionName : i2c_master_getAck
 * Description  : confirm if peer send ack
 * Parameters   : NONE
 * Returns      : uint8 - ack value, 0 or 1
*******************************************************************************/
uint8 ICACHE_FLASH_ATTR
i2c_master_getAck(void)
{
    uint8 retVal;
    i2c_master_setDC(m_nLastSDA, 0);
    i2c_master_wait(5);
    i2c_master_setDC(1, 0);
    i2c_master_wait(5);
    i2c_master_setDC(1, 1);
    i2c_master_wait(5);

    retVal = i2c_master_getDC();
    i2c_master_wait(5);
    i2c_master_setDC(1, 0);
    i2c_master_wait(5);

    return retVal;
}

/******************************************************************************
* FunctionName : i2c_master_checkAck
* Description  : get dev response
* Parameters   : NONE
* Returns      : true : get ack ; false : get nack
*******************************************************************************/
bool ICACHE_FLASH_ATTR
i2c_master_checkAck(void)
{
    if(i2c_master_getAck()){
        return FALSE;
    }else{
        return TRUE;
    }
}

/******************************************************************************
* FunctionName : i2c_master_send_ack
* Description  : response ack
* Parameters   : NONE
* Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_send_ack(void)
{
    i2c_master_setAck(0x0);
}
/******************************************************************************
* FunctionName : i2c_master_send_nack
* Description  : response nack
* Parameters   : NONE
* Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_send_nack(void)
{
    i2c_master_setAck(0x1);
}

/******************************************************************************
 * FunctionName : i2c_master_readByte
 * Description  : read Byte from i2c bus
 * Parameters   : NONE
 * Returns      : uint8 - readed value
*******************************************************************************/
uint8 ICACHE_FLASH_ATTR
i2c_master_readByte(uint8 ack)
{
    uint8 retVal = 0;
    uint8 k, i;

    i2c_master_wait(5);
    i2c_master_setDC(m_nLastSDA, 0);
    i2c_master_wait(5);	// sda 1, scl 0

    for (i = 0; i < 8; i++) {
        i2c_master_wait(5);
        i2c_master_setDC(1, 0);
        i2c_master_wait(5);	// sda 1, scl 0
        i2c_master_setDC(1, 1);
        i2c_master_wait(5);	// sda 1, scl 1

        k = i2c_master_getDC();
        i2c_master_wait(5);

        if (i == 7) {
            i2c_master_wait(3);   ////
        }

        k <<= (7 - i);
        retVal |= k;
    }

    i2c_master_setDC(1, 0);
    i2c_master_wait(5);	// sda 1, scl 0

	if(ack==0)
		i2c_master_send_nack();//主机非应答
	else
		i2c_master_send_ack();//主机应答
    return retVal;
}

/******************************************************************************
 * FunctionName : i2c_master_writeByte
 * Description  : write wrdata value(one byte) into i2c
 * Parameters   : uint8 wrdata - write value
 * Returns      : NONE
*******************************************************************************/
void ICACHE_FLASH_ATTR
i2c_master_writeByte(uint8 wrdata)
{
    uint8 dat;
    sint8 i;

    i2c_master_wait(5);

    i2c_master_setDC(m_nLastSDA, 0);
    i2c_master_wait(5);

    for (i = 7; i >= 0; i--) {
        dat = wrdata >> i;
        i2c_master_setDC(dat, 0);
        i2c_master_wait(5);
        i2c_master_setDC(dat, 1);
        i2c_master_wait(5);

        if (i == 0) {
            i2c_master_wait(3);   ////
        }

        i2c_master_setDC(dat, 0);
        i2c_master_wait(5);
    }
}

//***************************************************
uint8 WriteNByte(unsigned char sla,unsigned char *s,unsigned char n)
{
   unsigned char i;

   i2c_master_start();  //启动I2C
   i2c_master_writeByte(sla);//发送器件地址
   i2c_master_getAck();   //只产生时钟，不去判断ACK或者NACK是否正确，容易出错。

   for(i=0;i<n;i++)//写入8字节数据
   {
	  i2c_master_writeByte(*(s+i));
      i2c_master_getAck();
   }
   i2c_master_stop();
   return 1;
}

uint8 ReadNByte(unsigned char Sal, unsigned char *p,unsigned char n)
{
  unsigned char i;
  i2c_master_start();    // 启动I2C
  i2c_master_writeByte(Sal+0x01); //发送器件地址
  i2c_master_getAck();
  i2c_master_wait(40); // 延时时间必须大于30us 只要大于 30us 以上的值都可以 但是最好不要太长 ，测试时，试过25MS都OK！

  for(i=0;i<n-1;i++)  //读取字节数据
  {
     *(p+i)=i2c_master_readByte(1); //读取数据
  }
  *(p+n-1)=i2c_master_readByte(0);
  i2c_master_stop();
  return 1;
}

static void AM2320_wakeUp(void)
{
	i2c_master_start();       // 启动I2C
	i2c_master_writeByte(0xB8); // 发送器件地址
	i2c_master_getAck();// 唤醒指令时 传感器不会回ACK 但是第一定要发检测ACK的时钟 否则会出错
    i2c_master_setDC(1, 0);
	i2c_master_wait(1000);
	i2c_master_stop();
}

///计算CRC校验码
unsigned int CRC16(unsigned char *ptr, unsigned char len)
{
   unsigned int crc=0xffff;
   unsigned char i;
   while(len--)
   {
       crc ^=*ptr++;
       for(i=0;i<8;i++)
	   {
	       if(crc & 0x1)
		   {
		      crc>>=1;
			  crc^=0xa001;
		   }
		   else
		   {
		      crc>>=1;
		   }
	   }
   }
   return crc;
}
///检测CRC校验码是否正确
unsigned char CheckCRC(unsigned char *ptr,unsigned char len)
{
  unsigned int crc;
	crc=(unsigned int)CRC16(ptr,len-2);
	if(ptr[len-1]==(crc>>8) && ptr[len-2]==(crc & 0x00ff))
	{
	    return 0xff;
	}
	else
	{
	   return 0x0;
	}
}

void Clear_Data (void)
{
	int i;
	for(i=0;i<6;i++)
	{
	IIC_RX_Buffer[i] = 0x00;
	}
}

void AM2320_GetValue(int* hum1,int* temp1)
{
	uint8 i;
	int a,b;
    int hum,temp;
    char uart_temp[30];
    Clear_Data();                       // 清除收到数据
    AM2320_wakeUp();	                // 唤醒传感器
    WriteNByte(0xB8,IIC_TX_Buffer,3);
    delay_ms(2);                        //发送读取或写数据命令后，至少等待2MS（给传感器返回数据作时间准备）
    ReadNByte(0xB8,IIC_RX_Buffer,8);    //读返回数据
    i2c_master_wait(2);
    i2c_master_setDC(1, 1);	                        //确认释放总线

    if(CheckCRC(IIC_RX_Buffer,8))
         {
            a =(( IIC_RX_Buffer[2]<<8 )+ IIC_RX_Buffer[3]);
            b =(( IIC_RX_Buffer[4]<<8)+ IIC_RX_Buffer[5]);

            hum=a/10;
            temp=b/10;
            *hum1=hum;
            *temp1=temp;

            printf("hum: %d  temp: %d \n\r",hum,temp);

         }
         else
         {
            printf("Data: CRC Wrong\n");
         }

//        for(i = 0; i< 8; i++)
//        {
//            printf("%d Byte: %x \n\r", i+1,IIC_RX_Buffer[i]);
//        }

}

