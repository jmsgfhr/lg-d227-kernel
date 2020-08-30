/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */

/* MediaTek Inc. (C) 2010. All rights reserved.
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   DUMA
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 * Author:
 * -------
 *   PC Huang (MTK02204)
 *============================================================================
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <asm/io.h>
#include <asm/system.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"
#include "isx012mipi_yuv_Sensor.h"
#include "isx012mipi_yuv_Camera_Sensor_para.h"
#include "isx012mipi_yuv_CameraCustomized.h"

#define ISX012MIPIYUV_DEBUG
#ifdef ISX012MIPIYUV_DEBUG
#define ISX012MIPISENSORDB printk
#else
#define ISX012MIPISENSORDB(x,...)
#endif

//for burst mode
#define I2C_BUFFER_LEN 254 //MAX data to send by MT6572 i2c dma mode is 255 bytes
#define BLOCK_I2C_DATA_WRITE iBurstWriteReg

#define AEC_ROI_DX (40)
#define AEC_ROI_DY (40)

static DEFINE_SPINLOCK(isx012mipi_drv_lock);
static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//                                                                                
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

extern int mt_set_gpio_mode(unsigned long pin, unsigned long mode);
extern int mt_set_gpio_dir(unsigned long pin, unsigned long dir);
extern int mt_set_gpio_out(unsigned long pin, unsigned long output);


#define ISX012MIPI_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para ,1,ISX012MIPI_WRITE_ID)
//#define ISX012MIPI_write_cmos_sensor_16(addr, para) iWriteReg((u16) addr , (u32) para ,2,ISX012MIPI_WRITE_ID)
#define ISX012MIPI_write_cmos_sensor1(addr, para) iWriteReg((u16) addr , (u32) para ,1,ISX012MIPI_WRITE_ID1)
void ISX012MIPI_write_cmos_sensor_16(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF),(char)(para & 0xFF) ,(char)(para >> 8)};
	iWriteRegI2C(puSendCmd , 4,ISX012MIPI_WRITE_ID);
}
inline int ISX012MIPI_write_cmos_sensor1_16(kal_uint32 addr, kal_uint32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF),(char)(para & 0xFF) ,(char)(para >> 8)};
	iWriteRegI2C(puSendCmd , 4,ISX012MIPI_WRITE_ID1);
}

static int ISX012_table_write_cmos_sensor(isx012_short_t* para, kal_uint32 len, kal_uint32 slave_addr)
{
	isx012_short_t* pPara = (isx012_short_t*) para;
	kal_uint8 puSendCmd[I2C_BUFFER_LEN]={0,};
	kal_uint32 tosend=0 , IDX=0;
	kal_uint16 addr, addr_next, data;

	if(pPara == NULL)
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ERROR!! pPara is Null!!\n");
		return 0;
	}

	while(IDX < len)
	{
		addr = pPara->address;

		if (tosend == 0) // new (addr, data) to send
		{
			puSendCmd[tosend++] = (kal_uint8)(addr >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)(addr & 0xff);
			if(pPara->type == 2){ // 16bit
				data = pPara->data & 0xff;
				puSendCmd[tosend++] = (kal_uint8)data;
				data = (pPara->data >> 8) & 0xff;
				puSendCmd[tosend++] = (kal_uint8)data;
			}
			else{ // 8bit
				data = pPara->data;
				puSendCmd[tosend++] = (kal_uint8)data;
			}
			addr_next = addr + pPara->type;
			IDX ++;
			pPara++;

		}
		else if (addr == addr_next) // to multiple write the data to the incremental address
		{
			if(pPara->type == 2){ // 16bit
				data = pPara->data & 0xff;
				puSendCmd[tosend++] = (kal_uint8)data;
				data = (pPara->data >> 8) & 0xff;
				puSendCmd[tosend++] = (kal_uint8)data;
			}
			else{ // 8bit
				data = pPara->data;
				puSendCmd[tosend++] = (kal_uint8)data;
			}
			addr_next = addr + pPara->type;
			IDX ++;
			pPara++;
		}
		else // to send out the data if the address not incremental.
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr);
			tosend = 0;
		}

		// to send out the data if the sen buffer is full or last data.
		if ((tosend >= (I2C_BUFFER_LEN-8)) || (IDX == len))
		{
			BLOCK_I2C_DATA_WRITE(puSendCmd , tosend, slave_addr);
			tosend = 0;
		}
	}
	return 0;
}

#define mDELAY(ms)  mdelay(ms)

kal_uint8 ISX012MIPI_sensor_socket = DUAL_CAMERA_NONE_SENSOR;
typedef enum
{
    PRV_W=1280,
    PRV_H=960
}PREVIEW_VIEW_SIZE;
kal_uint16 ISX012MIPIYUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,ISX012MIPI_WRITE_ID);
    return get_byte;
}
kal_uint16 ISX012MIPIYUV_read_cmos_sensor1(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,ISX012MIPI_WRITE_ID1);
    return get_byte;
}
//                                                                                  
kal_uint16 ISX012MIPIYUV_read_cmos_sensor1_16(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    char puReadCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF)};
    iReadRegI2C( puReadCmd , 2, (u8*)&get_byte, 2, ISX012MIPI_WRITE_ID1);
    return get_byte;
}
//                                                                                  
static struct
{
	//kal_uint8   Banding;
	kal_bool	  NightMode;
	kal_bool      pv_mode;
	kal_bool      video_mode;
    kal_bool      capture_mode;
	kal_uint32    pv_dummy_lines;
	kal_uint32	  pv_dummy_pixels;
	kal_uint32    video_dummy_pixels;
	kal_uint32    video_dummy_lines;
	kal_uint32    cp_dummy_pixels;
	kal_uint32    cp_dummy_lines;

	kal_uint32 pv_line_length;
    kal_uint32 pv_frame_length;
	kal_uint32 video_line_length;
	kal_uint32 video_frame_length;
	kal_uint32 cp_line_length;
	kal_uint32 cp_frame_length;

	kal_uint32    PreviewPclk;
	kal_uint32      CapturePclk;
	kal_uint32      VideoPclk;

	kal_uint32 		pv_shutter;
	kal_uint32 		video_shutter;
	kal_uint32 		cp_shutter;

	kal_bool    	manualAEStart;
	kal_bool    	userAskAeLock;
    kal_bool    	userAskAwbLock;

	kal_uint32      currentExposureTime;
    kal_uint32      currentAxDGain;

	kal_uint32  	sceneMode;

    unsigned char isoSpeed;

	kal_uint16 af_xcoordinate;
	kal_uint16 af_ycoordinate;
	unsigned char   awbMode;

	unsigned char Flash_OnOff;
	unsigned char Flash_AE_Start;
	//                                                                 
	unsigned char exposure_value;
	unsigned char flash_mode;
	// set AE for flashlight
	kal_uint16 AE_auto;
	short int ERSC_auto;
	kal_uint16 AE_now;
	short int ERSC_now;
	kal_uint32 AE_scl;
	//                                                                 

	ISX012MIPI_SENSOR_MODE SensorMode;

	kal_bool		is_touch;
} ISX012MIPISensor;
/* Global Valuable */
static kal_uint32 zoom_factor = 0;
static kal_int8 ISX012MIPI_DELAY_AFTER_PREVIEW = -1;
static kal_uint8 ISX012MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
static kal_bool ISX012MIPI_AWB_ENABLE = KAL_TRUE;
static kal_bool ISX012MIPI_AE_ENABLE = KAL_TRUE;
MSDK_SENSOR_CONFIG_STRUCT ISX012MIPISensorConfigData;
#define ISX012_TEST_PATTERN_CHECKSUM (0x7ba87eae)
void ISX012MIPI_set_scene_mode(UINT16 para);
BOOL ISX012MIPI_set_param_wb(UINT16 para);
/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_dummy
*
* DESCRIPTION
*	This function set the dummy pixels(Horizontal Blanking) & dummy lines(Vertical Blanking), it can be
*	used to adjust the frame rate or gain more time for back-end process.
*
*	IMPORTANT NOTICE: the base shutter need re-calculate for some sensor, or else flicker may occur.
*
* PARAMETERS
*	1. kal_uint32 : Dummy Pixels (Horizontal Blanking)
*	2. kal_uint32 : Dummy Lines (Vertical Blanking)
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIinitalvariable(void)
{
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.video_mode = KAL_FALSE;
	ISX012MIPISensor.pv_mode = KAL_TRUE;
    ISX012MIPISensor.capture_mode= KAL_FALSE;
	ISX012MIPISensor.NightMode = KAL_FALSE;
	ISX012MIPISensor.cp_dummy_pixels = 0;
	ISX012MIPISensor.cp_dummy_lines = 0;
	ISX012MIPISensor.pv_dummy_pixels = 0;
	ISX012MIPISensor.pv_dummy_lines = 0;
	ISX012MIPISensor.video_dummy_pixels = 0;
	ISX012MIPISensor.video_dummy_lines = 0;
	ISX012MIPISensor.SensorMode= SENSOR_MODE_INIT;
	ISX012MIPISensor.pv_mode= KAL_TRUE;
	/******************************xiaoyu*****************************************/
	ISX012MIPISensor.PreviewPclk=864;
	ISX012MIPISensor.CapturePclk=864;
	ISX012MIPISensor.VideoPclk=864;
	ISX012MIPISensor.pv_shutter=0;
	ISX012MIPISensor.video_shutter=0;
	ISX012MIPISensor.cp_shutter=0;
	/******************************xiaoyu*****************************************/
	ISX012MIPISensor.manualAEStart=0;
	ISX012MIPISensor.isoSpeed=AE_ISO_100;
	ISX012MIPISensor.exposure_value = 0;  //                                                               
	ISX012MIPISensor.flash_mode = FLASH_MODE_FORCE_OFF;  //                                                               
	ISX012MIPISensor.userAskAeLock=KAL_FALSE;
    ISX012MIPISensor.userAskAwbLock=KAL_FALSE;

	ISX012MIPISensor.currentExposureTime=0;//for HDR save current shutter
    ISX012MIPISensor.currentAxDGain=0;//for HDR save current shutter

	ISX012MIPISensor.awbMode = AWB_MODE_AUTO;
	ISX012MIPISensor.Flash_AE_Start=0;

	ISX012MIPISensor.is_touch=false;
	spin_unlock(&isx012mipi_drv_lock);
}

//                                                                                        
static kal_uint16 ISX012MIPIGetRealIso(void)
{
	kal_uint16 iso_tbl[] = {
		25, 32, 40, 50, 64, 80, 100, 125, 160, 200,
		250, 320, 400, 500, 640, 800, 1000, 1250, 1600 };
	kal_uint8 iso_idx = ISX012MIPIYUV_read_cmos_sensor1(0x019A) - 1;

	if (iso_idx >= 19)
		iso_idx = 18; // ISO_1600

	//                                                                                                                     
	if (iso_idx <= 2)
		iso_idx = 3;  // ISO_50

	return (ISX012MIPISensor.isoSpeed == AE_ISO_AUTO) ? iso_tbl[iso_idx] : ISX012MIPISensor.isoSpeed;
}
//                                              

//                                                                 
#define AE_MAXDIFF 4000
#define AE_OFSETVAL 3450
const uint16_t aeoffset_table[] = {
	0, 35, 70, 103, 136, 167, 198, 228, 257, 285,
	313, 339, 366, 391, 416, 441, 465, 488, 511, 533,
	555, 576, 597, 618, 638, 657, 677, 696, 714, 732,
	750, 768, 785, 802, 818, 835, 851, 866, 882, 897,
	912, 927, 941, 955, 969, 983, 997, 1010, 1023, 1036,
	1049, 1061, 1074, 1086, 1098, 1109, 1121, 1133, 1144, 1155,
	1166, 1177, 1187, 1198, 1208, 1219, 1229, 1239, 1248, 1258,
	1268, 1277, 1286, 1296, 1305, 1314, 1322, 1331, 1340, 1348,
	1357, 1365, 1373, 1381, 1389, 1397, 1405, 1413, 1420, 1428,
	1435, 1443, 1450, 1457, 1464, 1471, 1478, 1485, 1492, 1499,
	1505, 1512, 1518, 1525, 1531, 1538, 1544, 1550, 1556, 1562,
	1568, 1574, 1580, 1585, 1591, 1597, 1602, 1608, 1613, 1619,
	1624, 1629, 1635, 1640, 1645, 1650, 1655, 1660, 1665, 1670,
	1675, 1679, 1684, 1689, 1693, 1698, 1703, 1707, 1712, 1716,
	1720, 1725, 1729, 1733, 1737, 1742, 1746, 1750, 1754, 1758,
	1762, 1766, 1770, 1774, 1777, 1781, 1785, 1789, 1792, 1796,
	1800, 1803, 1807, 1810, 1814, 1817, 1821, 1824, 1828, 1831,
	1834, 1837, 1841, 1844, 1847, 1850, 1853, 1856, 1860, 1863,
	1866, 1869, 1872, 1875, 1877, 1880, 1883, 1886, 1889, 1892,
	1894, 1897, 1900, 1903, 1905, 1908, 1911, 1913, 1916, 1918,
	1921, 1923, 1926, 1928, 1931, 1933, 1936, 1938, 1941, 1943,
	1945, 1948, 1950, 1952, 1954, 1957, 1959, 1961, 1963, 1965,
	1968, 1970, 1972, 1974, 1976, 1978, 1980, 1982, 1984, 1986,
	1988, 1990, 1992, 1994, 1996, 1998, 2000, 2002, 2003, 2005,
	2007, 2009, 2011, 2013, 2014, 2016, 2018, 2020, 2021, 2023,
	2025, 2026, 2028, 2030, 2031, 2033, 2034, 2036, 2038, 2039,
	2041, 2042, 2044, 2045, 2047, 2048, 2050, 2051, 2053, 2054,
	2056, 2057, 2059, 2060, 2061, 2063, 2064, 2066, 2067, 2068,
	2070, 2071, 2072, 2074, 2075, 2076, 2077, 2079, 2080, 2081,
	2082, 2084, 2085, 2086, 2087, 2089, 2090, 2091, 2092, 2093,
	2094, 2096, 2097, 2098, 2099, 2100, 2101, 2102, 2103, 2104,
	2105, 2106, 2107, 2109, 2110, 2111, 2112, 2113, 2114, 2115,
	2116, 2117, 2118, 2119, 2120, 2120, 2121, 2122, 2123, 2124,
	2125, 2126, 2127, 2128, 2129, 2130, 2130, 2131, 2132, 2133,
	2134, 2135, 2136, 2136, 2137, 2138, 2139, 2140, 2141, 2141,
	2142, 2143, 2144, 2144, 2145, 2146, 2147, 2148, 2148, 2149,
	2150, 2150, 2151, 2152, 2153, 2153, 2154, 2155, 2155, 2156,
	2157, 2158, 2158, 2159, 2160, 2160, 2161, 2162, 2162, 2163,
	2163, 2164, 2165, 2165, 2166, 2167, 2167, 2168, 2168, 2169,
	2170, 2170, 2171, 2171, 2172, 2172, 2173, 2174, 2174, 2175,
	2175, 2176, 2176, 2177, 2177, 2178, 2179, 2179, 2180, 2180,
	2181, 2181, 2182, 2182, 2183, 2183, 2184, 2184, 2185, 2185,
	2186, 2186, 2186, 2187, 2187, 2188, 2188, 2189, 2189, 2190,
	2190, 2191, 2191, 2191, 2192, 2192, 2193, 2193, 2194, 2194,
	2194, 2195, 2195, 2196, 2196, 2196, 2197, 2197, 2198, 2198,
	2198, 2199, 2199, 2200, 2200, 2200, 2201, 2201, 2201, 2202,
	2202, 2203, 2203, 2203, 2204, 2204, 2204, 2205, 2205, 2205,
	2206, 2206, 2206, 2207, 2207, 2207, 2208, 2208, 2208, 2209,
	2209, 2209, 2210, 2210, 2210, 2210, 2211, 2211, 2211, 2212,
	2212, 2212, 2213, 2213, 2213, 2213, 2214, 2214, 2214, 2214,
};
//                                                                 

void ISX012MIPIGetExifInfo(UINT32 exifAddr)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetExifInfo function\n");
	SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
	pExifInfo->FNumber = 27;                         //                                                           
	pExifInfo->AEISOSpeed = ISX012MIPIGetRealIso();  //                                                                                      
	pExifInfo->AWBMode = ISX012MIPISensor.awbMode;
	pExifInfo->CapExposureTime = 0;
	pExifInfo->FlashLightTimeus = 0;
	pExifInfo->RealISOValue = pExifInfo->AEISOSpeed; //                                                                                      
	pExifInfo->FocalLength = 343;                    //                                                               
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetExifInfo function\n");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPIWriteShutter
*
* DESCRIPTION
*	This function used to write the shutter.
*
* PARAMETERS
*	1. kal_uint32 : The shutter want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIWriteShutter(kal_uint32 shutter)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIWriteShutter function\n");
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIWriteShutter function\n");
}    /* ISX012MIPI_write_shutter */
/*************************************************************************
* FUNCTION
*	ISX012MIPIWriteSensorGain
*
* DESCRIPTION
*	This function used to write the sensor gain.
*
* PARAMETERS
*	1. kal_uint32 : The sensor gain want to apply to sensor.
*
* RETURNS
*	None
*
*************************************************************************/
static void ISX012MIPIWriteSensorGain(kal_uint32 gain)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIWriteSensorGain function:gain=%d\n",gain);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIWriteSensorGain function:\n ");
}  /* ISX012MIPI_write_sensor_gain */

/*************************************************************************
* FUNCTION
*	ISX012MIPIReadShutter
*
* DESCRIPTION
*	This function read current shutter for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current shutter value.
*
*************************************************************************/
#if 0
static kal_uint32 ISX012MIPIReadShutter(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIReadShutter function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.pv_shutter  = 0;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIReadShutter function:\n ");
	return ISX012MIPISensor.pv_shutter;
} /* ISX012MIPI_read_shutter */
#endif

/*************************************************************************
* FUNCTION
*	ISX012MIPIReadSensorGain
*
* DESCRIPTION
*	This function read current sensor gain for calculate the exposure.
*
* PARAMETERS
*	None
*
* RETURNS
*	kal_uint16 : The current sensor gain value.
*
*************************************************************************/
#if 0
static kal_uint32 ISX012MIPIReadSensorGain(void)
{
	kal_uint32 sensor_gain = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIReadSensorGain function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIReadSensorGain function:\n ");
	return sensor_gain;
}  /* ISX012MIPIReadSensorGain */
#endif
/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_AE_mode
*
* DESCRIPTION
*	This function ISX012MIPI_set_AE_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#define REG_CPUEXT_AE_HOLD		(0x01 << 1)
static void ISX012MIPI_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 AeTemp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AE_mode function:\n ");
    //AeTemp = ISX012MIPIYUV_read_cmos_sensor(0x3503);
    if (AE_enable == KAL_TRUE)
    { //Lock
        AeTemp = ISX012MIPIYUV_read_cmos_sensor1(0x5000);
        AeTemp |= REG_CPUEXT_AE_HOLD;
        ISX012MIPI_write_cmos_sensor1(0x5000,AeTemp);
    }
    else
    { //Unlock
        AeTemp = ISX012MIPIYUV_read_cmos_sensor1(0x5000);
        AeTemp &= ~REG_CPUEXT_AE_HOLD;
        ISX012MIPI_write_cmos_sensor1(0x5000,AeTemp);
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_AE_mode function:\n ");
}

/*************************************************************************
* FUNCTION
*	ISX012MIPI_set_AWB_mode
*
* DESCRIPTION
*	This function ISX012MIPI_set_AWB_mode.
*
* PARAMETERS
*	none
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
#define REG_CPUEXT_AWB_PULLALL		(0x01 << 1)
static void ISX012MIPI_set_AWB_mode(kal_bool AWB_enable)
{
    kal_uint8 AwbTemp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AWB_mode function:\n ");
    if (AWB_enable == KAL_TRUE)
    { // MODE Change - PULLALL
        AwbTemp = ISX012MIPIYUV_read_cmos_sensor1(0x0282);
        AwbTemp |= REG_CPUEXT_AWB_PULLALL;
        ISX012MIPI_write_cmos_sensor1(0x0282,AwbTemp);
    }
    else
    {
        AwbTemp = ISX012MIPIYUV_read_cmos_sensor1(0x0282);
        AwbTemp &= ~REG_CPUEXT_AWB_PULLALL;
        ISX012MIPI_write_cmos_sensor1(0x0282,AwbTemp);
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_AWB_mode function:\n ");
}
#if 0
static void ISX012MIPI_set_AWB_mode_UNLOCK()
{
    ISX012MIPI_set_AWB_mode(KAL_TRUE);
    if (!((SCENE_MODE_OFF == ISX012MIPISensor.sceneMode) || (SCENE_MODE_NORMAL ==
    ISX012MIPISensor.sceneMode) || (SCENE_MODE_HDR == ISX012MIPISensor.sceneMode)))
    {
      ISX012MIPI_set_scene_mode(ISX012MIPISensor.sceneMode);
    }
    if (!((AWB_MODE_OFF == ISX012MIPISensor.awbMode) || (AWB_MODE_AUTO == ISX012MIPISensor.awbMode)))
    {
	   ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_AWB_mode_UNLOCK function:awbMode=%d\n ",ISX012MIPISensor.awbMode);
	   ISX012MIPI_set_param_wb(ISX012MIPISensor.awbMode);
    }
    return;
}
#endif
/*************************************************************************
* FUNCTION
*	ISX012MIPI_GetSensorID
*
* DESCRIPTION
*	This function get the sensor ID
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//static
extern int g_lge_camera; //                                                                                      
kal_uint32 ISX012MIPI_GetSensorID(kal_uint32 *sensorID)
{
	//                                                                 
	if(g_lge_camera==0) //                                                                      
	{
		*sensorID =0xffffffff;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	//                                                                 
    volatile signed char i;
	kal_uint32 sensor_id=0;
	kal_uint32 sensor_tmp=0;
	//kal_uint8 temp_sccb_addr = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_GetSensorID function:\n ");
	for(i=0;i<3;i++)
	{
		sensor_id  = (ISX012MIPIYUV_read_cmos_sensor(0x000E));
		sensor_tmp = sensor_id & 0x01;
		ISX012MIPISENSORDB("ISX012MIPI READ ID: %x",sensor_tmp);
		if(sensor_tmp)
		{
			*sensorID=ISX012MIPI_SENSOR_ID;
		        break;
		}
	}
	if(*sensorID != ISX012MIPI_SENSOR_ID)
	{
		*sensorID =0xffffffff;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_GetSensorID function:\n ");
    return ERROR_NONE;
}
UINT32 ISX012SetTestPatternMode(kal_bool bEnable)
{
	ISX012MIPISENSORDB("[ISX012MIPI_ISX012SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);
	if(bEnable)
	{
	}
	else
	{
	}
	return ERROR_NONE;
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_OM
*
* DESCRIPTION
*	This function wait the 0x000E bit 0 is 1;then clear the bit 0;
*      The salve address is 0x34
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_OM(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_OM function:\n ");
    do
    {
	tmp=ISX012MIPIYUV_read_cmos_sensor(0x000E) & 0x01;
	}while(!tmp);
	ISX012MIPI_write_cmos_sensor(0x0012,0x01);
	mDELAY(10);
	do
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor(0x000E) & 0x01;
	}while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_OM function:\n ");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_CM
*
* DESCRIPTION
*	This function wait the 0x000E bit 1 is 1;then clear the bit 1;
*      The salve address is 0x78
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_CM(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_CM function:\n ");
    do
    {
	tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) & 0x02;
	}while(tmp !=0x02);
	ISX012MIPI_write_cmos_sensor1(0x0012,0x02);
	mDELAY(10);
	do
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) & 0x02;
	}
	while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_CM function:\n ");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPI_WAIT_OM1
*
* DESCRIPTION
*	This function wait the 0x000E bit 0 is 1;then clear the bit 0;
*      The salve address is 0x78
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void ISX012MIPI_WAIT_OM1(void)
{
	kal_uint32 tmp;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_WAIT_OM1 function:\n ");
	do
    {
	tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) & 0x01;
	}while(!tmp);
	ISX012MIPI_write_cmos_sensor1(0x0012,0x01);
	mDELAY(10);
	do
	{
         tmp=ISX012MIPIYUV_read_cmos_sensor1(0x000E) & 0x01;
	}while(tmp);
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_WAIT_OM1 function:\n ");
}

/*************************************************************************
* FUNCTION
*    ISX012MIPIInitialSetting
*
* DESCRIPTION
*    This function initialize the registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
//static
void ISX012MIPIInitialSetting(void)
{
	kal_uint32 len=0;
	//;ISX012MIPI preview  640*480      30fps
	//;ISX012MIPI capture  2560*1920   7.5fps
	//;ISX012MIPI video     1280*720     30fps
	//86.4Mhz, 432Mbps/Lane, 2 Lane
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIInitialSetting function:\n ");
	ISX012MIPI_WAIT_OM();

	len = sizeof(ISX012_Pll_Reg) / sizeof(ISX012_Pll_Reg[0]);
	ISX012_table_write_cmos_sensor(ISX012_Pll_Reg,len,ISX012MIPI_WRITE_ID);

	ISX012MIPI_write_cmos_sensor(0x0006,0x16);

	mDELAY(200);
	ISX012MIPI_WAIT_OM();

	mt_set_gpio_mode(GPIO88,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO88,1);
	mt_set_gpio_out(GPIO88,1);

	ISX012MIPI_WAIT_OM1();
	ISX012MIPI_WAIT_CM();

	ISX012MIPI_write_cmos_sensor1(0x5008,0x00);

	// ISX012_Init_Reg
#if 1
	/////////////////////////////////////
	//AF driver setting                //
	/////////////////////////////////////
	ISX012MIPI_write_cmos_sensor1(0x66C2,0x0C);     //AF_INTERNAL_LENSDRV_ADRS
	ISX012MIPI_write_cmos_sensor1(0x66C3,0x03);     //AF_INTERNAL_LENSDRV_SIZE
	ISX012MIPI_write_cmos_sensor1(0x000B,0x01);     //AF_EXT : AF driver start
	mDELAY(33);
	ISX012MIPI_write_cmos_sensor1_16(0x66C8,0x0000); //AF_INTERNAL_LENSDRV_FIXEDPTN
	ISX012MIPI_write_cmos_sensor1_16(0x66CA,0x0131);
	ISX012MIPI_write_cmos_sensor1(0x66CC,0x01);
	 mDELAY(33);
	ISX012MIPI_write_cmos_sensor1(0x66C5,0x08);     //AF_INTERNAL_LENSDRV_SHIFT
	ISX012MIPI_write_cmos_sensor1_16(0x66C8,0x0000); //AF_INTERNAL_LENSDRV_FIXEDPTN
	ISX012MIPI_write_cmos_sensor1_16(0x66CA,0x0200);
#endif

	len = sizeof(ISX012_Init_Reg) / sizeof(ISX012_Init_Reg[0]);
	ISX012_table_write_cmos_sensor(ISX012_Init_Reg,len,ISX012MIPI_WRITE_ID1);

	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIInitialSetting function:\n ");

}
/*****************************************************************
* FUNCTION
*    ISX012MIPIPreviewSetting
*
* DESCRIPTION
*    This function config Preview setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
#if 0
static void ISX012MIPIPreviewSetting_SVGA(void)
{
	//;ISX012MIPI 1280x960,30fps
	//86.4Mhz, 432Mbps/Lane, 2Lane
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIPreviewSetting_SVGA function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIPreviewSetting_SVGA function:\n ");
}
#endif
/*************************************************************************
* FUNCTION
*     ISX012MIPIFullSizeCaptureSetting
*
* DESCRIPTION
*    This function config full size capture setting related registers of CMOS sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
#if 0
static void ISX012MIPIFullSizeCaptureSetting(void)
{
	//ISX012MIPI 2592x1944,7.5fps
	//86.4Mhz, 432Mbps/Lane, 2Lane
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIFullSizeCaptureSetting function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIFullSizeCaptureSetting function:\n ");
}
#endif
/*************************************************************************
* FUNCTION
*    ISX012MIPISetHVMirror
*
* DESCRIPTION
*    This function set sensor Mirror
*
* PARAMETERS
*    Mirror
*
* RETURNS
*    None
*
* LOCAL AFFECTED
*
*************************************************************************/
static void ISX012MIPISetHVMirror(kal_uint8 Mirror, kal_uint8 Mode)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPISetHVMirror function:\n ");
	if (Mode==SENSOR_MODE_PREVIEW)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x02);
				break;
			case IMAGE_V_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x01);
				break;
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008c,0x3);
				break;
			default:
				break;
		}
	}
	else if (Mode== SENSOR_MODE_CAPTURE)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x02);
				break;
			case IMAGE_V_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x01);
				break;
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008d,0x03);
				break;
			default:
				break;
		}
	}
	else if (Mode== SENSOR_MODE_VIDEO)
	{
		switch (Mirror)
		{
			case IMAGE_NORMAL:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x00);
				break;
			case IMAGE_H_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x02);
				break;
			case IMAGE_V_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x01);
				break;
			case IMAGE_HV_MIRROR:
				ISX012MIPI_write_cmos_sensor1(0x008e,0x03);
				break;
			default:
				break;
		}
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPISetHVMirror function:\n ");
}

void ISX012MIPI_Standby(void)
{
}

void ISX012MIPI_Wakeup(void)
{
}
/*************************************************************************
* FUNCTION
*   ISX012_FOCUS_OVT_AFC_Init
* DESCRIPTION
*   This function is to load micro code for AF function
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void ISX012_FOCUS_Move_to(UINT16 focus_level)
{
	#define MIN_LENS_POS        200	// infinite
	#define MAX_LENS_POS        600	// macro
	#define LENS_STEP           6
	#define MIN_FOCUS_LEVEL     0
	#define MAX_FOCUS_LEVEL     60

	UINT16 lens_pos;

	if (focus_level > MAX_FOCUS_LEVEL)
		return;

	if (focus_level == MAX_FOCUS_LEVEL)
		lens_pos = MIN_LENS_POS;
	else
		lens_pos = MAX_LENS_POS-LENS_STEP*focus_level;

	ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Move_to (%d)\n", focus_level);
	ISX012MIPI_write_cmos_sensor1(0x00B2, 0x03);         // AFMODE_MONI
	ISX012MIPI_write_cmos_sensor1(0x00B2, 0x02);         // AFMODE_MONI : Manual AF mode
	ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);         // MODESEL : Monitoring mode
	ISX012MIPI_write_cmos_sensor1_16(0x6648, lens_pos);  // AF_MANUAL_POS : MANUA AF search start position
	ISX012MIPI_write_cmos_sensor1(0x00B1, 0x01);         // AF_RESTART_F
	ISX012MIPI_write_cmos_sensor1(0x0082, 0x01);         // MONI_REFRESH
}
void ISX012_check_modesel()
{
	int MODESEL_FIX=0, HALF_MOVE_STS=0, timeout_cnt=0;

	if(ISX012MIPISensor.Flash_AE_Start==2)
	{
		while(1)
		{
			MODESEL_FIX = ISX012MIPIYUV_read_cmos_sensor1(0x0080);
			if(MODESEL_FIX == 0x1)
			{
				break;
			}
			if(timeout_cnt >= 300)
			{
				ISX012MIPISENSORDB("[ISX012MIPI][ERROR] MODESEL_FIX time out.\n");
				break;
			}
			msleep(10);
			timeout_cnt++;
		}
		timeout_cnt=0;
		while(1)
		{
			HALF_MOVE_STS = ISX012MIPIYUV_read_cmos_sensor1(0x01B0);
			if(HALF_MOVE_STS == 0x0)
			{
				break;
			}
			if(timeout_cnt >= 300)
			{
				ISX012MIPISENSORDB("[ISX012MIPI][ERROR] HALF_MOVE_STS time out.\n");
				break;
			}
			msleep(10);
			timeout_cnt++;
		}
		ISX012MIPISensor.Flash_AE_Start=1;
	}
}

void ISX012_FOCUS_Get_AF_Status(UINT32 *pFeatureReturnPara32)
{
	kal_uint16 af_state, af_result;

	if(ISX012MIPISensor.Flash_OnOff == 1)
	{
		ISX012_check_modesel();
	}

	af_state = ISX012MIPIYUV_read_cmos_sensor1(0x8B8A);
	if (af_state != 8)
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Get_AF_Status: focusing...\n");
		*pFeatureReturnPara32 = SENSOR_AF_FOCUSING;
		return;
	}

	ISX012MIPI_write_cmos_sensor1(0x0012,0x10);	// INTCLR : Clear AF_LOCK_STS
	//
	af_result = ISX012MIPIYUV_read_cmos_sensor1(0x8B8B);
	if (af_result)
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Get_AF_Status: success!\n");
		*pFeatureReturnPara32 = SENSOR_AF_FOCUSED;
	}
	else
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Get_AF_Status: failure!\n");
		*pFeatureReturnPara32 = SENSOR_AF_ERROR;
	}
	//

	if(ISX012MIPISensor.Flash_OnOff == 1)
	{
		//                                                                 
		// Read data for Flash AE
		ISX012MIPISensor.ERSC_now = ISX012MIPIYUV_read_cmos_sensor1_16(0x01CC);
		ISX012MIPISensor.AE_now = ISX012MIPIYUV_read_cmos_sensor1_16(0x01D0);
		ISX012MIPISensor.AE_scl = ISX012MIPIYUV_read_cmos_sensor1_16(0x8BC0);
		//                                                                 
		ISX012MIPI_set_AE_mode(true); //AE Lock
		ISX012MIPI_write_cmos_sensor1(0x01AE,0x00); //Half AWB Lock
		ISX012MIPISensor.Flash_AE_Start = 0;
		return;
	}

	if (ISX012MIPISensor.is_touch)
	{
		// isx012_AF_TouchSAF_OFF
		ISX012MIPI_write_cmos_sensor1(0x6674,0x00);	// AF_MONICHG_MOVE_F
		ISX012MIPI_write_cmos_sensor1(0x0082,0x01);	// MONI_REFRESH
		msleep(33);
		ISX012MIPI_write_cmos_sensor1(0x00B2,0x03);	// AFMODE_MONI : AF OFF
		ISX012MIPI_write_cmos_sensor1(0x00B3,0x03);	// AFMODE_HREL : AF OFF
		msleep(33);
		ISX012MIPI_write_cmos_sensor1(0x0081,0x00);	 //MODESEL : AF unlock
		msleep(66);
	}
	else
	{
		// ISX012_AF_SAF_OFF
		ISX012MIPI_write_cmos_sensor1(0x6674,0x00);	// AF_MONICHG_MOVE_F
		ISX012MIPI_write_cmos_sensor1(0x0082,0x01);	// MONI_REFRESH
		msleep(66);
		ISX012MIPI_write_cmos_sensor1(0x00B2,0x03);	// AFMODE_MONI : AF OFF
		ISX012MIPI_write_cmos_sensor1(0x00B3,0x03);	// AFMODE_HREL : AF OFF
		msleep(66);
	}
}

void ISX012_FOCUS_Single_Focus(void)
{
	//                                                                                 
	kal_uint8 gain_level = ISX012MIPIYUV_read_cmos_sensor1(0x01A5);

	if (gain_level >= 0x3D)
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Single_Focus (low lux)\n");
		ISX012MIPI_write_cmos_sensor1(0x660E,0x04); // AF_HBPF_PEAK_OPD_TH_MIN
		ISX012MIPI_write_cmos_sensor1(0x6610,0x04); // AF_HBPF_PEAK_OPD_TH_MAX
		ISX012MIPI_write_cmos_sensor1(0x664A,0x01); // AF_DROPN_ON_PEAK_DETECT :
		ISX012MIPI_write_cmos_sensor1(0x6640,0x01); // AF_DROPN_ON_PEAK_DETECT_SECOND :

		ISX012MIPI_write_cmos_sensor1(0x6616,0x00); // direct back off  first
		ISX012MIPI_write_cmos_sensor1(0x663F,0x01); // direct back on   fine
	}
	else
	{
		ISX012MIPISENSORDB("[ISX012MIPI] ISX012_FOCUS_Single_Focus (normal lux)\n");
		ISX012MIPI_write_cmos_sensor1(0x660E,0x40); // AF_HBPF_PEAK_OPD_TH_MIN
		ISX012MIPI_write_cmos_sensor1(0x6610,0x40); // AF_HBPF_PEAK_OPD_TH_MAX
		ISX012MIPI_write_cmos_sensor1(0x664A,0x02); // AF_DROPN_ON_PEAK_DETECT :
		ISX012MIPI_write_cmos_sensor1(0x6640,0x02); // AF_DROPN_ON_PEAK_DETECT_SECOND :

		ISX012MIPI_write_cmos_sensor1(0x6616,0x01); // direct back off  first
		ISX012MIPI_write_cmos_sensor1(0x663F,0x01); // direct back on   fine
	}

	ISX012MIPI_write_cmos_sensor1(0x0082,0x01); // MONI_REFRESH
	ISX012MIPI_write_cmos_sensor1(0x00B1,0x01); // AF_RESTART_F :
	ISX012MIPI_write_cmos_sensor1(0x00B2,0x03); // AFMODE_MONI : AF OFF
	ISX012MIPI_write_cmos_sensor1(0x00B3,0x00); // AFMODE_HREL :
	mDELAY(33);
	ISX012MIPI_write_cmos_sensor1(0x0081,0x01); // MODESEL
	if(ISX012MIPISensor.Flash_OnOff == 1)
	{// Vlatch On
		ISX012MIPI_write_cmos_sensor1(0x8800,0x1);
		mdelay(40);
	}
}

static void ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas function:\n ");
	*pFeatureReturnPara32 = 1;
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas function:\n ");
}

static void ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
	*pFeatureReturnPara32 = 1;
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas function:\n ");
}

static void ISX012_FOCUS_Get_AF_Macro(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}
static void ISX012_FOCUS_Get_AF_Inf(UINT32 *pFeatureReturnPara32)
{
    *pFeatureReturnPara32 = 0;
}

static void ISX012_FOCUS_Set_AF_Window(UINT32 zone_addr)
{

    INT32 x0, y0, x1, y1, width, height, coordinate_x, coordinate_y;
    UINT32 *ptr = (UINT32*)zone_addr;
	INT32 h_ratio, v_ratio;
	INT32 x, y, h, v;
	UINT32 len;

    x0 = *ptr;
    y0 = *(ptr + 1);
    x1 = *(ptr + 2);
    y1 = *(ptr + 3);
    width = *(ptr + 4);
    height = *(ptr + 5);
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] AF_Set_Window 3AWin: (%d,%d)~(%d,%d) w=%d h=%d\n",x0, y0, x1, y1, width, height);

	if (((x0==124) && (y0==84) && (x1==195) && (y1==155)) ||  // 4:3 default
	    ((x0==131) && (y0==84) && (x1==188) && (y1==155)) ||  // 16:9 default
	    ((x0==160) && (y0==120) && (x1==160) && (y1==120)))   //                                                                    
	{
		// ISX012_AF_Init_Macro_Off setting
		ISX012MIPI_write_cmos_sensor1(0x6674,0x01);
		return;
	}

	// AF window Size & Weight
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] AF_WIN set\n");

	// 320x240 base
	coordinate_x = (x0 + x1)/2;
	coordinate_y = (y0 + y1)/2;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] coordinate (%d, %d)\n", coordinate_x, coordinate_y);

	// 320x240 base
	x = (coordinate_x - (AEC_ROI_DX/2) > 0)? coordinate_x - (AEC_ROI_DX/2) : 0;
	y = (coordinate_y - (AEC_ROI_DY/2) > 0)? coordinate_y - (AEC_ROI_DY/2) : 0;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] x,y (%d, %d) base 320x240\n", x, y);

	//
	h_ratio = 2560/width;
	v_ratio = 1920/height;
	x = x*h_ratio + 49;
	y = y*v_ratio + 4;
	h = AEC_ROI_DX*h_ratio;
	v = AEC_ROI_DY*v_ratio;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] x,y,h,v (%d, %d, %d, %d) base 2560x1920\n", x, y, h, v);

#if 0
	// compensation h/v flip
	x = 2690 - x - h;
	y = 1952 - y - v;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] coordinate (%d, %d, %d, %d) after flip\n", x, y, h, v);
#endif

	ISX012MIPI_write_cmos_sensor1_16(0x6A50, x); // X position
	ISX012MIPI_write_cmos_sensor1_16(0x6A52, y); // Y position
	ISX012MIPI_write_cmos_sensor1_16(0x6A54, h); // H size
	ISX012MIPI_write_cmos_sensor1_16(0x6A56, v); //                                                             
	//
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AF_Window] Set focusing with 3A window data\n");
	ISX012MIPI_write_cmos_sensor1(0x6A80,0x00);  //                                                            
	ISX012MIPI_write_cmos_sensor1(0x6A81,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A82,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A83,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A84,0x08);
	ISX012MIPI_write_cmos_sensor1(0x6A85,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A86,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A87,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A88,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6A89,0x00);
	ISX012MIPI_write_cmos_sensor1(0x6646,0x08);

	// ISX012_AF_Init_Macro_Off setting
	ISX012MIPI_write_cmos_sensor1(0x6674,0x01);
}

static void ISX012_FOCUS_Set_AE_Window(UINT32 zone_addr)
{
	INT32 x0, y0, x1, y1, width, height, coordinate_x, coordinate_y;
	UINT32 *ptr = (UINT32*)zone_addr;
	INT32 h_ratio, v_ratio;
	INT32 x, y, h, v;

	x0 = *ptr;
	y0 = *(ptr + 1);
	x1 = *(ptr + 2);
	y1 = *(ptr + 3);
	width = *(ptr + 4);
	height = *(ptr + 5);
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] AE_Set_Window 3AWin: (%d,%d)~(%d,%d) w=%d h=%d\n",x0, y0, x1, y1, width, height);

	if (((x0==124) && (y0==84) && (x1==195) && (y1==155)) ||    // 4:3 default
	    ((x0==131) && (y0==84) && (x1==188) && (y1==155)))      // 16:9 default
	{
		ISX012MIPISensor.is_touch = false;
		return;
	}

	//                                                                           
	if ((x0==160) && (y0==120) && (x1==160) && (y1==120))
	{
		ISX012MIPI_write_cmos_sensor1(0x0188, 0x0);
		return;
	}
	//                                                                           

	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] AE_WIN set\n");
	ISX012MIPISensor.is_touch = true;

	// 320x240 base
	coordinate_x = (x0 + x1)/2;
	coordinate_y = (y0 + y1)/2;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] coordinate (%d, %d)\n", coordinate_x, coordinate_y);

#if 0
	// compensation h/v flip
	coordinate_x = width - coordinate_x;
	coordinate_y = height - coordinate_y;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] coordinate (%d, %d) after flip\n", coordinate_x, coordinate_y);
#endif

	// 320x240 base
	x = (coordinate_x - (AEC_ROI_DX/2) > 0)? coordinate_x - (AEC_ROI_DX/2) : 0;
	y = (coordinate_y - (AEC_ROI_DY/2) > 0)? coordinate_y - (AEC_ROI_DY/2) : 0;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] x,y (%d, %d) base 320x240\n", x, y);

	// 2560x1920 base
	h_ratio = 2560/width;
	v_ratio = 1920/height;
	x = x*h_ratio;
	y = y*v_ratio;
	h = AEC_ROI_DX*h_ratio;
	v = AEC_ROI_DY*v_ratio;
	ISX012MIPISENSORDB("[ISX012_FOCUS_Set_AE_Window] x,y,h,v (%d, %d, %d, %d) base 2560x1920\n", x, y, h, v);

	//                                                                    
	ISX012MIPI_write_cmos_sensor1_16(0x6A24, x); // HDELAY
	ISX012MIPI_write_cmos_sensor1_16(0x6A26, h); // HVALID
	ISX012MIPI_write_cmos_sensor1_16(0x6A28, y); // VDELAY
	ISX012MIPI_write_cmos_sensor1_16(0x6A2A, v); // VVALID
	//                                                                    

	ISX012MIPI_write_cmos_sensor1(0x0188, 0x1); // AE_FREE_WEIGHT_MODE
}

/*************************************************************************
* FUNCTION
*   ISX012WBcalibattion
* DESCRIPTION
*   color calibration
* PARAMETERS
*   None
* RETURNS
*   None
* GLOBALS AFFECTED
*************************************************************************/
static void ISX012WBcalibattion(kal_uint32 color_r_gain,kal_uint32 color_b_gain)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012WBcalibattion function:color_r_gain=%d,color_b_gain=%d\n",color_r_gain,color_b_gain);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012WBcalibattion function:\n ");
}
/*************************************************************************
* FUNCTION
*	ISX012MIPIOpen
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 ISX012MIPIOpen(void)
{
	volatile signed int i;
	kal_uint16 sensor_id = 0;
	kal_uint32 sensor_tmp=0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIOpen function:\n ");
	for(i=0;i<3;i++)
	{
		sensor_id  = (ISX012MIPIYUV_read_cmos_sensor(0x000E));
		sensor_tmp = sensor_id & 0x01;
		if(sensor_tmp)
			break;
	}
	if(!sensor_tmp)
	{
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	ISX012MIPIinitalvariable();
	ISX012MIPIInitialSetting();
	mDELAY(20);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIOpen function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIOpen() */

/*************************************************************************
* FUNCTION
*	ISX012MIPIClose
*
* DESCRIPTION
*	This function is to turn off sensor module power.
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 ISX012MIPIClose(void)
{
 	//CISModulePowerOn(FALSE);
	return ERROR_NONE;
}	/* ISX012MIPIClose() */
/*************************************************************************
* FUNCTION
*	ISX012MIPIPreview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void ISX012MIPI_set_iso(UINT16 para);
UINT32 ISX012MIPIPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	UINT32 len;

	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIPreview function:\n ");
	//                                                                 
	//Flash Reset
	ISX012MIPI_set_AE_mode(false); // AE Unlock
	if(ISX012MIPISensor.Flash_OnOff == 1)
	{
		len = sizeof(ISX012_Flash_OFF) / sizeof(ISX012_Flash_OFF[0]);
		ISX012_table_write_cmos_sensor(ISX012_Flash_OFF,len,ISX012MIPI_WRITE_ID1);

		if(ISX012MIPISensor.awbMode == AWB_MODE_AUTO)
		{
			ISX012MIPI_write_cmos_sensor1(0x0282,0x20);
		}
		// Vlatch On
		ISX012MIPI_write_cmos_sensor1(0x8800,0x1);
		ISX012MIPISensor.Flash_OnOff=0;
	}
	//                                                                 

	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.SensorMode=SENSOR_MODE_PREVIEW;
	spin_unlock(&isx012mipi_drv_lock);

	//                                                                  
	if (CurrentScenarioId == MSDK_SCENARIO_ID_CAMERA_PREVIEW)
	{
		len = sizeof(ISX012_Camcorder_Mode_OFF) / sizeof(ISX012_Camcorder_Mode_OFF[0]);
		ISX012_table_write_cmos_sensor(ISX012_Camcorder_Mode_OFF,len,ISX012MIPI_WRITE_ID1);

		//Preview SizeSetting
		ISX012MIPI_write_cmos_sensor1_16(0x0090, 0x0320); // 800
		ISX012MIPI_write_cmos_sensor1_16(0x0096, 0x0258); // 600

		//Preview Mode
		ISX012MIPI_write_cmos_sensor1(0x0089, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x0083, 0x01);
		ISX012MIPI_write_cmos_sensor1(0x0086, 0x02);
		ISX012MIPI_write_cmos_sensor1(0x0012, 0xFF);
		ISX012MIPI_write_cmos_sensor1(0x00F7, 0x52);
		ISX012MIPI_write_cmos_sensor1(0x00F8, 0x59);
		ISX012MIPI_write_cmos_sensor1(0x00F9, 0x5F);
		ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x0082, 0x01);
		mDELAY(30);

		// CANCEL AF
		ISX012MIPI_write_cmos_sensor1(0x6674, 0x00);      // AF_MONICHG_MOVE_F
		ISX012MIPI_write_cmos_sensor1(0x0082, 0x01);      // MONI_REFRESH
		ISX012MIPI_write_cmos_sensor1(0x00B2,0x02);       // AFMODE_MONI : Manual AF mode
		ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);      // MODESEL : Monitoring mode
		ISX012MIPI_write_cmos_sensor1_16(0x6648, 0x00C8); // AF_MANUAL_POS : MANUA AF search start position
		ISX012MIPI_write_cmos_sensor1(0x00B1, 0x01);      // AF_RESTART_F

		//ISX012_AF_Macro_OFF
		ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);
		ISX012MIPI_write_cmos_sensor1_16(0x6648, 0x00C8);
		ISX012MIPI_write_cmos_sensor1_16(0x66DC, 0x02A8);
		ISX012MIPI_write_cmos_sensor1_16(0x665A, 0x00C8);
		ISX012MIPI_write_cmos_sensor1(0x028E, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x00B3, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x00B2, 0x02);

	}
	else // camcorder
	{
		len = sizeof(ISX012_Camcorder_Mode_ON) / sizeof(ISX012_Camcorder_Mode_ON[0]);
		ISX012_table_write_cmos_sensor(ISX012_Camcorder_Mode_ON,len,ISX012MIPI_WRITE_ID1);

		//Preview SizeSetting
		ISX012MIPI_write_cmos_sensor1_16(0x0090, 0x0320); // 800
		ISX012MIPI_write_cmos_sensor1_16(0x0096, 0x0258); // 600

		//Preview Mode
		ISX012MIPI_write_cmos_sensor1(0x0089, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x0083, 0x01);
		ISX012MIPI_write_cmos_sensor1(0x0086, 0x02);
		ISX012MIPI_write_cmos_sensor1(0x0012, 0xFF);
		ISX012MIPI_write_cmos_sensor1(0x00F7, 0x52);
		ISX012MIPI_write_cmos_sensor1(0x00F8, 0x59);
		ISX012MIPI_write_cmos_sensor1(0x00F9, 0x5F);
		ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x0082, 0x01);
		mDELAY(30);

		//                                                                                           
		len = sizeof(ISX012_CAF_setting) / sizeof(ISX012_CAF_setting[0]);
		ISX012_table_write_cmos_sensor(ISX012_CAF_setting,len,ISX012MIPI_WRITE_ID1);
		//                                                                                           

		//ISX012_AF_Macro_OFF for CAF
		ISX012MIPI_write_cmos_sensor1(0x0081, 0x00);
		ISX012MIPI_write_cmos_sensor1_16(0x6648, 0x00C8);
		ISX012MIPI_write_cmos_sensor1_16(0x66DC, 0x02A8);
		ISX012MIPI_write_cmos_sensor1_16(0x665A, 0x00C8);
		ISX012MIPI_write_cmos_sensor1(0x028E, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x00B3, 0x00);
		ISX012MIPI_write_cmos_sensor1(0x00B2, 0x01);      // for CAF (0x2-->0x1)

		ISX012MIPI_write_cmos_sensor1(0x00B1, 0x01);      // AF_RESTART_F
	}
	//                                                                  

	// AF Window reset
	len = sizeof(ISX012_AF_Window_Reset) / sizeof(ISX012_AF_Window_Reset[0]);
	ISX012_table_write_cmos_sensor(ISX012_AF_Window_Reset,len,ISX012MIPI_WRITE_ID1);

	// AE Window reset
	ISX012MIPI_write_cmos_sensor1(0x0188, 0x0);	// AE_FREE_WEIGHT_MODE

	return ERROR_NONE ;
}	/* ISX012MIPIPreview() */
BOOL ISX012MIPI_set_param_exposure_for_HDR(UINT16 para)
{
    kal_uint32 totalGain = 0, exposureTime = 0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_exposure_for_HDR function:para=%d,manualAEStart=%d\n",para,ISX012MIPISensor.manualAEStart);
    if (0 == ISX012MIPISensor.manualAEStart)
    {
        ISX012MIPI_set_AE_mode(KAL_FALSE);//Manual AE enable
        spin_lock(&isx012mipi_drv_lock);
        ISX012MIPISensor.manualAEStart = 1;
		spin_unlock(&isx012mipi_drv_lock);
    }
	totalGain = ISX012MIPISensor.currentAxDGain;
    exposureTime = ISX012MIPISensor.currentExposureTime;
	switch (para)
	{
		case AE_EV_COMP_20:	//+2 EV
		case AE_EV_COMP_10:	// +1 EV
			totalGain = totalGain<<1;
			exposureTime = exposureTime<<1;
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_20\n");
		break;
		case AE_EV_COMP_00:	// +0 EV
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_00\n");
		break;
		case AE_EV_COMP_n10:  // -1 EV
		case AE_EV_COMP_n20:  // -2 EV
			totalGain = totalGain >> 1;
			exposureTime = exposureTime >> 1;
			ISX012MIPISENSORDB("[ISX012MIPI] HDR AE_EV_COMP_n20\n");
			break;
		default:
		break;//return FALSE;
	}
    ISX012MIPIWriteSensorGain(totalGain);
	ISX012MIPIWriteShutter(exposureTime);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_exposure_for_HDR function:\n ");
	return TRUE;
}
static void ISX012MIPI_FLASH_CAPTURE(void)
{
	unsigned char AWBSTS;
	int timeout_cnt=0;

	mDELAY(210);

#if 0
	while(1)
	{
		AWBSTS = ISX012MIPIYUV_read_cmos_sensor1(0x8A24);
		if( (AWBSTS == 0x2) || (AWBSTS == 0x04) || (AWBSTS == 0x6) )
		{
			break;
		}
		if(timeout_cnt >= 20)
		{
			ISX012MIPISENSORDB("[ISX012MIPI][ERROR] AWBSTS time out.\n");
			break;
		}
		msleep(10);
		timeout_cnt++;
	}
#endif
}

UINT32 ISX012MIPICapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.SensorMode=SENSOR_MODE_CAPTURE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x0081,0x02);
	//Capture SizeSetting
	ISX012MIPI_write_cmos_sensor1_16(0x0092, 0x0A00);
	ISX012MIPI_write_cmos_sensor1_16(0x0098, 0x0780);
	//Capture Mode
	ISX012MIPI_write_cmos_sensor1(0x008A, 0x00);
	ISX012MIPI_write_cmos_sensor1(0x0084, 0x00);
	ISX012MIPI_write_cmos_sensor1(0x0087, 0x03);
	ISX012MIPI_write_cmos_sensor1(0x0012, 0xFF);
	ISX012MIPI_write_cmos_sensor1(0x0081, 0x02);
	ISX012MIPI_write_cmos_sensor1(0x0082, 0x01);
	mDELAY(66);
	//ISX012MIPISetHVMirror(IMAGE_HV_MIRROR,SENSOR_MODE_CAPTURE);
	if(ISX012MIPISensor.Flash_OnOff == 1)
	{
		ISX012MIPI_WAIT_CM();
	}
	return ERROR_NONE;
}/* ISX012MIPICapture() */

UINT32 ISX012MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetResolution function:\n ");
	//                                                                                                
	pSensorResolution->SensorPreviewWidth=  ISX012MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight= ISX012MIPI_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorFullWidth= ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
	pSensorResolution->SensorFullHeight= ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
	pSensorResolution->SensorVideoWidth= ISX012MIPI_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorVideoHeight= ISX012MIPI_IMAGE_SENSOR_PV_HEIGHT;
	//                                                                                                
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetResolution function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIGetResolution() */

UINT32 ISX012MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,MSDK_SENSOR_INFO_STRUCT *pSensorInfo,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetInfo function:ScenarioId=%d\n",ScenarioId);
	//                                                                                                
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			pSensorInfo->SensorPreviewResolutionY=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=7.5;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX=ISX012MIPI_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY=ISX012MIPI_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}
	pSensorInfo->SensorFullResolutionX= ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
	pSensorInfo->SensorFullResolutionY= ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
	//                                                                                                
	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=5;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;
	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=4;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_UYVY;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorInterruptDelayLines = 2;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
	//                                                                                                                              
	pSensorInfo->CaptureDelayFrame = ISX012MIPISensor.Flash_OnOff ? 2 : 0; //                                                                              
	//pSensorInfo->CaptureDelayFrame = 1;
	//                                                                                                                              
	pSensorInfo->PreviewDelayFrame = 1;
	pSensorInfo->VideoDelayFrame = 0;
	pSensorInfo->SensorMasterClockSwitch = 0;
	pSensorInfo->YUVAwbDelayFrame = 2;
	pSensorInfo->YUVEffectDelayFrame= 2;
	pSensorInfo->AEShutDelayFrame= 0;
 	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_PV_GRAB_START_X;
			pSensorInfo->SensorGrabStartY = ISX012MIPI_PV_GRAB_START_Y;
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	5;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_FULL_GRAB_START_X;
			pSensorInfo->SensorGrabStartY = ISX012MIPI_FULL_GRAB_START_Y;
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount =0x1A;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=5;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = ISX012MIPI_PV_GRAB_START_X;
			pSensorInfo->SensorGrabStartY = ISX012MIPI_PV_GRAB_START_Y;
			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
			pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 0x1A;
			pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
			pSensorInfo->SensorWidthSampling = 0;
			pSensorInfo->SensorHightSampling = 0;
			pSensorInfo->SensorPacketECCOrder = 1;
		  break;
	}
	memcpy(pSensorConfigData, &ISX012MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetInfo function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIGetInfo() */

UINT32 ISX012MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	  ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIControl function:ScenarioId=%d\n",ScenarioId);
	  spin_lock(&isx012mipi_drv_lock);
	  CurrentScenarioId = ScenarioId;
	  spin_unlock(&isx012mipi_drv_lock);
	  switch (ScenarioId)
	  {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 ISX012MIPIPreview(pImageWindow, pSensorConfigData);
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 ISX012MIPICapture(pImageWindow, pSensorConfigData);
	  	     break;
		default:
			return ERROR_INVALID_SCENARIO_ID;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIControl function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIControl() */

/* [TC] YUV sensor */

BOOL ISX012MIPI_set_param_wb(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_wb function:para=%d\n",para);
	spin_lock(&isx012mipi_drv_lock);
    ISX012MIPISensor.awbMode = para;
    spin_unlock(&isx012mipi_drv_lock);
	switch (para)
    {
        case AWB_MODE_OFF:
							spin_lock(&isx012mipi_drv_lock);
							ISX012MIPI_AWB_ENABLE = KAL_FALSE;
							spin_unlock(&isx012mipi_drv_lock);
							ISX012MIPI_set_AWB_mode(ISX012MIPI_AWB_ENABLE);
							break;
        case AWB_MODE_AUTO: //auto
							spin_lock(&isx012mipi_drv_lock);
							ISX012MIPI_AWB_ENABLE = KAL_TRUE;
							spin_unlock(&isx012mipi_drv_lock);
							ISX012MIPI_set_AWB_mode(ISX012MIPI_AWB_ENABLE);
							ISX012MIPI_write_cmos_sensor1(0x0282,0x20);
							break;
        case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
							ISX012MIPI_write_cmos_sensor1(0x0282,0x26);
							break;
        case AWB_MODE_DAYLIGHT: //sunny
							ISX012MIPI_write_cmos_sensor1(0x0282,0x25);
							break;
        case AWB_MODE_FLUORESCENT:
							ISX012MIPI_write_cmos_sensor1(0x0282,0x27);
							break;
        case AWB_MODE_INCANDESCENT:
							ISX012MIPI_write_cmos_sensor1(0x0282,0x28);
							break;
#if 0
        case AWB_MODE_TUNGSTEN:
							ISX012MIPI_write_cmos_sensor1(0x0282,0x28);
							break;
#endif
        default:
			return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_wb function:\n ");
    return TRUE;
} /* ISX012MIPI_set_param_wb */
void ISX012MIPI_set_contrast(UINT16 para)
{
#if 0
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_contrast function:\n ");
    switch (para)
    {
        case ISP_CONTRAST_LOW:
             ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x14);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x14);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_HIGH:
             ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x2c);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x1c);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
             break;
        case ISP_CONTRAST_MIDDLE:
			 ISX012MIPI_write_cmos_sensor(0x3212,0x03);
			 ISX012MIPI_write_cmos_sensor(0x5586,0x20);
			 ISX012MIPI_write_cmos_sensor(0x5585,0x00);
			 ISX012MIPI_write_cmos_sensor(0x3212,0x13);
			 ISX012MIPI_write_cmos_sensor(0x3212,0xa3);
			 break;
        default:
             break;
    }
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_contrast function:\n ");
#endif
    return;
}

void ISX012MIPI_set_brightness(UINT16 para)
{
#if 0
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_brightness function:para=%d\n",para);
    switch (para)
    {
        case ISP_BRIGHT_LOW:
             ISX012MIPI_write_cmos_sensor1(0x0180, 0x01);
             break;
        case ISP_BRIGHT_HIGH:
             ISX012MIPI_write_cmos_sensor1(0x0180, 0x03);
             break;
        case ISP_BRIGHT_MIDDLE:
			 ISX012MIPI_write_cmos_sensor1(0x0180, 0x06);
			 break;
        default:
             return ;
             break;
    }
    ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_brightness function:\n ");
#endif
    return;
}
void ISX012MIPI_set_saturation(UINT16 para)
{
#if 0
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_saturation function:para=%d\n",para);
    switch (para)
    {
        case ISP_SAT_HIGH:
			break;
        case ISP_SAT_LOW:
             break;
        case ISP_SAT_MIDDLE:
			 break;
        default:
			 break;
    }
	mDELAY(50);
#endif
     return;
}
void ISX012MIPI_scene_mode_PORTRAIT(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SPORTS function:\n ");

	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1 : Auto
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG3
	ISX012MIPI_write_cmos_sensor1(0x038F,0x50);	 //PICT1_SN1 :
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);   // AF_SEARCH_OFFSE
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);   // AF_SEARCH_OFFSE
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	 // FAST_SHT_MODE_SE
	ISX012MIPI_write_cmos_sensor1(0x0280,0x00);	 //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SPORTS function:\n ");
}

void ISX012MIPI_scene_mode_LANDSCAPE(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_LANDSCAPE function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 :
	ISX012MIPI_write_cmos_sensor1(0x039F,0x9E);	  //UISATURATION
	ISX012MIPI_write_cmos_sensor1(0x03A3,0x2C);	  //UISHARPNESS_
	ISX012MIPI_write_cmos_sensor1(0x03A6,0x2C);	  //UISHARPNESS_
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 :
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x01);	  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_LANDSCAPE function:\n ");
}
void ISX012MIPI_scene_mode_SUNSET(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SUNSET function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1
	ISX012MIPI_write_cmos_sensor1(0x0287,0x25);	 //AWB_SN6 :
	ISX012MIPI_write_cmos_sensor1(0x0394,0x00);	 //PICT1_SN6
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	 //PICT1_SN1
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);	  // AF_S
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);	  // AF_S
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x05);  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SUNSET function:\n ");
}
void ISX012MIPI_scene_mode_SPORTS(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_SPORTS function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 :
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 :
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);    // AF_SEA
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x02);	  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_SPORTS function:\n ");
}
void ISX012MIPI_scene_mode_OFF(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_scene_mode_OFF function:\n ");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_FALSE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	  //ISO_TYPE1 : Au
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	  //SHTCTRLMAG3
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	  //PICT1_SN1 :
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);	// AF_SEARCH_O
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);	// AF_SEARCH_O
	ISX012MIPI_write_cmos_sensor1(0x500B,0x01);	  // FAST_SHT_MODE
	ISX012MIPI_write_cmos_sensor1(0x0280,0x00);	  //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_scene_mode_OFF function:\n ");
}
void ISX012MIPI_scene_mode_night(void)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter SCENE_MODE_NIGHTSCENE function\n");
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.NightMode=KAL_TRUE;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPI_write_cmos_sensor1(0x02A8,0x00);	 //ISO_TYPE1 :
	ISX012MIPI_write_cmos_sensor1(0x5E06,0x02);	 //SHTCTRLMAG3
	ISX012MIPI_write_cmos_sensor1(0x038F,0x00);	 //PICT1_SN1 :
	ISX012MIPI_write_cmos_sensor1_16(0x6742,0x0012);	  // AF_SEA
	ISX012MIPI_write_cmos_sensor1_16(0x6744,0x0006);	  // AF_SEA
	ISX012MIPI_write_cmos_sensor1(0x500B,0x00);	 // FAST_SHT_MO
	ISX012MIPI_write_cmos_sensor1(0x0280,0x07);	 //SCENE_SELECT
	ISX012MIPISENSORDB("[ISX012MIPI]exit SCENE_MODE_NIGHTSCENE function\n");
}

void ISX012MIPI_set_scene_mode(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_scene_mode function:para=%d\n",para);
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.sceneMode=para;
	spin_unlock(&isx012mipi_drv_lock);
    switch (para)
    {
		case SCENE_MODE_NIGHTSCENE:
			 ISX012MIPI_scene_mode_night();
			break;
        case SCENE_MODE_PORTRAIT:
			ISX012MIPI_scene_mode_PORTRAIT();
             break;
        case SCENE_MODE_LANDSCAPE:
			ISX012MIPI_scene_mode_LANDSCAPE();
             break;
        case SCENE_MODE_SUNSET:
			ISX012MIPI_scene_mode_SUNSET();
            break;
        case SCENE_MODE_SPORTS:
            ISX012MIPI_scene_mode_SPORTS();
            break;
        case SCENE_MODE_HDR:
            if (1 == ISX012MIPISensor.manualAEStart)
            {
                ISX012MIPI_set_AE_mode(KAL_TRUE);//Manual AE disable
                spin_lock(&isx012mipi_drv_lock);
            	ISX012MIPISensor.manualAEStart = 0;
                ISX012MIPISensor.currentExposureTime = 0;
                ISX012MIPISensor.currentAxDGain = 0;
				spin_unlock(&isx012mipi_drv_lock);
            }
            break;
        case SCENE_MODE_OFF:
        default:
			ISX012MIPI_scene_mode_OFF();
            break;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_scene_mode function:\n ");
	return;
}
void ISX012MIPI_set_iso(UINT16 para)
{
    spin_lock(&isx012mipi_drv_lock);
    ISX012MIPISensor.isoSpeed = para;
    spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_iso function:para=%d\n",para);
    switch (para)
    {
        case AE_ISO_100:
			ISX012MIPI_write_cmos_sensor1(0x02A8, 0x07);
			ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
			break;
        case AE_ISO_AUTO:
			ISX012MIPI_write_cmos_sensor1(0x02A8, 0x00);
			ISX012MIPI_write_cmos_sensor1(0x0362, 0x55);
			ISX012MIPI_write_cmos_sensor1(0x0365, 0x55);
			break;
        case AE_ISO_200:
			ISX012MIPI_write_cmos_sensor1(0x02A8, 0x0A);
			ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
			break;
        case AE_ISO_400:
			ISX012MIPI_write_cmos_sensor1(0x02A8, 0x0D);
			ISX012MIPI_write_cmos_sensor1(0x0362, 0x57);
			ISX012MIPI_write_cmos_sensor1(0x0365, 0x57);
			break;
        case AE_ISO_800:
			ISX012MIPI_write_cmos_sensor1(0x02A8,0x10);
			ISX012MIPI_write_cmos_sensor1(0x0362,0x57);
			ISX012MIPI_write_cmos_sensor1(0x0365,0x57);
			break;
        default:
			break;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_iso function:para=%d\n",para);
    return;
}

BOOL ISX012MIPI_set_param_effect(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_effect function:para=%d\n ",para);
	switch (para)
    {
        case MEFFECT_OFF:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x00);
	         break;
	case MEFFECT_MONO:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x04);
	         break;
	case MEFFECT_NEGATIVE:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x02);
			 break;
        case MEFFECT_SEPIA:
			 ISX012MIPI_write_cmos_sensor1(0x01c5,0x03);
			 break;
        default:
             return KAL_FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_effect function:\n ");
    return KAL_FALSE;
} /* ISX012MIPI_set_param_effect */

BOOL ISX012MIPI_set_param_banding(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_banding function:\n ");
	/*
	switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
						spin_lock(&isx012mipi_drv_lock);
						ISX012MIPI_Banding_setting = AE_FLICKER_MODE_50HZ;
						spin_unlock(&isx012mipi_drv_lock);
						ISX012MIPI_write_cmos_sensor(0x3c00,0x04);
						ISX012MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        case AE_FLICKER_MODE_60HZ:
						spin_lock(&isx012mipi_drv_lock);
						ISX012MIPI_Banding_setting = AE_FLICKER_MODE_60HZ;
						spin_unlock(&isx012mipi_drv_lock);
						ISX012MIPI_write_cmos_sensor(0x3c00,0x00);
						ISX012MIPI_write_cmos_sensor(0x3c01,0x80);
            			break;
        default:
             return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_banding function:\n ");
	*/
    return TRUE;
} /* ISX012MIPI_set_param_banding */

BOOL ISX012MIPI_set_param_exposure(UINT16 para)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_set_param_exposure function:\n ");
	ISX012MIPISENSORDB("[ISX012MIPI]para=%d:\n",para);
	ISX012MIPISensor.exposure_value = para;
    if (SCENE_MODE_HDR == ISX012MIPISensor.sceneMode &&
    SENSOR_MODE_CAPTURE == ISX012MIPISensor.SensorMode)
    {
       ISX012MIPI_set_param_exposure_for_HDR(para);
       return TRUE;
    }
	switch (para)
    {
		case AE_EV_COMP_30:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0x05);//	EVSEL
			break;
		case AE_EV_COMP_20:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0x04);//	EVSEL
			break;
		case AE_EV_COMP_10:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0x02);//	EVSEL
			break;
		case AE_EV_COMP_00:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0x00);//	EVSEL
			break;
		case AE_EV_COMP_n10:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0xfe);//	EVSEL
			break;
		case AE_EV_COMP_n20:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0xfc);//	EVSEL
			break;
		case AE_EV_COMP_n30:
			ISX012MIPI_write_cmos_sensor1(0x0180, 0xfb);//	EVSEL
			break;
        default:
			return FALSE;
    }
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_set_param_exposure function:\n ");
    return TRUE;
} /* ISX012MIPI_set_param_exposure */

UINT32 ISX012MIPIYUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIYUVSensorSetting function:iCmd=%d,iPara=%d, %d \n",iCmd, iPara);
	switch (iCmd)
	{
		case FID_SCENE_MODE:
			ISX012MIPI_set_scene_mode(iPara);
	    	break;
		case FID_AWB_MODE:
			ISX012MIPI_set_param_wb(iPara);
			  break;
		case FID_COLOR_EFFECT:
			ISX012MIPI_set_param_effect(iPara);
		 	  break;
		case FID_AE_EV:
			ISX012MIPI_set_param_exposure(iPara);
		    break;
		case FID_AE_FLICKER:
			 ISX012MIPI_set_param_banding(iPara);
		 	 break;
		case FID_ISP_CONTRAST:
            ISX012MIPI_set_contrast(iPara);
            break;
        case FID_ISP_BRIGHT:
            ISX012MIPI_set_brightness(iPara);
            break;
        case FID_ISP_SAT:
            ISX012MIPI_set_saturation(iPara);
            break;
    	case FID_ZOOM_FACTOR:
			spin_lock(&isx012mipi_drv_lock);
	        zoom_factor = iPara;
			spin_unlock(&isx012mipi_drv_lock);
            break;
		case FID_AE_ISO:
			ISX012MIPISENSORDB("[ISX012MIPI]FID_AE_ISO:%d\n", iPara);
            ISX012MIPI_set_iso(iPara);
            break;
	  	default:
		    break;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIYUVSensorSetting function\n");
	return TRUE;
}   /* ISX012MIPIYUVSensorSetting */
void ISX012MIPIYUV_SetFrameRate(UINT16 u2FrameRate)
{
	/*
	if (u2FrameRate == 30)
       {
       }
	else if (u2FrameRate == 15)
	{
        }
	else
	{
	    printk("Wrong frame rate setting \n");
	}
    */
}

UINT32 ISX012MIPIYUVSetVideoMode(UINT16 u2FrameRate)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIYUVSetVideoMode function:u2FrameRate=%d\n",u2FrameRate);

	ISX012MIPI_write_cmos_sensor1_16(0x665A, 0x00D0); //                                                                                             

	//ISX012MIPISetHVMirror(IMAGE_HV_MIRROR,SENSOR_MODE_VIDEO);
	spin_lock(&isx012mipi_drv_lock);
	ISX012MIPISensor.video_mode =KAL_TRUE;
	ISX012MIPISensor.pv_mode=KAL_FALSE;
	ISX012MIPISensor.capture_mode=KAL_FALSE;
	ISX012MIPISensor.SensorMode=SENSOR_MODE_VIDEO;
	spin_unlock(&isx012mipi_drv_lock);
	ISX012MIPIYUV_SetFrameRate(u2FrameRate);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIYUVSetVideoMode function:\n ");
    return TRUE;
}

/************************************************************/
static void ISX012MIPIGetEvAwbRef(UINT32 pSensorAEAWBRefStruct)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetEvAwbRef function:\n ");
	/*
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	Ref->SensorAERef.AeRefLV05Shutter=0x170c;
	Ref->SensorAERef.AeRefLV05Gain=0x30;
	Ref->SensorAERef.AeRefLV13Shutter=0x24e;
	Ref->SensorAERef.AeRefLV13Gain=0x10;
	Ref->SensorAwbGainRef.AwbRefD65Rgain=0x610;
	Ref->SensorAwbGainRef.AwbRefD65Bgain=0x448;
	Ref->SensorAwbGainRef.AwbRefCWFRgain=0x4e0;
	Ref->SensorAwbGainRef.AwbRefCWFBgain=0x5a0;
	*/
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetEvAwbRef function:\n ");
}

static void ISX012MIPIGetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetCurAeAwbInfo function:\n ");
	/*
	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	Info->SensorAECur.AeCurShutter=ISX012MIPIReadShutter();
	Info->SensorAECur.AeCurGain=ISX012MIPIReadSensorGain() ;
	Info->SensorAwbGainCur.AwbCurRgain=((ISX012MIPIYUV_read_cmos_sensor(0x3401)&0xff)+((ISX012MIPIYUV_read_cmos_sensor(0x3400)&0xff)*256));
	Info->SensorAwbGainCur.AwbCurBgain=((ISX012MIPIYUV_read_cmos_sensor(0x3405)&0xff)+((ISX012MIPIYUV_read_cmos_sensor(0x3404)&0xff)*256));
	*/
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetCurAeAwbInfo function:\n ");
}
UINT32 ISX012MIPIMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
	{
        /*
		kal_uint32 pclk;
		kal_int16 dummyLine;
		kal_uint16 lineLength,frameHeight;
		ISX012MIPISENSORDB("ISX012MIPIMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
		ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIMaxFramerateByScenario function:\n ");
		switch (scenarioId) {
			case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
				pclk = 56000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_SVGA_WIDTH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_SVGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&isx012mipi_drv_lock);
				ISX012MIPISensor.SensorMode= SENSOR_MODE_PREVIEW;
				ISX012MIPISensor.pv_dummy_lines = dummyLine;
				spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(ISX012MIPISensor.PreviewDummyPixels, dummyLine);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				pclk = 56000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_VIDEO_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_VIDEO_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				//spin_lock(&isx012mipi_drv_lock);
				//ov8825.sensorMode = SENSOR_MODE_VIDEO;
				//spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(0, dummyLine);
				break;
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
				pclk = 90000000;
				lineLength = ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
				frameHeight = (10 * pclk)/frameRate/lineLength;
				dummyLine = frameHeight - ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
				if(dummyLine<0)
					dummyLine = 0;
				spin_lock(&isx012mipi_drv_lock);
				ISX012MIPISensor.cp_dummy_lines = dummyLine;
				ISX012MIPISensor.SensorMode= SENSOR_MODE_CAPTURE;
				spin_unlock(&isx012mipi_drv_lock);
				//ISX012MIPISetDummy(ISX012MIPISensor.CaptureDummyPixels, dummyLine);
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
				break;
			case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
				break;
			default:
				break;
		}
		ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIMaxFramerateByScenario function:\n ");
		*/
		return ERROR_NONE;
	}
UINT32 ISX012MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPIGetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = 75;
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
			 *pframeRate = 300;
			break;
		default:
			break;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIGetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}
void ISX012MIPI_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	ISX012MIPISENSORDB("[ISX012MIPI]ISX012MIPI_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_get_AEAWB_lock function:\n ");
}
void ISX012MIPI_GetDelayInfo(UINT32 delayAddr)
{
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
    ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_GetDelayInfo function:\n ");
	pDelayInfo->InitDelay=0;
	pDelayInfo->EffectDelay=0;
	pDelayInfo->AwbDelay=0;
	pDelayInfo->AFSwitchDelayFrame=50;
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_GetDelayInfo function:\n ");
}
void ISX012MIPI_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
	/*
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAeLock = TRUE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AE_mode(KAL_FALSE);
      break;
      case SENSOR_3A_AE_UNLOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAeLock = FALSE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAwbLock = TRUE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          spin_lock(&isx012mipi_drv_lock);
          ISX012MIPISensor.userAskAwbLock = FALSE;
          spin_unlock(&isx012mipi_drv_lock);
          ISX012MIPI_set_AWB_mode_UNLOCK();
      break;
      default:
      	break;
   }
   */
   ISX012MIPISENSORDB("[ISX012MIPI]exit ACDK_SENSOR_3A_LOCK_ENUM function:action=%d\n",action);
   return;
}
//                                                                 
static void ISX012MIPI_calc_AEgain_offset()
{
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_calc_AEgain_offset function\n");
	short int aediff,aeoffset;

	kal_uint16 ae_now = ISX012MIPISensor.AE_now , ae_auto = ISX012MIPISensor.AE_auto;
	short int ersc_now = ISX012MIPISensor.ERSC_now, ersc_auto = ISX012MIPISensor.ERSC_auto;

	//AE_Gain_Offset = Target - ERRSCL_NOW
	//auto : preview, now : pre strobe
	aediff = (ae_now + ersc_now) - (ae_auto + ersc_auto);

	if(aediff < 0)
	{
		aediff = 0;
	}

	if(ersc_now < 0)
	{
		if(aediff >= AE_MAXDIFF)
		{
			aeoffset = -AE_OFSETVAL - ersc_now;
		}
		else
		{
			aeoffset = -aeoffset_table[aediff/10] - ersc_now;
		}
	}
	else
	{
		if(aediff >= AE_MAXDIFF)
		{
			aeoffset = -AE_OFSETVAL;
		}
		else
		{
			aeoffset = -aeoffset_table[aediff/10];
		}
	}

	//Sset AE Gain offset
	ISX012MIPI_write_cmos_sensor1_16(0x0186 , (kal_uint16)aeoffset);
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_calc_AEgain_offset function\n");
}
static void ISX012MIPI_Flash_Condition(unsigned int *pFeatureReturnPara32)
{
	*pFeatureReturnPara32=0;
	kal_uint8  user_gain_level_now = 0;
	kal_uint32 sht_time_out = 0;

	if(ISX012MIPISensor.isoSpeed == AE_ISO_AUTO)
	{ // check EV
		user_gain_level_now = ISX012MIPIYUV_read_cmos_sensor1(0x01A5);
		ISX012MIPISENSORDB("[ISX012MIPI]user_gain_level_now : %x\n",user_gain_level_now);
		if(((ISX012MIPISensor.exposure_value == AE_EV_COMP_30)  && (user_gain_level_now > 0x3E))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_20)  && (user_gain_level_now > 0x3B))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_10)  && (user_gain_level_now > 0x35))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_00)  && (user_gain_level_now > 0x2D))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_n10) && (user_gain_level_now > 0x23))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_n20) && (user_gain_level_now > 0x15))
			|| ((ISX012MIPISensor.exposure_value == AE_EV_COMP_n30) && (user_gain_level_now > 0x10)))
		{
			*pFeatureReturnPara32 = 1;
		}
	}
	else
	{ // check ISO
		sht_time_out = (ISX012MIPIYUV_read_cmos_sensor1_16(0x019E) << 16) + ISX012MIPIYUV_read_cmos_sensor1_16(0x019C);
		ISX012MIPISENSORDB("[ISX012MIPI]sht_time_out : %x\n",sht_time_out);
		if(((ISX012MIPISensor.isoSpeed == AE_ISO_100) && (sht_time_out > 0x9DBA))
			|| ((ISX012MIPISensor.isoSpeed == AE_ISO_200) && (sht_time_out > 0x864A))
			|| ((ISX012MIPISensor.isoSpeed == AE_ISO_400) && (sht_time_out > 0x738A))
			|| ((ISX012MIPISensor.isoSpeed == AE_ISO_800) && (sht_time_out > 0x738A)))
		{
			*pFeatureReturnPara32 = 1;
		}
	}
}

#define FLASH_BV_THRESHOLD 0x25
static void ISX012MIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{

	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_FlashTriggerCheck function. flash_mode=%d\n",ISX012MIPISensor.flash_mode);
	*pFeatureReturnPara32=0;
	kal_uint32 len=0;
	kal_uint16 ae_auto=0, ersc_auto=0;
	ISX012MIPISensor.Flash_OnOff = 0;

	if(ISX012MIPISensor.flash_mode == FLASH_MODE_FORCE_OFF)
	{
		return;
	}

	if(ISX012MIPISensor.flash_mode == FLASH_MODE_AUTO)
	{
		ISX012MIPI_Flash_Condition(pFeatureReturnPara32);
		if(*pFeatureReturnPara32 != 1){
			return;
		}
	}
	// Flash AE line write
	len = sizeof(ISX012_Flash_AELINE) / sizeof(ISX012_Flash_AELINE[0]);
	ISX012_table_write_cmos_sensor(ISX012_Flash_AELINE,len,ISX012MIPI_WRITE_ID1);

	mDELAY(120); // Wait 1V time 60ms

	// Preview AE Scale Read
	ISX012MIPISensor.AE_auto = ISX012MIPIYUV_read_cmos_sensor1_16(0x01CE);
	ISX012MIPISensor.ERSC_auto = ISX012MIPIYUV_read_cmos_sensor1_16(0x01CA);
	// AWB mode AUTO??
	if(ISX012MIPISensor.awbMode == AWB_MODE_AUTO)
	{
		if( ISX012MIPIYUV_read_cmos_sensor1(0x8a25) )
		{//1 outdoor
			ISX012MIPISENSORDB("[ISX012MIPI]Flash AWB outdoor!!\n");
			ISX012MIPI_write_cmos_sensor1(0x0282,0x10);
		}
		else
		{//0 indoor
			ISX012MIPISENSORDB("[ISX012MIPI]Flash AWB indoor!!\n");
			ISX012MIPI_write_cmos_sensor1(0x0282,0x00);
		}
	}
	len = sizeof(ISX012_Flash_ON) / sizeof(ISX012_Flash_ON[0]);
	ISX012_table_write_cmos_sensor(ISX012_Flash_ON,len,ISX012MIPI_WRITE_ID1);
	//ISX012_FOCUS_Single_Focus();

	// Vlatch On
	//ISX012MIPI_write_cmos_sensor1(0x8800,0x1);

	//mDELAY(40); // Wait 1V time 40ms
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_FlashTriggerCheck function\n");
	ISX012MIPISensor.Flash_OnOff = 1;
	ISX012MIPISensor.Flash_AE_Start = 2;
	return;
}

static void ISX012MIPI_FlashAE()
{
	UINT32 calc_ae_scl=0;
	ISX012MIPISENSORDB("[ISX012MIPI]enter ISX012MIPI_FlashAE function\n");
	//                                                                               
	if(ISX012MIPISensor.Flash_AE_Start == 2)
	{
		// Read data for Flash AE
		ISX012MIPISensor.ERSC_now = ISX012MIPIYUV_read_cmos_sensor1_16(0x01CC);
		ISX012MIPISensor.AE_now = ISX012MIPIYUV_read_cmos_sensor1_16(0x01D0);
		ISX012MIPISensor.AE_scl = ISX012MIPIYUV_read_cmos_sensor1_16(0x8BC0);
		ISX012MIPI_set_AE_mode(true); //AE Lock
		ISX012MIPI_write_cmos_sensor1(0x01AE,0x00); //Half AWB Lock
		ISX012MIPISensor.Flash_AE_Start = 0;
	}
	//                                                                               
	calc_ae_scl = ISX012MIPISensor.AE_scl - 4802;
	ISX012MIPI_write_cmos_sensor1_16(0x5E02,calc_ae_scl);

	ISX012MIPI_calc_AEgain_offset(); // Set AE Gain offset
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPI_FlashAE function\n");
}
//                                                                 

UINT32 ISX012MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
	UINT32 Tony_Temp1 = 0;
	UINT32 Tony_Temp2 = 0;
	Tony_Temp1 = pFeaturePara[0];
	Tony_Temp2 = pFeaturePara[1];
	ISX012MIPISENSORDB("[ISX012MIPI]enter[ISX012MIPIFeatureControl]feature id=%d \n",FeatureId);
	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH;
			*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_QSXGA_WITDH + ISX012MIPISensor.cp_dummy_pixels;
					*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_QSXGA_HEIGHT + ISX012MIPISensor.cp_dummy_lines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=ISX012MIPI_IMAGE_SENSOR_PV_WIDTH + ISX012MIPISensor.pv_dummy_pixels;
					*pFeatureReturnPara16=ISX012MIPI_IMAGE_SENSOR_PV_HEIGHT  + ISX012MIPISensor.pv_dummy_lines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = ISX012MIPISensor.VideoPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = ISX012MIPISensor.PreviewPclk * 1000 *100;	 //unit: Hz
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			break;
		/**********************Strobe Ctrl Start *******************************/
		case SENSOR_FEATURE_SET_ESHUTTER:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_ESHUTTER: Not Support\n");
			break;
		case SENSOR_FEATURE_SET_GAIN:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_GAIN: Not Support\n");
			break;
		case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
			ISX012MIPI_FlashAE(); //                                                               
			ISX012MIPISENSORDB("[ISX012MIPI] F_GET_AE_FLASHLIGHT_INFO: Not Support\n");
			break;

		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
			ISX012MIPI_FlashTriggerCheck(pFeatureData32);
			ISX012MIPISENSORDB("[ISX012MIPI] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
			break;

		case SENSOR_FEATURE_SET_FLASHLIGHT:
			ISX012MIPISensor.flash_mode = *pFeatureData16; //                                                               
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_FLASHLIGHT: %d\n", *pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_FLASH_CAPTURE:
			ISX012MIPI_FLASH_CAPTURE();
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_FLASH_CAPTURE\n");
			break;
		case SENSOR_FEATURE_GET_AUTO_FLASHLIGHT_CONDITION:
			ISX012MIPI_Flash_Condition(pFeatureData32);
			ISX012MIPISENSORDB("[ISX012MIPI] F_GET_FLASH_CONDITION : %d\n",*pFeatureData16);
			break;
		/**********************Strobe Ctrl End *******************************/

		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			ISX012MIPISENSORDB("[ISX012MIPI] F_SET_ISP_MASTER_CLOCK_FREQ\n");
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			ISX012MIPI_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = ISX012MIPIYUV_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &ISX012MIPISensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
			*pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
			break;
		case SENSOR_FEATURE_SET_CCT_REGISTER:
		case SENSOR_FEATURE_GET_CCT_REGISTER:
		case SENSOR_FEATURE_SET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_ENG_REGISTER:
		case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
		case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
		case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
		case SENSOR_FEATURE_GET_GROUP_INFO:
		case SENSOR_FEATURE_GET_ITEM_INFO:
		case SENSOR_FEATURE_SET_ITEM_INFO:
		case SENSOR_FEATURE_GET_ENG_INFO:
			break;
		case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=0;
            *pFeatureParaLen=4;
		    break;
		case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			ISX012MIPIYUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
            ISX012MIPI_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
            break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_VIDEO_MODE\n");
		    ISX012MIPIYUVSetVideoMode(*pFeatureData16);
		    break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CHECK_SENSOR_ID\n");
			ISX012MIPI_GetSensorID(pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_EV_AWB_REF\n");
			ISX012MIPIGetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN\n");
			ISX012MIPIGetCurAeAwbInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_EXIF_INFO\n");
            ISX012MIPIGetExifInfo(*pFeatureData32);
            break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_DELAY_INFO\n");
			ISX012MIPI_GetDelayInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_SET_SLAVE_I2C_ID:
             ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_SLAVE_I2C_ID\n");
             ISX012MIPI_sensor_socket = *pFeatureData32;
             break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_TEST_PATTERN\n");
			ISX012SetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			ISX012MIPISENSORDB("[ISX012MIPI]ISX012_TEST_PATTERN_CHECKSUM\n");
			*pFeatureReturnPara32=ISX012_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO\n");
			ISX012MIPIMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:\
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO\n");
			ISX012MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
	    /**********************below is AF control**********************/
		case SENSOR_FEATURE_INITIALIZE_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_INITIALIZE_AF\n");
			//ISX012_FOCUS_OVT_AFC_Init();
            break;
		case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
			ISX012_FOCUS_Move_to(*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_AF_STATUS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
            ISX012_FOCUS_Get_AF_Status(pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
			ISX012_FOCUS_Single_Focus();
            break;
		case SENSOR_FEATURE_CONSTANT_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CONSTANT_AF\n");
			//ISX012_FOCUS_OVT_AFC_Constant_Focus();
			break;
		case SENSOR_FEATURE_CANCEL_AF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_CANCEL_AF\n");
           // ISX012_FOCUS_OVT_AFC_Cancel_Focus();
            break;
		case SENSOR_FEATURE_GET_AF_INF:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_INF\n");
            ISX012_FOCUS_Get_AF_Inf(pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_GET_AF_MACRO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_MACRO\n");
            ISX012_FOCUS_Get_AF_Macro(pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_SET_AF_WINDOW:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_SET_AF_WINDOW\n");
			ISX012_FOCUS_Set_AF_Window(*pFeatureData32);
            break;
        case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
            ISX012_FOCUS_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			ISX012MIPISENSORDB("[ISX012MIPI]SENSOR_FEATURE_GET_AF_STATUS\n");
			ISX012MIPI_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;

        case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			ISX012MIPISENSORDB("[ISX012MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
            ISX012_FOCUS_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_SET_AE_WINDOW:
            ISX012MIPISENSORDB("[ISX012MIPI]AE zone addr = 0x%x\n",*pFeatureData32);
            ISX012_FOCUS_Set_AE_Window(*pFeatureData32);
            break;

		//                                                                 
		case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
			{
				UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
				UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

				ISX012MIPISENSORDB("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
				*pHorFOV = 56;  // HorFOV = 55.7
				*pVerFOV = 42;  // VerFOV = 42.2
				*pFeatureParaLen = 8;
			}
			break;
		//                                                                 

		default:
			break;
	}
	ISX012MIPISENSORDB("[ISX012MIPI]exit ISX012MIPIFeatureControl function:\n ");
	return ERROR_NONE;
}	/* ISX012MIPIFeatureControl() */

SENSOR_FUNCTION_STRUCT	SensorFuncISX012MIPI=
{
	ISX012MIPIOpen,
	ISX012MIPIGetInfo,
	ISX012MIPIGetResolution,
	ISX012MIPIFeatureControl,
	ISX012MIPIControl,
	ISX012MIPIClose
};

UINT32 ISX012_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncISX012MIPI;
	return ERROR_NONE;
}	/* SensorInit() */



