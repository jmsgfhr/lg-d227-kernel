
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
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
/*****************************************************************************
 *
 *
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
#include <linux/slab.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "t4k28yuv_Sensor.h"
#include "t4k28yuv_Camera_Sensor_para.h"
#include "t4k28yuv_CameraCustomized.h"

#include "kd_camera_feature.h"
//#include "parser.h"

kal_bool parser_enable = KAL_FALSE;
kal_bool  T4K28YUV_MPEG4_encode_mode = KAL_FALSE;
kal_uint16  T4K28YUV_sensor_gain_base = 0x0;
/* MAX/MIN Explosure Lines Used By AE Algorithm */
kal_uint16 T4K28YUV_MAX_EXPOSURE_LINES = T4K28_PV_PERIOD_LINE_NUMS-4;
kal_uint8  T4K28YUV_MIN_EXPOSURE_LINES = 2;
kal_uint32 T4K28YUV_isp_master_clock;
kal_uint16 T4K28YUV_CURRENT_FRAME_LINES = T4K28_PV_PERIOD_LINE_NUMS;

static kal_uint16 T4K28YUV_dummy_pixels=0, T4K28YUV_dummy_lines=0;
kal_uint16 T4K28YUV_PV_dummy_pixels=0,T4K28YUV_PV_dummy_lines=0;

kal_uint8 T4K28YUV_sensor_write_I2C_address = T4K28_WRITE_ID;
kal_uint8 T4K28YUV_sensor_read_I2C_address = T4K28_READ_ID;

static kal_uint32 T4K28YUV_zoom_factor = 0;

kal_uint32 set_pv_back_es = 0xff;
kal_uint32 set_pv_back_ag = 0x20;
kal_uint32 set_pv_back_dg = 0x10;

#define LOG_TAG "[T4K28Yuv]"
#define SENSORDB(fmt, arg...) printk( LOG_TAG  fmt, ##arg)
#define RETAILMSG(x,...)
#define TEXT

static MSDK_SCENARIO_ID_ENUM CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

kal_uint16 T4K28YUV_g_iDummyLines = 28;
//XB.PANG NEED CHECK
struct
{
  kal_bool    NightMode;
  kal_uint8   ZoomFactor; /* Zoom Index */
  kal_uint16  Banding;
  kal_uint32  PvShutter;
  kal_uint32  PvDummyPixels;
  kal_uint32  PvDummyLines;
  kal_uint32  CapDummyPixels;
  kal_uint32  CapDummyLines;
  kal_uint32  PvOpClk;
  kal_uint32  CapOpClk;

  kal_uint8   Effect;
  kal_uint8   Brightness;
  kal_uint32  sceneMode;//FOR HDR
  kal_uint16  SensorMode;

  kal_uint8   isoSpeed;	//                                                                                      
} t4k28yuv_status;

//static struct T4K28YUVStatus t4k28yuv_status;
static DEFINE_SPINLOCK(T4K28_drv_lock);

UINT8 T4K28YUVPixelClockDivider=0;
kal_uint32 T4K28YUV_sensor_pclk=52000000;;
kal_uint32 T4K28YUV_PV_pclk = 5525;

kal_uint32 T4K28YUV_CAP_pclk = 6175;

kal_uint16 T4K28YUV_pv_exposure_lines=0x100, T4K28YUV_g_iBackupExtraExp = 0, T4K28YUV_extra_exposure_lines = 0;

kal_uint16 T4K28YUV_sensor_id=0;

MSDK_SENSOR_CONFIG_STRUCT T4K28YUVSensorConfigData;

kal_uint32 T4K28YUV_FAC_SENSOR_REG;
kal_uint16 T4K28YUV_sensor_flip_value;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT T4K28YUVSensorCCT[] = CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT T4K28YUVSensorReg[ENGINEER_END] = CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

typedef enum
{
  T4K28_720P,
  T4K28_2M,
} T4K28_RES_TYPE;

T4K28_RES_TYPE T4K28YUV_g_RES = T4K28_720P;

typedef enum
{
  T4K28_MODE_PREVIEW,
  T4K28_MODE_CAPTURE,
} T4K28_MODE;
T4K28_MODE g_iT4K28YUV_Mode = T4K28_MODE_PREVIEW;

typedef enum
{
  AE_enable,
  AE_lock,
  AE_unlock,
  AE_disable,
} AE_status;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;

//for burst mode
#define I2C_BUFFER_LEN 254 //MAX data to send by MT6572 i2c dma mode is 255 bytes
#define BLOCK_I2C_DATA_WRITE iBurstWriteReg

#if 0 // test
#define T4K28YUV_write_cmos_sensor(addr, para) {  SENSORDB("[%#x] %#x\n",(u16)addr, (u16)para);  iWriteReg((u16)addr , (u32)para , 1, T4K28_WRITE_ID);  }
#else
#define T4K28YUV_write_cmos_sensor(addr, para) iWriteReg((u16)addr , (u32)para , 1, T4K28_WRITE_ID)
#endif

static int T4K28_table_write_cmos_sensor(t4k28_short_t* para, kal_uint32 len, kal_uint32 slave_addr)
{
	t4k28_short_t* pPara = (t4k28_short_t*) para;
	kal_uint8 puSendCmd[I2C_BUFFER_LEN]={0,};
	kal_uint32 tosend=0, IDX=0;
	kal_uint16 addr, addr_next, data;

	if(pPara == NULL)
	{
		SENSORDB("[ISX012MIPI] ERROR!! pPara is Null!!\n");
		return 0;
	}

	while(IDX < len)
	{
		addr = pPara->address;

		if (tosend == 0) // new (addr, data) to send
		{
			puSendCmd[tosend++] = (kal_uint8)(addr >> 8) & 0xff;
			puSendCmd[tosend++] = (kal_uint8)(addr & 0xff);

			data = pPara->data;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 1;
			IDX += 1;
			pPara++;
		}
		else if (addr == addr_next) // to multiple write the data to the incremental address
		{
			data = pPara->data;
			puSendCmd[tosend++] = (kal_uint8)data;
			addr_next = addr + 1;
			IDX += 1;
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

static UINT32 g_sensorAfStatus = 0;

#define PROFILE 1

#if PROFILE
static struct timeval T4K28YUV_ktv1, T4K28YUV_ktv2;
inline void T4K28YUV_imgSensorProfileStart(void)
{
    do_gettimeofday(&T4K28YUV_ktv1);
}

inline void T4K28YUV_imgSensorProfileEnd(char *tag)
{
    unsigned long TimeIntervalUS;
    do_gettimeofday(&T4K28YUV_ktv2);

    TimeIntervalUS = (T4K28YUV_ktv2.tv_sec - T4K28YUV_ktv1.tv_sec) * 1000000 + (T4K28YUV_ktv2.tv_usec - T4K28YUV_ktv1.tv_usec);
    SENSORDB("[%s]Profile = %lu\n",tag, TimeIntervalUS);
}
#else
inline static void T4K28YUV_imgSensorProfileStart() {}
inline static void T4K28YUV_imgSensorProfileEnd(char *tag) {}
#endif

kal_uint16 T4K28YUV_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,T4K28_WRITE_ID);
    return get_byte;
}

#define Sleep(ms)	mdelay(ms)

void T4K28YUV_write_shutter(kal_uint16 shutter)
{
	return;
}   /* write_T4K28_shutter */

static kal_uint16 T4K28YUVReg2Gain(const kal_uint8 iReg)
{
	return iReg;
}

static kal_uint8 T4K28YUVGain2Reg(const kal_uint16 iGain)
{
	return iGain;
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40); again_min = 0x1a; again_max = 0x9c
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_SetGain(UINT16 gain)
{
    kal_uint16 a_gain = gain;
	T4K28YUV_write_cmos_sensor(0x350a, ((a_gain >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
	T4K28YUV_write_cmos_sensor(0x350b, (a_gain & 0xFF));//MAG[7:0]
	return;
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_GetGain
*
* DESCRIPTION
*    This function is to get sensor anolog gain.
*
* PARAMETERS
*    anolog gain : sensor anolog gain: again_min = 0x1a; again_max = 0x9c
*
* RETURNS
*    anolog gain.
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 T4K28YUV_GetGain(void)
{
	kal_uint16 a_gain;

	a_gain = T4K28YUV_GetGain();
	a_gain = (T4K28YUV_read_cmos_sensor(0x3561) << 8) | T4K28YUV_read_cmos_sensor(0x3562);

	return a_gain;
}

/*************************************************************************
* FUNCTION
*    read_T4K28YUV_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_uint16 read_T4K28YUV_gain(void)
{
    kal_uint16 a_gain, d_gain;

	a_gain = T4K28YUV_GetGain();
	d_gain = (T4K28YUV_read_cmos_sensor(0x3563) << 8) | T4K28YUV_read_cmos_sensor(0x3564);
    return a_gain;
}/* read_T4K28YUV_gain */

/*************************************************************************
* FUNCTION
*    write_T4K28YUV_gain
*
* DESCRIPTION
*    This function is to write anolog gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void write_T4K28YUV_gain(kal_uint16 gain)
{
    T4K28YUV_SetGain(gain);
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_camera_para_to_sensor
*
* DESCRIPTION
*    update sensor register from camera_para
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_camera_para_to_sensor(void)
{
    kal_uint32 i;
    for(i = 0; 0xFFFFFFFF != T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorReg[i].Addr, T4K28YUVSensorReg[i].Para);
    }
    for(i = ENGINEER_START_ADDR; 0xFFFFFFFF != T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorReg[i].Addr, T4K28YUVSensorReg[i].Para);
    }
    for(i = FACTORY_START_ADDR; i < FACTORY_END_ADDR; i++)
    {
        T4K28YUV_write_cmos_sensor(T4K28YUVSensorCCT[i].Addr, T4K28YUVSensorCCT[i].Para);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_sensor_to_camera_para
*
* DESCRIPTION
*    update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_sensor_to_camera_para(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUVSensorReg[i].Para = T4K28YUV_read_cmos_sensor(T4K28YUVSensorReg[i].Addr);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=T4K28YUVSensorReg[i].Addr; i++)
    {
        T4K28YUVSensorReg[i].Para = T4K28YUV_read_cmos_sensor(T4K28YUVSensorReg[i].Addr);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_get_sensor_group_count
*
* DESCRIPTION
*    None
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

kal_int32  T4K28YUV_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void T4K28YUV_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
	}
}

void T4K28YUV_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Global");
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"GLOBAL_GAIN");
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }
            temp_para = T4K28YUVSensorCCT[temp_addr].Para;
            temp_gain = T4K28YUVReg2Gain(temp_para);

            temp_gain=(temp_gain * 1000) / BASEGAIN;

            info_ptr->ItemValue = temp_gain;
            info_ptr->IsTrueFalse = KAL_FALSE;
            info_ptr->IsReadOnly = KAL_FALSE;
            info_ptr->IsNeedRestart = KAL_FALSE;
            info_ptr->Min = 1000;
            info_ptr->Max = 15875;
            break;

        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg == ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue = 2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue = 4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue = 6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue = 8;
                    }

                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_TRUE;
                    info_ptr->Min = 2;
                    info_ptr->Max = 8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue = T4K28YUV_MAX_EXPOSURE_LINES;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_TRUE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue = 12;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_TRUE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue = 0;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue = 0;
                    info_ptr->IsTrueFalse = KAL_FALSE;
                    info_ptr->IsReadOnly = KAL_FALSE;
                    info_ptr->IsNeedRestart = KAL_FALSE;
                    info_ptr->Min = 0;
                    info_ptr->Max = 0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}

kal_bool T4K28YUV_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16 temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
                case 0:
                    temp_addr = PRE_GAIN_INDEX;
                    break;
                case 1:
                    temp_addr = GLOBAL_GAIN_INDEX;
                    break;
                default:
                    ASSERT(0);
            }

            temp_para = T4K28YUVGain2Reg(ItemValue);

            T4K28YUVSensorCCT[temp_addr].Para = temp_para;
            T4K28YUV_write_cmos_sensor(T4K28YUVSensorCCT[temp_addr].Addr,temp_para);

            T4K28YUV_sensor_gain_base=read_T4K28YUV_gain();

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    if(ItemValue==2)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_2MA;
                    }
                    else if(ItemValue==3 || ItemValue==4)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_4MA;
                    }
                    else if(ItemValue==5 || ItemValue==6)
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_6MA;
                    }
                    else
                    {
                        T4K28YUVSensorReg[CMMCLK_CURRENT_INDEX].Para = ISP_DRIVING_8MA;
                    }
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    T4K28YUV_FAC_SENSOR_REG=ItemValue;
                    break;
                case 1:
                    T4K28YUV_write_cmos_sensor(T4K28YUV_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}

static void T4K28YUV_SetDummy(const kal_uint16 iPixels, const kal_uint16 iLines)
{
    kal_uint16 LinesOneframe;
    kal_uint16 PixelsOneline = T4K28_FULL_PERIOD_PIXEL_NUMS;

    if(T4K28_720P == T4K28YUV_g_RES)
    {
        PixelsOneline = (T4K28_PV_PERIOD_PIXEL_NUMS_HTS + iPixels);
        LinesOneframe =iLines + T4K28_PV_PERIOD_LINE_NUMS_VTS;
        if(T4K28YUV_MPEG4_encode_mode == KAL_FALSE)
            T4K28YUV_CURRENT_FRAME_LINES = iLines + T4K28_PV_PERIOD_LINE_NUMS_VTS;
    }
    else if(T4K28_2M == T4K28YUV_g_RES)
    {
        PixelsOneline = T4K28_FULL_PERIOD_PIXEL_NUMS_HTS + iPixels;
		LinesOneframe =iLines + T4K28_FULL_PERIOD_LINE_NUMS_VTS;
        T4K28YUV_CURRENT_FRAME_LINES = iLines + T4K28_FULL_PERIOD_LINE_NUMS_VTS;
    }
    if(iPixels)
    {
    	T4K28YUV_write_cmos_sensor(0x3015, (PixelsOneline >> 8) & 0xFF);
    	T4K28YUV_write_cmos_sensor(0x3016, PixelsOneline & 0xFF);
    }
    if(iLines)
    {
    	T4K28YUV_write_cmos_sensor(0x3017, (LinesOneframe >> 8) & 0xFF);
    	T4K28YUV_write_cmos_sensor(0x3018, LinesOneframe & 0xFF);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AE_mode
*
* DESCRIPTION
*    ae enable or manual ae
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AE_mode(kal_bool AE_enable)
{
    kal_uint8 temp_AE_reg = 0;

    if (AE_enable == KAL_TRUE) {
        //turn on AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x80);
    } else {
        //turn off AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x80);
    }
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AE_status
*
* DESCRIPTION
*    AE enable, manual AE or lock AE
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AE_status(kal_uint8 AE_status)
{
    kal_uint8 temp_AE_reg = 0;

    if(AE_status == AE_enable) {
        //turn on AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, ((temp_AE_reg | 0x80) & ~0x20));
		//temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
		//T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x20);
    } else if(AE_status == AE_lock) {
        //Lock AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x20);
    } else if(AE_status == AE_unlock) {
        //Lock AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg & ~0x20);
    }
	else {
		//turn off AEC/AGC
        temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
        T4K28YUV_write_cmos_sensor(0x3500, ((temp_AE_reg & ~0x80)|0x20));
	//	temp_AE_reg = T4K28YUV_read_cmos_sensor(0x3500);
    //    T4K28YUV_write_cmos_sensor(0x3500, temp_AE_reg | 0x20);
    }
}


/*************************************************************************
* FUNCTION
*    T4K28YUV_set_AWB_mode
*
* DESCRIPTION
*    awb enable or manual awb
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_set_AWB_mode(kal_bool AWB_enable)
{
	kal_uint8 temp_AWB_reg = 0;

    if(AWB_enable == KAL_TRUE)
    {
      //enable Auto WB
      temp_AWB_reg = T4K28YUV_read_cmos_sensor(0x3500);
      T4K28YUV_write_cmos_sensor(0x3500, temp_AWB_reg | 0x40);
    }
    else
    {
       //turn off AWB
       temp_AWB_reg = T4K28YUV_read_cmos_sensor(0x3500);
       T4K28YUV_write_cmos_sensor(0x3500, temp_AWB_reg & ~0x40);
    }
}

void T4K28YUV_Sensor_Init_vga(int lines);
void T4K28YUV_set_2M_init(void);

/*************************************************************************
* FUNCTION
*    T4K28YUV_Sensor_Init
*
* DESCRIPTION
*    init sensor
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

static void T4K28YUV_Sensor_Init(int lines)

{

    SENSORDB("lln:: T4K28YUV_Sensor_Init, use T4K28YUV_Sensor_Init_vga");


   //T4K28YUV_Sensor_Init_vga(lines);
   T4K28YUV_Sensor_Init_vga(0);


    SENSORDB("Init Success \n");
}/*  T4K28YUV_Sensor_Init  */






void T4K28YUV_Sensor_Init_vga(int lines)
{
	int i;
	kal_uint32 len;
	//parser_enable = KAL_FALSE;

	if(lines)
	{
		//for(i=0;i<lines;i++)
		//T4K28YUV_write_cmos_sensor(pgParaBuffer[i].regAddr,pgParaBuffer[i].value);
		//parser_enable = KAL_TRUE;
	}
	else
	{
		len = sizeof(T4K28_Init_Reg)/sizeof(T4K28_Init_Reg[0]);
		T4K28_table_write_cmos_sensor(T4K28_Init_Reg,len,T4K28_WRITE_ID);
	}

	Sleep(20);

}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_2M_init
*
* DESCRIPTION
*    init sensor 2Mega setting
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_set_2M_init(void)
{
	T4K28YUV_write_cmos_sensor(0x3548,0x80);//CGRANGE[1:0]/-/AWBHUECOR/AWBSPD[3:0];
	T4K28YUV_write_cmos_sensor(0x3500,0xE5);//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE;
	T4K28YUV_write_cmos_sensor(0x3517,0x0A);
	T4K28YUV_write_cmos_sensor(0x3012,0x03);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD;
	T4K28YUV_write_cmos_sensor(0x3015,0x05);//-/-/-/H_COUNT[12:8];
	T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0];
	T4K28YUV_write_cmos_sensor(0x3017,0x04);//-/-/-/V_COUNT[12:8];
	T4K28YUV_write_cmos_sensor(0x3018,0x5C);//V_COUNT[7:0];
	T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8];
	T4K28YUV_write_cmos_sensor(0x301A,0x10);//SCALE_M[7:0];
	T4K28YUV_write_cmos_sensor(0x301B,0x00);//-/-/-/V_ANABIN/-/-/-/-;
	T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE;
	T4K28YUV_write_cmos_sensor(0x3020,0x06);//-/-/-/-/-/HOUTPIX[10:8];
	T4K28YUV_write_cmos_sensor(0x3021,0x40);//HOUTPIX[7:0];
	T4K28YUV_write_cmos_sensor(0x3022,0x04);//-/-/-/-/-/VOUTPIX[10:8];
	T4K28YUV_write_cmos_sensor(0x3023,0xB0);//VOUTPIX[7:0];
	T4K28YUV_write_cmos_sensor(0x334E,0x00);//-/-/-/-/LSVCNT_MPY[11:8];
	T4K28YUV_write_cmos_sensor(0x334F,0x98);//LSVCNT_MPY[7:0];
	T4K28YUV_write_cmos_sensor(0x351B,0x00);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
	T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD;
}

void T4K28YUV_set_2M(void)
{
    T4K28YUV_g_RES = T4K28_2M;
    T4K28YUV_set_2M_init();
}

void T4K28YUV_dump_2M(void)
{
}

/*************************************************************************
* FUNCTION
*    T4K28YUV_set_pv_init
*
* DESCRIPTION
*    init sensor 2Mega setting
*
* PARAMETERS
*    None
*
* RETURNS
*    None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_set_pv_init(void)
{
#if 0 // test (remove)
	T4K28YUV_write_cmos_sensor(0x351B,0x08);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
	T4K28YUV_write_cmos_sensor(0x3012,0x03);//-/-/-/-/-/-/GROUP_HOLD

	//T4K28YUV_set_AE_mode(KAL_FALSE);
	T4K28YUV_set_AE_status(AE_disable);

	T4K28YUV_write_cmos_sensor(0x3506, ((set_pv_back_es>> 8) & 0xFF));//MES[15:8]
	T4K28YUV_write_cmos_sensor(0x3507, (set_pv_back_es & 0xFF));//MES[7:0]
	T4K28YUV_write_cmos_sensor(0x350a, ((set_pv_back_ag >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
	T4K28YUV_write_cmos_sensor(0x350b, (set_pv_back_ag & 0xFF));//MAG[7:0]
	T4K28YUV_write_cmos_sensor(0x350c, set_pv_back_dg >> 2);//MDG[7:0]
#endif

	//	800*600
    T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
    T4K28YUV_write_cmos_sensor(0x3015,0x05);//-/-/-/H_COUNT[12:8]
    T4K28YUV_write_cmos_sensor(0x3016,0x16);//H_COUNT[7:0]
    T4K28YUV_write_cmos_sensor(0x3017,0x01);//-/-/-/V_COUNT[12:8]
    T4K28YUV_write_cmos_sensor(0x3018,0x75);//V_COUNT[7:0]
    T4K28YUV_write_cmos_sensor(0x3019,0x00);//-/-/-/-/-/-/-/SCALE_M[8]
    T4K28YUV_write_cmos_sensor(0x301A,0x20);//SCALE_M[7:0]
    T4K28YUV_write_cmos_sensor(0x301B,0x10);//-/-/-/V_ANABIN/-/-/-/-
    T4K28YUV_write_cmos_sensor(0x301C,0x01);//-/-/-/-/-/-/-/SCALING_MODE
    T4K28YUV_write_cmos_sensor(0x3020,0x03);//-/-/-/-/-/HOUTPIX[10:8]
    T4K28YUV_write_cmos_sensor(0x3021,0x20);//HOUTPIX[7:0]
    T4K28YUV_write_cmos_sensor(0x3022,0x02);//-/-/-/-/-/VOUTPIX[10:8]
    T4K28YUV_write_cmos_sensor(0x3023,0x58);//VOUTPIX[7:0]
    T4K28YUV_write_cmos_sensor(0x334E,0x01);//-/-/-/-/LSVCNT_MPY[11:8]
    T4K28YUV_write_cmos_sensor(0x334F,0x30);//LSVCNT_MPY[7:0]
    T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
    T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/VLAT_ON/GROUP_HOLD
    T4K28YUV_write_cmos_sensor(0x3548,0x84);//CGRANGE[1:0]/-/AWBHUECOR/AWBSPD[3:0]
    T4K28YUV_write_cmos_sensor(0x3500,0xC5);//ALCSW/AWBSW/ALCLOCK/-/ESLIMMODE/ROOMDET/-/ALCLIMMODE
    T4K28YUV_write_cmos_sensor(0x3517,0x01);

#if 0 // test (remove)
	//T4K28YUV_set_AE_mode(KAL_TRUE);
	T4K28YUV_set_AE_status(AE_enable);

	T4K28YUV_write_cmos_sensor(0x3012,0x02);//-/-/-/-/-/-/GROUP_HOLD
#endif
}

/*************************************************************************
* FUNCTION
*   T4K28YUVOpen
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 T4K28YUVOpen(void)
{
    int retry = 0;
	int lines;

    T4K28YUV_sensor_id = ((T4K28YUV_read_cmos_sensor(0x3000) << 8) | T4K28YUV_read_cmos_sensor(0x3001));
    printk("MYCAT Read Sensor ID = 0x%04x\n", T4K28YUV_sensor_id);


    if (T4K28YUV_sensor_id != T4K28_SENSOR_ID)
        return ERROR_SENSOR_CONNECT_FAIL;
    T4K28YUV_Sensor_Init(0);   //xb.pang
    spin_lock(&T4K28_drv_lock);
    t4k28yuv_status.Banding = 0;
	t4k28yuv_status.Brightness = 0;
	t4k28yuv_status.CapDummyLines = 0;
	t4k28yuv_status.CapDummyPixels = 0;
	t4k28yuv_status.CapOpClk = 0;
	t4k28yuv_status.Effect = MEFFECT_OFF;
	t4k28yuv_status.NightMode = FALSE;
	t4k28yuv_status.PvDummyLines = 0;
	t4k28yuv_status.PvDummyPixels = 0;
	t4k28yuv_status.PvOpClk = 0;
	t4k28yuv_status.PvShutter = 0;
	t4k28yuv_status.sceneMode = SCENE_MODE_NORMAL;
	t4k28yuv_status.SensorMode = SENSOR_MODE_INIT;
	t4k28yuv_status.ZoomFactor = 0;
	t4k28yuv_status.isoSpeed = AE_ISO_100;	//                                                                                      
	spin_unlock(&T4K28_drv_lock);

    return ERROR_NONE;
}



/*************************************************************************
* FUNCTION
*   T4K28YUV_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of T4K28 to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_SetShutter(kal_uint16 iShutter)
{

    if (iShutter < 1)
        iShutter = 1;

	//T4K28YUV_write_cmos_sensor(0x3500, 0x40);//-/-/-/-/-/-/ALCSW/ALCLOCK
	T4K28YUV_set_AE_status(AE_disable);

	T4K28YUV_write_cmos_sensor(0x3506, ((iShutter >> 8) & 0xFF));//MES[15:8]
	T4K28YUV_write_cmos_sensor(0x3507, (iShutter & 0xFF));//MES[7:0]
}

/*************************************************************************
* FUNCTION
*   T4K28YUV_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT16 T4K28YUV_read_shutter(void)
{
	kal_uint16 MESH = 0;
	kal_uint16 MESL = 0;
	kal_uint16 MES = 0;

	MESH = T4K28YUV_read_cmos_sensor(0x355F);//[RO]ALC_ES[15:8]
	MESL = T4K28YUV_read_cmos_sensor(0x3560);//[RO]ALC_ES[7:0]
	MES = (MESH << 8) | (MESL);
    return MES;
}

/*************************************************************************
* FUNCTION
*   T4K28_night_mode
*
* DESCRIPTION
*   This function night mode of T4K28.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

void T4K28YUV_NightMode(kal_bool bEnable)
{
	SENSORDB("T4K28YUV_NightMode()\n");

	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    if(bEnable) {
        if(T4K28YUV_MPEG4_encode_mode==KAL_TRUE) {
			T4K28YUV_write_cmos_sensor(0x351B,0x18);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 15fps

        } else {
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Auto Min 10fps
        }
    } else {
        if(T4K28YUV_MPEG4_encode_mode==KAL_TRUE) {
			T4K28YUV_write_cmos_sensor(0x351B,0x08); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 30fps
        } else {
			T4K28YUV_write_cmos_sensor(0x351B,0x98); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Auto Min 15fps
        }
    }

}

/*************************************************************************
* FUNCTION
*   T4K28YUVClose
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 T4K28YUVClose(void)
{
    //parser_close();  //xb.pang
    return ERROR_NONE;
}

void T4K28YUV_Set_Mirror_Flip(kal_uint8 image_mirror)
{
	kal_uint16 iMirror, iFlip;

    switch (image_mirror)
	{
	    case IMAGE_NORMAL:
			T4K28YUV_write_cmos_sensor(0x3011,0x00);//Set normal
			break;
	    case IMAGE_H_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x01);//set IMAGE_H_MIRROR
			break;
	    case IMAGE_V_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x02);//set IMAGE_V_MIRROR
			break;
	    case IMAGE_HV_MIRROR:
			T4K28YUV_write_cmos_sensor(0x3011,0x03);//Set IMAGE_HV_MIRROR
			break;
    }
}

/*************************************************************************
* FUNCTION
*   T4K28YUVPreview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
bool capture_flag = FALSE;
UINT32 T4K28YUVPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	kal_uint16 iStartX = 0, iStartY = 0;

	g_iT4K28YUV_Mode = T4K28_MODE_PREVIEW;
	capture_flag = FALSE;

	if(sensor_config_data->SensorOperationMode == MSDK_SENSOR_OPERATION_MODE_VIDEO)
	{
	    spin_lock(&T4K28_drv_lock);
		T4K28YUV_MPEG4_encode_mode = KAL_TRUE;
		spin_unlock(&T4K28_drv_lock);
	}
	else
	{
		spin_lock(&T4K28_drv_lock);
		T4K28YUV_MPEG4_encode_mode = KAL_FALSE;
		spin_unlock(&T4K28_drv_lock);
	}

	T4K28YUV_set_pv_init();
	if(CurrentScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW)
	{
		T4K28YUV_write_cmos_sensor(0x351B,0x98); //Auto Min 15fps
	}

	iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
	iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
	image_window->GrabStartX = iStartX;
	image_window->GrabStartY = iStartY;
	image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * iStartX;
	image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * iStartY;

#if 0 // test (remove)
	T4K28YUV_Set_Mirror_Flip(IMAGE_NORMAL);
#endif
	spin_lock(&T4K28_drv_lock);
	t4k28yuv_status.SensorMode = SENSOR_MODE_PREVIEW;
    spin_unlock(&T4K28_drv_lock);

	memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

#if 0 // test (remove)
	T4K28YUV_write_cmos_sensor(0x351B,0x98);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0]
#endif

	return ERROR_NONE;
}	/* T4K28YUVPreview() */

UINT32 T4K28YUVCapture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    kal_uint16 iStartX = 0, iStartY = 0;

	kal_uint32 MESH = 0;
	kal_uint32 MESL = 0;
	kal_uint32 MES = 0;
	kal_uint32 MAGH = 0;
	kal_uint32 MAGL = 0;
	kal_uint32 MAG = 0;
	kal_uint32 MDG = 0;
	kal_uint32 MDGH = 0;
	kal_uint32 MDGL = 0;

	kal_uint32 tmp_ae;

#if 0 // test (remove)
	//T4K28YUV_write_cmos_sensor(0x3500,0xE0);   //-/-/-/-/-/-/ALCSW/ALCLOCK
	T4K28YUV_set_AE_status(AE_lock);
#endif

	if(capture_flag == FALSE) {
		MESH = T4K28YUV_read_cmos_sensor(0x355F);//[RO]ALC_ES[15:8]
		MESL = T4K28YUV_read_cmos_sensor(0x3560);//[RO]ALC_ES[7:0]
		MAGH = T4K28YUV_read_cmos_sensor(0x3561);//[RO]-/-/-/-/ALC_AG[11:8]
		MAGL = T4K28YUV_read_cmos_sensor(0x3562);//[RO]ALC_AG[7:0]
		MDGH = T4K28YUV_read_cmos_sensor(0x3563);//[RO]-/-/-/-/-/-/ALC_DG[9:8]
		MDGL = T4K28YUV_read_cmos_sensor(0x3564);//[RO]ALC_DG[7:0]
	    capture_flag = TRUE;

		spin_lock(&T4K28_drv_lock);
	    t4k28yuv_status.SensorMode = SENSOR_MODE_CAPTURE;
        spin_unlock(&T4K28_drv_lock);
	}
//L = MES * MAG *MDG;
	MES = (MESH << 8) | (MESL);
	MAG = (MAGH << 8) | (MAGL);
	MDG = ((MDGH & 0x03) << 8) | (MDGL & 0xFF);

	set_pv_back_es = MES;
	set_pv_back_ag = MAG;
	set_pv_back_dg = MDG;

	if ((image_window->ImageTargetWidth <= T4K28_IMAGE_SENSOR_PV_WIDTH) &&
        (image_window->ImageTargetHeight <= T4K28_IMAGE_SENSOR_PV_HEIGHT)) {

        iStartX = T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * iStartY;
    } else { // 2M  Mode
		T4K28YUV_set_2M();

#if 0 // test (remove)
		T4K28YUV_write_cmos_sensor(0x3506, ((MES >> 8) & 0xFF));//MES[15:8]
		T4K28YUV_write_cmos_sensor(0x3507, (MES & 0xFF));//MES[7:0]
		T4K28YUV_write_cmos_sensor(0x350a, ((MAG >> 8) & 0xFF));//ESLIMMODE/ROOMDET/-/-/MAG[11:8]
		T4K28YUV_write_cmos_sensor(0x350b, (MAG & 0xFF));//MAG[7:0]
		T4K28YUV_write_cmos_sensor(0x350c, MDG >> 2);//MDG[7:0]

        if (MES > ((0x0300)*2+10))
        {
		   T4K28YUV_write_cmos_sensor(0x3017, ((MES >> 8) & 0xFF));;//-/-/-/V_COUNT[12:8]
		   T4K28YUV_write_cmos_sensor(0x3018, (MES & 0xFF));;//V_COUNT[12:8]
        }
#endif

        iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH -2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT-2 * iStartY;

    }

    memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}
UINT32 T4K28YUVZsdPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	spin_lock(&T4K28_drv_lock);
	t4k28yuv_status.SensorMode = SENSOR_MODE_ZSD;
    spin_unlock(&T4K28_drv_lock);

	//T4K28YUV_write_cmos_sensor(0x3500,0xE0);   //-/-/-/-/-/-/ALCSW/ALCLOCK
	kal_uint16 iStartX = 0, iStartY = 0;
	T4K28YUV_set_2M();
	T4K28YUV_set_AE_status(AE_enable);
	        iStartX = 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
        iStartY = 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
        image_window->GrabStartX = iStartX;
        image_window->GrabStartY = iStartY;
        image_window->ExposureWindowWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH -2 * iStartX;
        image_window->ExposureWindowHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT-2 * iStartY;

#if 0 // test (remove)
		T4K28YUV_Set_Mirror_Flip(IMAGE_NORMAL);
#endif
		memcpy(&T4K28YUVSensorConfigData, sensor_config_data, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    return ERROR_NONE;
}


UINT32 T4K28YUVGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    pSensorResolution->SensorFullWidth = T4K28_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight = T4K28_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorPreviewWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorPreviewHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
   	pSensorResolution->SensorVideoWidth = T4K28_IMAGE_SENSOR_PV_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorResolution->SensorVideoHeight = T4K28_IMAGE_SENSOR_PV_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;

    return ERROR_NONE;
}
UINT32 T4K28YUVGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
													MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	//pSensorInfo->SensorPreviewResolutionX = T4K28_IMAGE_SENSOR_PV_WIDTH;
    //pSensorInfo->SensorPreviewResolutionY = T4K28_IMAGE_SENSOR_PV_HEIGHT;
	switch(ScenarioId)
	{
		  case MSDK_SCENARIO_ID_CAMERA_ZSD:
			   pSensorInfo->SensorPreviewResolutionX=T4K28_IMAGE_SENSOR_FULL_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;;
			   pSensorInfo->SensorPreviewResolutionY=T4K28_IMAGE_SENSOR_FULL_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;
			   pSensorInfo->SensorCameraPreviewFrameRate=14;
		  break;
		  default:
			   pSensorInfo->SensorPreviewResolutionX=T4K28_IMAGE_SENSOR_PV_WIDTH;
			   pSensorInfo->SensorPreviewResolutionY=T4K28_IMAGE_SENSOR_PV_HEIGHT;
			   pSensorInfo->SensorCameraPreviewFrameRate=30;
	}

	pSensorInfo->SensorFullResolutionX = T4K28_IMAGE_SENSOR_FULL_WIDTH - 2 * T4K28_IMAGE_SENSOR_PV_STARTX;
    pSensorInfo->SensorFullResolutionY = T4K28_IMAGE_SENSOR_FULL_HEIGHT - 2 * T4K28_IMAGE_SENSOR_PV_STARTY;

    pSensorInfo->SensorCameraPreviewFrameRate = 30;
    pSensorInfo->SensorVideoFrameRate = 30;
    pSensorInfo->SensorStillCaptureFrameRate = 14;
    pSensorInfo->SensorWebCamCaptureFrameRate = 15;
    pSensorInfo->SensorResetActiveHigh = FALSE;
    pSensorInfo->SensorResetDelayCount = 5;
    pSensorInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_UYVY;

    pSensorInfo->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_HIGH;
    pSensorInfo->SensorInterruptDelayLines = 1;
    pSensorInfo->SensroInterfaceType = SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 1;//                                                                       
    pSensorInfo->PreviewDelayFrame = 4;//2;
    pSensorInfo->VideoDelayFrame = 5;
    pSensorInfo->SensorMasterClockSwitch = 0;
    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;

    pSensorInfo->AEShutDelayFrame = 0;/* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 1;/* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 1;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount =	3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = T4K28_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = T4K28_IMAGE_SENSOR_PV_STARTY;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount = 3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = 2*T4K28_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = 2*T4K28_IMAGE_SENSOR_PV_STARTY;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
            pSensorInfo->SensorClockFreq = 24;
            pSensorInfo->SensorClockDividCount =	3;
            pSensorInfo->SensorClockRisingCount = 0;
            pSensorInfo->SensorClockFallingCount = 2;
            pSensorInfo->SensorPixelClockCount = 3;
            pSensorInfo->SensorDataLatchCount = 2;
            pSensorInfo->SensorGrabStartX = 1;
            pSensorInfo->SensorGrabStartY = 1;

			pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	        pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	        pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
            pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    T4K28YUVPixelClockDivider=pSensorInfo->SensorPixelClockCount;
    memcpy(pSensorConfigData, &T4K28YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* T4K28YUVGetInfo() */


UINT32 T4K28YUVControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    CurrentScenarioId = ScenarioId;
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            SENSORDB("CONTROLFLOW MSDK_SCENARIO_ID_CAMERA/VIDEO_PREVIEW (%d)\n", ScenarioId);
            T4K28YUVPreview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            SENSORDB("CONTROLFLOW MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG\n");
            T4K28YUVCapture(pImageWindow, pSensorConfigData);
            break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
	        SENSORDB("CONTROLFLOW MSDK_SCENARIO_ID_CAMERA_ZSD\n" );
	        T4K28YUVZsdPreview(pImageWindow, pSensorConfigData);
	        break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return TRUE;
}

UINT32 T4K28YUVSetVideoMode(UINT16 u2FrameRate)
{
    //XB.PANG NEED CHECK set the video mode fps---normal/night

    if(u2FrameRate == 15) {
			T4K28YUV_write_cmos_sensor(0x351B,0x18);  //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 15fps

    } else if(u2FrameRate == 30) {
			T4K28YUV_write_cmos_sensor(0x351B,0x98); //FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0] //Fix 30fps
    }else
    {
      printk("Wrong frame rate setting \n");
	}

    spin_lock(&T4K28_drv_lock);
    T4K28YUV_MPEG4_encode_mode = KAL_TRUE;
	spin_unlock(&T4K28_drv_lock);
    return TRUE;
}

kal_uint32 T4K28_set_param_wb(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

	switch (para)
	{
		case AWB_MODE_AUTO:
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT: //cloudy
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x26);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x1C);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x02);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x02);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x02);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x02);//WBGBMIN[7:0];
			break;

		case AWB_MODE_DAYLIGHT: //sunny
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x20);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x05);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x08);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x08);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x05);//WBGBMIN[7:0];
			break;

		case AWB_MODE_INCANDESCENT: //office
			T4K28YUV_write_cmos_sensor(0x3322,0x1C);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x1C);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x70);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x05);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x10);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x05);//WBGBMIN[7:0];
			break;

		case AWB_MODE_TUNGSTEN: //home
#if 0 // test (remove) : TODO: Request tungsten values...
			T4K28YUV_write_cmos_sensor(0x3500, 0x80);//AWBSW/AWBONDOT[2:0]/-/-/WBMRG[9:8];
			T4K28YUV_write_cmos_sensor(0x3322, 0x40);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323, 0x40);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324, 0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325, 0xE0);//PWBGAINB[7:0];
#endif
			break;

		case AWB_MODE_FLUORESCENT:
			T4K28YUV_write_cmos_sensor(0x3322,0x12);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x12);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x4F);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x10);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x05);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x0C);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		default:
			return KAL_FALSE;
	}
	return KAL_TRUE;
}

kal_uint32 T4K28_set_param_exposure(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;
	if (t4k28yuv_status.sceneMode == SCENE_MODE_HDR)
	{
		switch (para)
		{
			case AE_EV_COMP_n20:
				/* EV -2 */
				SENSORDB("[Hi251_Debug_HDR]AE_EV_COMP_n20 Para:%d;\n",para);
				T4K28YUV_write_cmos_sensor(0x3501,0x00);
				T4K28YUV_write_cmos_sensor(0x3502,0x64);
				break;
			case AE_EV_COMP_20:			   /* EV +2 */
				SENSORDB("[Hi251_Debug_HDR]AE_EV_COMP_20 Para:%d;\n",para);
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0x68);
				break;
			case AE_EV_COMP_00:			 /* EV 00 */
				SENSORDB("[Hi251_Debug_HDR]ISP_BRIGHT_MIDDLE Para:%d;\n",para);
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0xEE);
				break;
			default:
				return KAL_FALSE;
		}
	}
	else
	{
		switch (para)
		{
			case AE_EV_COMP_n30:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0x32);
				break;

			case AE_EV_COMP_n20:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0x64);
				break;

			case AE_EV_COMP_n10:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0x96);
				break;

			case AE_EV_COMP_00:
				T4K28YUV_write_cmos_sensor(0x3501,0x0);
				T4K28YUV_write_cmos_sensor(0x3502,0xEE);
				break;

			case AE_EV_COMP_10:
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0x18);
				break;

			case AE_EV_COMP_20:
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0x68);
				break;

			case AE_EV_COMP_30:
				T4K28YUV_write_cmos_sensor(0x3501,0x01);
				T4K28YUV_write_cmos_sensor(0x3502,0xB8);
				break;

			default:
				return KAL_FALSE;
		}
	}
    return KAL_TRUE;
}

kal_uint32 T4K28_set_param_effect(kal_uint32 para)
{
    kal_uint32 ret = KAL_TRUE;

	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    switch (para)
    {
        case MEFFECT_OFF:
			T4K28YUV_write_cmos_sensor(0x3402,0x00);//-/-/-/-/PICEFF[3:0];
            break;

	 	case MEFFECT_MONO:
			T4K28YUV_write_cmos_sensor(0x3402,0x06);//-/-/-/-/PICEFF[3:0];
            break;

        case MEFFECT_SEPIA:
			T4K28YUV_write_cmos_sensor(0x3402, 0x05);//-/-/-/-/PICEFF[3:0];
#if 0 // test (remove)
			T4K28YUV_write_cmos_sensor(0x3454, 0x6A);//
			T4K28YUV_write_cmos_sensor(0x3455, 0x93);//
#endif
            break;

        case MEFFECT_NEGATIVE:
			T4K28YUV_write_cmos_sensor(0x3402,0x03);//-/-/-/-/PICEFF[3:0];
            break;

        case MEFFECT_SEPIABLUE:
			T4K28YUV_write_cmos_sensor(0x3402, 0x05);//-/-/-/-/PICEFF[3:0];
#if 0 // test (remove)
			T4K28YUV_write_cmos_sensor(0x3454, 0xa9);//
			T4K28YUV_write_cmos_sensor(0x3455, 0x60);//
#endif
            break;

        default:
            ret = KAL_FALSE;
    }
    return KAL_TRUE;
}

kal_uint32 T4K28_set_param_banding(kal_uint32 para)
{
	//if(KAL_TRUE = parser_enable) return KAL_TRUE;

    switch (para)
    {
        case AE_FLICKER_MODE_50HZ:
			T4K28YUV_write_cmos_sensor(0x351E,0x16);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
	    	break;

        case AE_FLICKER_MODE_60HZ:
			T4K28YUV_write_cmos_sensor(0x351E,0x56);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
	    	break;

        default:
            T4K28YUV_write_cmos_sensor(0x351E,0xd6);//ACFDET/AC60M/FLMANU/ACDETDLY/MSKLINE[1:0]/ACDPWAIT[1:0]
            return KAL_FALSE;
    }
    return KAL_TRUE;
}

//add new; XB.PANG NEED CHECK
void T4K28YUVset_scene_mode(UINT16 para)
{
	spin_lock(&T4K28_drv_lock);
	t4k28yuv_status.sceneMode = para;
	spin_unlock(&T4K28_drv_lock);

	SENSORDB("iPara=%d",para);
	switch(para)
	{
		case SCENE_MODE_HDR:
			//XB.PANG NEED CHECK
			break;

		case SCENE_MODE_SUNSET:
			//XB.PANG NEED CHECK
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x19);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x1b);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x92);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0xa0);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xA8);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x26);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x1c);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x02);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x02);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x02);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x02);//WBGBMIN[7:0];
			break;

		case SCENE_MODE_NIGHTSCENE:
#if 0 // test (remove)
			T4K28YUV_NightMode(TRUE);
#endif
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x20);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x19);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x1b);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x92);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0xa0);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xA8);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		case SCENE_MODE_PORTRAIT:
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x08);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x0B);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x80);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0x80);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xA8);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		case SCENE_MODE_LANDSCAPE:
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x19);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x1b);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x92);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0xa0);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xD0);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		case SCENE_MODE_SPORTS:
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x19);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x1b);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x92);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0xa0);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xA8);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			T4K28YUV_write_cmos_sensor(0x351B,0x08);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

		case SCENE_MODE_OFF:
		default:
#if 0 // test (remove)
			T4K28YUV_NightMode(FALSE);
#endif
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x350D,0x19);//A1WEIGHT[1:0]/A2WEIGHT[1:0]/A3WEIGHT[1:0]/A4WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350E,0x1b);//A5WEIGHT[1:0]/B1WEIGHT[1:0]/B2WEIGHT[1:0]/B3WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x350F,0x92);//B4WEIGHT[1:0]/B5WEIGHT[1:0]/C1WEIGHT[1:0]/C2WEIGHT[1:0];
			T4K28YUV_write_cmos_sensor(0x3510,0xa0);//C3WEIGHT[1:0]/C4WEIGHT[1:0]/C5WEIGHT[1:0]/-/-;
			T4K28YUV_write_cmos_sensor(0x3422,0xB0);//Cbr_MGAIN[7:0];
			T4K28YUV_write_cmos_sensor(0x3322,0x00);//PWBGAINGR[7:0];
			T4K28YUV_write_cmos_sensor(0x3323,0x00);//PWBGAINGB[7:0];
			T4K28YUV_write_cmos_sensor(0x3324,0x00);//PWBGAINR[7:0];
			T4K28YUV_write_cmos_sensor(0x3325,0x38);//PWBGAINB[7:0];
			if(CurrentScenarioId == MSDK_SCENARIO_ID_VIDEO_PREVIEW)
			{
				T4K28YUV_write_cmos_sensor(0x351B,0x98); //Auto Min 15fps
			}
			else
			{
				T4K28YUV_write_cmos_sensor(0x351B,0xA8);//FAUTO/FCOUNT[2:0]/FCLSBON/EXPLIM[2:0];
			}
			T4K28YUV_write_cmos_sensor(0x352A,0x20);//WBGRMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352B,0x20);//WBGRMIN[7:0];
			T4K28YUV_write_cmos_sensor(0x352C,0x20);//WBGBMAX[7:0];
			T4K28YUV_write_cmos_sensor(0x352D,0x20);//WBGBMIN[7:0];
			break;

	}
}

//add new; XB.PANG NEED CHECK
void T4K28_set_contrast(UINT16 para)
{
#if 0 // test (remove)
	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_contrast Para:%d;\n",para);

	switch (para)
    {
        case ISP_CONTRAST_LOW:
			 //Low
			 T4K28YUV_write_cmos_sensor(0x3444, 0x0);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x0);
             break;

        case ISP_CONTRAST_HIGH:
			 //Hig
			 T4K28YUV_write_cmos_sensor(0x3444, 0x8);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x8);
             break;


        case ISP_CONTRAST_MIDDLE:
        default:
	         //Med
	         T4K28YUV_write_cmos_sensor(0x3444, 0x4);
			 T4K28YUV_write_cmos_sensor(0x3446, 0x4);
             break;
    }
#endif
    return;
}

//add new; XB.PANG NEED CHECK
void T4K28_set_brightness(UINT16 para)
{
#if 0 // test (remove)
	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_brightness Para:%d;\n",para);

    switch (para)
    {
        case ISP_BRIGHT_LOW:
		     //Low
			 T4K28YUV_write_cmos_sensor(0x343f, 0xe0);
			 T4K28YUV_write_cmos_sensor(0x3440, 0xc0);
             break;

        case ISP_BRIGHT_HIGH:
		     //Hig
			 T4K28YUV_write_cmos_sensor(0x343f, 0x40);
			 T4K28YUV_write_cmos_sensor(0x3440, 0x20);
             break;

        case ISP_BRIGHT_MIDDLE:
        default:
	         //Med
			 T4K28YUV_write_cmos_sensor(0x343f, 0x10);
			 T4K28YUV_write_cmos_sensor(0x3440, 0xf0);
             break;
    }
#endif
    return;
}

//add new; XB.PANG NEED CHECK
void T4K28_set_saturation(UINT16 para)
{
#if 0 // test (remove)
	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_saturation Para:%d;\n",para);
	//0x341e
    //0x3421
    switch (para)
    {
        case ISP_SAT_HIGH:
	         //Hig
	         T4K28YUV_write_cmos_sensor(0x341e, 0xe0);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xe0);
             break;

        case ISP_SAT_LOW:
	         //Low
	         T4K28YUV_write_cmos_sensor(0x341e, 0xa0);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xa0);
             break;

        case ISP_SAT_MIDDLE:
        default:
	         //Med
	         T4K28YUV_write_cmos_sensor(0x341e, 0xc4);
			 T4K28YUV_write_cmos_sensor(0x3421, 0xc4);
             break;
    }
#endif
    return;
}

//add new; XB.PANG NEED CHECK
void T4K28_set_iso(UINT16 para)
{

	SENSORDB("[T4K28]CONTROLFLOW T4K28_set_iso Para:%d;\n",para);

    //                                                                                        
    spin_lock(&T4K28_drv_lock);
    t4k28yuv_status.isoSpeed = para;
    spin_unlock(&T4K28_drv_lock);
    //                                                                                        

	switch (para)
	{
		case AE_ISO_100:
			//ISO100
			T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8];
			T4K28YUV_write_cmos_sensor(0x3509,0x10);//MMES[7:0];
			T4K28YUV_write_cmos_sensor(0x351A,0xFF);//SATSET[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x00);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x25);//AGMAX[7:0];
			break;

		case AE_ISO_200:
			//ISO200
			T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8];
			T4K28YUV_write_cmos_sensor(0x3509,0x10);//MMES[7:0];
			T4K28YUV_write_cmos_sensor(0x351A,0xFF);//SATSET[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x00);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x4A);//AGMAX[7:0];
			break;

		case AE_ISO_400:
			//ISO400
			T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8];
			T4K28YUV_write_cmos_sensor(0x3509,0x10);//MMES[7:0];
			T4K28YUV_write_cmos_sensor(0x351A,0xFF);//SATSET[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x00);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x94);//AGMAX[7:0];
			break;

		default:
		case AE_ISO_AUTO:
			//Auto
			T4K28YUV_write_cmos_sensor(0x3508,0x00);//MMES[15:8];
			T4K28YUV_write_cmos_sensor(0x3509,0x86);//MMES[7:0];
			T4K28YUV_write_cmos_sensor(0x351A,0xC0);//SATSET[7:0];
			T4K28YUV_write_cmos_sensor(0x3504,0x01);//-/-/-/-/AGMAX[11:8];
			T4K28YUV_write_cmos_sensor(0x3505,0x00);//AGMAX[7:0];
			break;
	}

	return;
}

void T4K28_set_hue(UINT16 para)
{
#if 0 // test (remove)
	SENSORDB("[Enter] T4K28_set_hue func:para = %d,ISP_HUE_MIDDLE=%d\n",para,ISP_HUE_MIDDLE);

	switch (para)
	{
		case ISP_HUE_LOW:
			T4K28YUV_write_cmos_sensor(0x3404, 0x3b);
			T4K28YUV_write_cmos_sensor(0x3405, 0xdd);
			T4K28YUV_write_cmos_sensor(0x3406, 0x1b);
			T4K28YUV_write_cmos_sensor(0x3407, 0x2a);
			T4K28YUV_write_cmos_sensor(0x3408, 0x2b);
			T4K28YUV_write_cmos_sensor(0x3409, 0x4c);
			break;

		case ISP_HUE_HIGH:
			 T4K28YUV_write_cmos_sensor(0x3404, 0x39);
			 T4K28YUV_write_cmos_sensor(0x3405, 0xe9);
			 T4K28YUV_write_cmos_sensor(0x3406, 0x29);
			 T4K28YUV_write_cmos_sensor(0x3407, 0x20);
			 T4K28YUV_write_cmos_sensor(0x3408, 0x03);
			 T4K28YUV_write_cmos_sensor(0x3409, 0x6c);
			 break;

		case ISP_HUE_MIDDLE:
		default:
			 T4K28YUV_write_cmos_sensor(0x3404, 0x3a);
			 T4K28YUV_write_cmos_sensor(0x3405, 0xe3);
			 T4K28YUV_write_cmos_sensor(0x3406, 0x22);
			 T4K28YUV_write_cmos_sensor(0x3407, 0x25);
			 T4K28YUV_write_cmos_sensor(0x3408, 0x17);
			 T4K28YUV_write_cmos_sensor(0x3409, 0x5c);
			 break;

	}
#endif
	return;
}


UINT32 T4K28YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
    //printk("\n T4K28YUVSensorSetting() is called ; \n");
    //SENSORDB("cmd=%d, para = 0x%x\n", iCmd, iPara);

	switch (iCmd)
	{
	case FID_SCENE_MODE:
		SENSORDB("[T4K28]FID_SCENE_MODE:%d\n",iPara);
		T4K28YUVset_scene_mode(iPara);
		//XB.PANG NEED CHECK
		/*
		 if ((iPara == SCENE_MODE_NORMAL) || (iPara == SCENE_MODE_OFF))
		 	{
		        T4K28YUV_NightMode(FALSE);
		    }
		 else if (iPara == SCENE_MODE_NIGHTSCENE)
		 	{
				T4K28YUV_NightMode(TRUE);
		    }
		    */
		break;

	case FID_AWB_MODE:
		SENSORDB("[T4K28]FID_AWB_MODE:%d\n",iPara);
		T4K28_set_param_wb(iPara);
		break;

	case FID_COLOR_EFFECT:
		SENSORDB("[T4K28]FID_COLOR_EFFECT:%d\n",iPara);
		T4K28_set_param_effect(iPara);
		break;

	case FID_AE_EV:
		SENSORDB("[T4K28]FID_AE_EV:%d\n",iPara);
		T4K28_set_param_exposure(iPara);
		break;

	case FID_AE_FLICKER:
		SENSORDB("[T4K28]FID_AE_FLICKER:%d\n",iPara);
		T4K28_set_param_banding(iPara);
		break;

	case FID_ZOOM_FACTOR:
		SENSORDB("[T4K28]FID_ZOOM_FACTOR:%d\n",iPara);
        T4K28YUV_zoom_factor = iPara;
		break;
    //XB.PANG NEED CHECK
	case FID_ISP_CONTRAST:
		SENSORDB("[T4K28]FID_ISP_CONTRAST:%d\n",iPara);
		T4K28_set_contrast(iPara);
		break;
	case FID_ISP_BRIGHT:
		SENSORDB("[T4K28]FID_ISP_BRIGHT:%d\n",iPara);
		T4K28_set_brightness(iPara);
		break;
	case FID_ISP_SAT:
		SENSORDB("[T4K28]FID_ISP_SAT:%d\n",iPara);
		T4K28_set_saturation(iPara);
		break;
	case FID_AE_ISO:
		SENSORDB("[T4K28]FID_AE_ISO:%d\n",iPara);
		T4K28_set_iso(iPara);
		break;
	case FID_ISP_HUE:
		SENSORDB("[T4K28]FID_ISP_HUE:%d\n",iPara);
		T4K28_set_hue(iPara);
		break;

	default:
		break;
    }
    return TRUE;
}

/*************************************************************************
* FUNCTION
*   T4K28YUVGetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 T4K28YUVGetSensorID(UINT32 *sensorID)
{
    int  retry = 3;
	UINT32 u_sensorid1,u_sensorid2;

    do {
		u_sensorid1 = T4K28YUV_read_cmos_sensor(0x3000);
		u_sensorid2 = T4K28YUV_read_cmos_sensor(0x3001);
		printk("MYCAT Read Sensor ID1,ID2  = %x,%x\n", u_sensorid1, u_sensorid2);
		*sensorID = (((u_sensorid1&0XFF) << 8 ) | (u_sensorid2&0XFF));
		if (*sensorID == T4K28_SENSOR_ID)
		    break;
		printk("MYCAT Read Sensor ID Fail = 0x%04x\n", *sensorID);
		retry --;
    } while (retry > 0);

    if (*sensorID != T4K28_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }
    return ERROR_NONE;
}
typedef struct
{
  UINT16  iSensorVersion;
  UINT16  iNightMode;
  UINT16  iWB;
  UINT16  iEffect;
  UINT16  iEV;
  UINT16  iBanding;
  UINT16  iMirror;
  UINT16  iFrameRate;
  //0    :No-Fix FrameRate
  //other:Fixed FrameRate
} T4K28Status;
T4K28Status T4K28CurrentStatus;

//                                                                                        
static kal_uint16 T4K28GetRealIso(void)
{
	kal_uint16 agH, agL, ag, iso;

	agH = T4K28YUV_read_cmos_sensor(0x3561);	// -/-/-/-/ALC_AG[11:8]
	agL = T4K28YUV_read_cmos_sensor(0x3562);	// ALC_AG[7:0]
	ag = ((agH&0xF)<<8)|(agL&0xFF);

	if (ag <= 9)
		iso = 25;
	else if (ag <= 12)
		iso = 32;
	else if (ag <= 15)
		iso = 40;
	else if (ag <= 24)
		iso = 65;
	else if (ag <= 30)
		iso = 80;
	else if (ag <= 37)
		iso = 100;
	else if (ag <= 46)
		iso = 125;
	else if (ag <= 59)
		iso = 160;
	else if (ag <= 74)
		iso = 200;
	else if (ag <= 93)
		iso = 250;
	else if (ag <= 118)
		iso = 320;
	else if (ag <= 148)
		iso = 400;
	else if (ag <= 185)
		iso = 500;
	else if (ag <= 236)
		iso = 640;
	else
		iso = 800;

	SENSORDB("[%s] iso = %d (ag=%d)\n",__FUNCTION__, iso, ag);

	return (t4k28yuv_status.isoSpeed == AE_ISO_AUTO) ? iso : t4k28yuv_status.isoSpeed;
}
//                                              

void T4K28GetExifInfo(UINT32 exifAddr)
{
    SENSOR_EXIF_INFO_STRUCT* pExifInfo = (SENSOR_EXIF_INFO_STRUCT*)exifAddr;
    pExifInfo->FNumber = 28;
    pExifInfo->AEISOSpeed = T4K28GetRealIso();       //                                                                                      
    pExifInfo->AWBMode = T4K28CurrentStatus.iWB;
    pExifInfo->CapExposureTime = 0;
    pExifInfo->FlashLightTimeus = 0;
    pExifInfo->RealISOValue = pExifInfo->AEISOSpeed; //                                                                                      
    pExifInfo->FocalLength = 270;                    //                                                            
}



#define T4K28YUV_FLASH_BV_THRESHOLD 120
static void T4K28YUVMIPI_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	unsigned int NormBr;

	NormBr=((T4K28YUV_read_cmos_sensor(0x3561)&0xff)<<8)+T4K28YUV_read_cmos_sensor(0x3562);

    printk("[%s]gain =0x%x \n",__FUNCTION__,NormBr);


	if (NormBr < T4K28YUV_FLASH_BV_THRESHOLD)
	{
	   *pFeatureReturnPara32 = FALSE;
		return;
	}
	*pFeatureReturnPara32 = TRUE;
	return;
}

//XB.PANG NEED CHECK
void T4K28YUV_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
   switch (action)
   {
      case SENSOR_3A_AE_LOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AE_LOCK\n");//T4K28YUV_set_AE_mode
          T4K28YUV_set_AE_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AE_UNLOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AE_UNLOCK\n");
          T4K28YUV_set_AE_mode(KAL_TRUE);
      break;

      case SENSOR_3A_AWB_LOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AWB_LOCK\n");
		  T4K28YUV_set_AWB_mode(KAL_FALSE);
      break;

      case SENSOR_3A_AWB_UNLOCK:
          SENSORDB("[T4K28YUV] SENSOR_3A_AWB_UNLOCK\n");
          T4K28YUV_set_AWB_mode(KAL_TRUE);

      break;

      default:
      break;
   }
   return;
}

#define T4K28YUV_TEST_PATTERN_CHECKSUM 0x96f31f11 //0x0334b745
UINT32 T4K28YUVSetTestPatternMode(kal_bool bEnable)
{
	kal_uint8 temp_testpattern_reg = 0;
    temp_testpattern_reg = T4K28YUV_read_cmos_sensor(0x335F);
    //XB.PANG NEED CHECK
    if (bEnable)
    {
		//enable test pattern
		T4K28YUV_write_cmos_sensor(0x335F, (temp_testpattern_reg | 0x01));
	}
	else
	{
		//disable test pattern
		T4K28YUV_write_cmos_sensor(0x335F, (temp_testpattern_reg & 0xFE));
	}
    return ERROR_NONE;
}



UINT32 T4K28YUVFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara, UINT32 *pFeatureParaLen)
{
    UINT8  *pFeatureData8 =pFeaturePara;
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData = (PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData = (MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData = (MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo = (MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo = (MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo = (MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++ = IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16 = IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_PERIOD:
            *pFeatureReturnPara16++ = T4K28_PV_PERIOD_PIXEL_NUMS + T4K28YUV_dummy_pixels;//T4K28_PV_PERIOD_PIXEL_NUMS+T4K28YUV_dummy_pixels;
            *pFeatureReturnPara16 = T4K28_PV_PERIOD_LINE_NUMS+T4K28YUV_dummy_lines;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            *pFeatureReturnPara32 = 55250000; //19500000;
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_SET_ESHUTTER:
			SENSORDB("SENSOR_FEATURE_SET_ESHUTTER\n");
            T4K28YUV_SetShutter(*pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_NIGHTMODE:
			SENSORDB("SENSOR_FEATURE_SET_NIGHTMODE\n");
            T4K28YUV_NightMode((BOOL) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_GAIN:
			SENSORDB("SENSOR_FEATURE_SET_GAIN\n");
            T4K28YUV_SetGain((UINT16) *pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;

        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            T4K28YUV_isp_master_clock=*pFeatureData32;
            break;

        case SENSOR_FEATURE_SET_REGISTER:
			SENSORDB("SENSOR_FEATURE_SET_REGISTER\n");
            T4K28YUV_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;

        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = T4K28YUV_read_cmos_sensor(pSensorRegData->RegAddr);
            break;

        case SENSOR_FEATURE_SET_CCT_REGISTER:
			SENSORDB("SENSOR_FEATURE_SET_CCT_REGISTER\n");
            SensorRegNumber = FACTORY_END_ADDR;
            for (i = 0; i < SensorRegNumber; i++)
            {
                T4K28YUVSensorCCT[i].Addr = *pFeatureData32++;
                T4K28YUVSensorCCT[i].Para = *pFeatureData32++;
            }
            break;

        case SENSOR_FEATURE_GET_CCT_REGISTER:
			SENSORDB("SENSOR_FEATURE_GET_CCT_REGISTER\n");
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++ = SensorRegNumber;
            for (i = 0; i < SensorRegNumber; i++)
            {
                *pFeatureData32++ = T4K28YUVSensorCCT[i].Addr;
                *pFeatureData32++ = T4K28YUVSensorCCT[i].Para;
            }
            break;

        case SENSOR_FEATURE_SET_ENG_REGISTER:
			SENSORDB("SENSOR_FEATURE_SET_ENG_REGISTER\n");
            SensorRegNumber=ENGINEER_END;
            for (i = 0; i < SensorRegNumber; i++)
            {
                T4K28YUVSensorReg[i].Addr = *pFeatureData32++;
                T4K28YUVSensorReg[i].Para = *pFeatureData32++;
            }
            break;

        case SENSOR_FEATURE_GET_ENG_REGISTER:
			SENSORDB("SENSOR_FEATURE_GET_ENG_REGISTER\n");
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen < (SensorRegNumber*sizeof(SENSOR_REG_STRUCT) + 4))
                return FALSE;
            *pFeatureData32++ = SensorRegNumber;
            for (i = 0; i < SensorRegNumber; i++)
            {
                *pFeatureData32++ = T4K28YUVSensorReg[i].Addr;
                *pFeatureData32++ = T4K28YUVSensorReg[i].Para;
            }
            break;

        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
			SENSORDB("SENSOR_FEATURE_GET_REGISTER_DEFAULT\n");
            if (*pFeatureParaLen >= sizeof(NVRAM_SENSOR_DATA_STRUCT)) {
                pSensorDefaultData->Version = NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId = T4K28_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, T4K28YUVSensorReg, sizeof(SENSOR_REG_STRUCT) * ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, T4K28YUVSensorCCT, sizeof(SENSOR_REG_STRUCT) * FACTORY_END_ADDR);
            } else
                return FALSE;
            *pFeatureParaLen = sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;

        case SENSOR_FEATURE_GET_CONFIG_PARA:
			SENSORDB("SENSOR_FEATURE_GET_CONFIG_PARA\n");
            memcpy(pSensorConfigData, &T4K28YUVSensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;

        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
			SENSORDB("SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR\n");
            T4K28YUV_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
			SENSORDB("SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA\n");
            T4K28YUV_sensor_to_camera_para();
            break;

        case SENSOR_FEATURE_GET_GROUP_COUNT:
			SENSORDB("SENSOR_FEATURE_GET_GROUP_COUNT\n");
            *pFeatureReturnPara32++ = T4K28YUV_get_sensor_group_count();
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_GROUP_INFO:
			SENSORDB("SENSOR_FEATURE_GET_GROUP_INFO\n");
            T4K28YUV_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ITEM_INFO:
			SENSORDB("SENSOR_FEATURE_GET_ITEM_INFO\n");
            T4K28YUV_get_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
			SENSORDB("SENSOR_FEATURE_SET_ITEM_INFO\n");
            T4K28YUV_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			SENSORDB("SENSOR_FEATURE_GET_ENG_INFO\n");
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat = SENSOR_OUTPUT_FORMAT_YUYV;
            *pFeatureParaLen = sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
			SENSORDB("SENSOR_FEATURE_GET_LENS_DRIVER_ID\n");
            *pFeatureReturnPara32 = LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
			SENSORDB("SENSOR_FEATURE_INITIALIZE_AF\n");
            SENSORDB("T4K28_FOCUS_Init\n");
            break;

        case SENSOR_FEATURE_CONSTANT_AF:
			SENSORDB("SENSOR_FEATURE_CONSTANT_AF\n");
            SENSORDB("T4K28_FOCUS_Constant_Focus\n");
	    	printk("kiwi-T4K28_FOCUS_Constant_Focus\n");
            break;

        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
			SENSORDB("SENSOR_FEATURE_MOVE_FOCUS_LENS\n");
            SENSORDB("T4K28_FOCUS_AD5820_Move_to %d\n", *pFeatureData16);
            break;

        case SENSOR_FEATURE_GET_AF_STATUS:
			SENSORDB("SENSOR_FEATURE_GET_AF_STATUS\n");
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_AF_INF:
			SENSORDB("SENSOR_FEATURE_GET_AF_INF\n");
            *pFeatureParaLen = 4;
            break;

        case SENSOR_FEATURE_GET_AF_MACRO:
			SENSORDB("SENSOR_FEATURE_GET_AF_MACRO\n");
            *pFeatureParaLen = 4;
            break;



		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
			   T4K28YUVMIPI_FlashTriggerCheck(pFeatureData32);
			   printk("[T4K28] F_GET_TRIGGER_FLASHLIGHT_INFO: %d\n", pFeatureData32);
			   break;



        case SENSOR_FEATURE_SET_VIDEO_MODE:
			SENSORDB("SENSOR_FEATURE_SET_VIDEO_MODE\n");
            T4K28YUVSetVideoMode(*pFeatureData16);
            break;

        case SENSOR_FEATURE_SET_YUV_CMD:
			SENSORDB("SENSOR_FEATURE_SET_YUV_CMD\n");
            T4K28YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
            break;

        case SENSOR_FEATURE_CHECK_SENSOR_ID:
			SENSORDB("SENSOR_FEATURE_CHECK_SENSOR_ID\n");
            T4K28YUVGetSensorID(pFeatureReturnPara32);
            break;

        case SENSOR_FEATURE_SINGLE_FOCUS_MODE:
            SENSORDB("SENSOR_FEATURE_SINGLE_FOCUS_MODE\n");
            break;

        case SENSOR_FEATURE_CANCEL_AF:
            SENSORDB("SENSOR_FEATURE_CANCEL_AF\n");
            break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			 //1 TODO
			 //SENSORDB("[HI251] F_SET_TEST_PATTERN: FAIL: NOT Support\n");
			 T4K28YUVSetTestPatternMode((BOOL)*pFeatureData16);
			 break;

		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing
			 *pFeatureReturnPara32= T4K28YUV_TEST_PATTERN_CHECKSUM;
			 *pFeatureParaLen=4;
			 break;

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			SENSORDB("SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS\n");
			*pFeatureReturnPara32 = 0;
			*pFeatureParaLen = 4;
			printk("AF *pFeatureReturnPara32 = %d\n", *pFeatureReturnPara32);
	     	break;

		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			SENSORDB("SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS\n");
			*pFeatureReturnPara32 = 0;
			*pFeatureParaLen = 4;
			printk("AE *pFeatureReturnPara32 = %d\n", *pFeatureReturnPara32);
	        break;

		case SENSOR_FEATURE_GET_EXIF_INFO:
			SENSORDB("SENSOR_FEATURE_GET_EXIF_INFO\n");
			SENSORDB("EXIF addr = 0x%x\n",*pFeatureData32);
			T4K28GetExifInfo(*pFeatureData32);
			break;

		case SENSOR_FEATURE_SET_AE_WINDOW:
            SENSORDB("SENSOR_FEATURE_SET_AE_WINDOW\n");
            printk("hwj SENSOR_FEATURE_SET_AE_WINDOW");
            SENSORDB("get zone addr = 0x%x\n", *pFeatureData32);
            break;
        case SENSOR_FEATURE_SET_AF_WINDOW:
            SENSORDB("SENSOR_FEATURE_SET_AF_WINDOW\n");
            printk("hwj SENSOR_FEATURE_SET_AF_WINDOW");
            SENSORDB("get zone addr = 0x%x\n", *pFeatureData32);
            break;
			//xb.pang
		case SENSOR_FEATURE_SET_YUV_3A_CMD:
			SENSORDB("SENSOR_FEATURE_SET_YUV_3A_CMD\n");
			T4K28YUV_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
			break;

		//                                                                 
		case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
			{
				UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
				UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

				SENSORDB("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
				*pHorFOV = 55;  // HorFOV = 54.7
				*pVerFOV = 42;  // VerFOV = 42.3
				*pFeatureParaLen = 8;
			}
			break;
		//                                                                 

        default:
            break;
    }
    return ERROR_NONE;
}

SENSOR_FUNCTION_STRUCT	SensorFuncT4K28YUV=
{
    T4K28YUVOpen,
    T4K28YUVGetInfo,
    T4K28YUVGetResolution,
    T4K28YUVFeatureControl,
    T4K28YUVControl,
    T4K28YUVClose
};

UINT32 T4K28_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc != NULL)
        *pfFunc = &SensorFuncT4K28YUV;
    return ERROR_NONE;
}
