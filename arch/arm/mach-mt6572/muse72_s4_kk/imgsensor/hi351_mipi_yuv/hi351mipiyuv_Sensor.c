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
 * Filename:
 * ---------
 *   sensor.c
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Source code of Sensor driver
 *
 *
 * Author:
 * -------
 *   Qihao Geng (mtk70548)
 *
 *============================================================================
 *             HISTORY
 * Below this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 * $Revision:$
 * $Modtime:$
 * $Log:$
 *
 * 07 31 2012 qihao.geng
 * NULL
 * Solve the ZSD capture's mirror/flip issue.
 *
 * 07 30 2012 qihao.geng
 * NULL
 * 1. support burst i2c write to save entry time.
 * 2. solve the view angle difference issue.
 * 3. video fix frame rate is OK and ready to release.
 *
 * 07 25 2012 qihao.geng
 * NULL
 * Increase the ZSD and read/write shutter/gain function.
 *
 * 07 20 2012 qihao.geng
 * NULL
 * HI351 MIPI Sensor Driver Check In 1st Versoin, can preview/capture.
 *
 * 07 19 2012 qihao.geng
 * NULL
 * HI351 MIPI YUV Sensor Driver Add, 1st version.
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

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "hi351mipiyuv_Sensor.h"
#include "hi351mipiyuv_Camera_Sensor_para.h"
#include "hi351mipiyuv_CameraCustomized.h"


/* Global Valuable */
#define PARAM_INIT_VALUE        0xFFFF

static kal_uint32 HI351_zoom_factor = 0;

static kal_bool HI351_gPVmode = KAL_TRUE; //PV size or Full size
static kal_bool HI351_VEDIO_encode_mode = KAL_FALSE; //Picture(Jpeg) or Video(Mpeg4)
static kal_bool HI351_sensor_cap_state = KAL_FALSE; //Preview or Capture
static kal_bool HI351_ZSD_Preview_state = KAL_FALSE; //Preview or Capture


UINT32 HI351_PV_dummy_pixels = 0, HI351_PV_dummy_lines = 0;
UINT32 HI351_FULL_dummy_pixels = 0, HI351_FULL_dummy_lines = 0;

kal_uint8 HI351_Banding_setting = AE_FLICKER_MODE_50HZ;

static kal_uint16  HI351_PV_Shutter = 0;

kal_uint32 HI351_capture_pclk_in_M=520,HI351_preview_pclk_in_M=390,HI351_isp_master_clock=0;
static kal_uint32  HI351_preview_pclk = 0, HI351_capture_pclk = 0;


UINT32 HI351_sensor_pclk=390;

UINT8 HI351_PixelClockDivider=0;

//static kal_bool HI351_AWB_ENABLE = KAL_TRUE;
//static kal_bool HI351_AE_ENABLE = KAL_TRUE;

static kal_uint32 Capture_Shutter = 0;
static kal_uint32 Capture_Gain = 0;
static kal_uint32 Capture_delay_in_Flash = 0;
static kal_uint32 Capture_delay = 0;

static kal_uint16 HI351_para_scene;
static kal_uint16 HI351_para_wb;
static kal_uint16 HI351_para_effect;

// rewriting prev exp and gain
static UINT8 HI351_prev_exp1 = 0;
static UINT8 HI351_prev_exp2 = 0;
static UINT8 HI351_prev_exp3 = 0;
static UINT8 HI351_prev_exp4 = 0;
static UINT8 HI351_prev_gain = 0;

typedef struct _HI351_EXIF_INFO_{
	kal_uint32 awb;
	kal_uint32 iso;
	kal_uint32 iso_auto;
	kal_uint32 flashlight;
}HI351_EXIF_INFO;

HI351_EXIF_INFO Exif_info;

typedef enum {
	HI351_60HZ,
	HI351_50HZ,
	HI351_HZ_MAX_NUM,
} HI351AntibandingType;

static int hi351_antibanding = HI351_50HZ;

//                                                                             
static bool HI351_Flash_OnOff = 0;
static kal_uint16 HI351_flash_mode = FLASH_MODE_FORCE_OFF;
//                                                                             

static DEFINE_SPINLOCK(hi351mipi_drv_lock);

MSDK_SENSOR_CONFIG_STRUCT HI351SensorConfigData;

MSDK_SCENARIO_ID_ENUM HI351CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId) ;
extern int iMultiWriteReg(u8 *pData, u16 lens);

kal_uint16 HI351_write_cmos_sensor(kal_uint8 addr, kal_uint8 para)
{
	char puSendCmd[2] = {(char)(addr & 0xFF) , (char)(para & 0xFF)};

	iWriteRegI2C(puSendCmd, 2, HI351_WRITE_ID);
	//HI351_TRACE("[HI351]:write addr0x%x=0x%x\n",addr,para);

	return 0;
}
kal_uint16 HI351_read_cmos_sensor(kal_uint8 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd = { (char)(addr & 0xFF) };
	iReadRegI2C(&puSendCmd , 1, (u8*)&get_byte, 1, HI351_WRITE_ID);

	return get_byte;
}

kal_uint16 HI351_burst_write_cmos_sensor(kal_uint8 *pData, kal_uint32 bytes)
{
	//iBurstWriteReg(pData, bytes, HI351_WRITE_ID);
	//iMultiWriteReg(pData, bytes);
	iBurstWriteReg(pData, bytes, HI351_WRITE_ID);

	return 0;
}
static int HI351_table_write_cmos_sensor(HI351_I2C_REG_STRUCT* para, kal_uint32 len)
{
	HI351_I2C_REG_STRUCT *init_reg = (HI351_I2C_REG_STRUCT *)para;
	UINT32 total_delay_ms = 0, error_cnt = 0;
	UINT8 burst_buff[255] = {0};
	UINT32 i=0, j=0;

	while (i < len)
	{
		if (BYTE_LEN == init_reg->type)
		{
			HI351_write_cmos_sensor(init_reg->addr, init_reg->val);
			i++;
			init_reg++;
		}
		else if (BURST_TYPE == init_reg->type)
		{
			j = 0;

			burst_buff[j] = init_reg->addr;
			j++;

			do
			{
				burst_buff[j] = init_reg->val;
				j++;

				/* I2C burst write max length is 255 bytes. */
				if (j >= (255-8))
				{
					init_reg++;
					i++;
					break;
				}
				init_reg++;
				i++;
			} while (BURST_TYPE == init_reg->type);

			/* Burst write the continue address register. */
			HI351_burst_write_cmos_sensor(burst_buff, j);
		}
		else if (DELAY_TYPE == init_reg->type)
		{
			if ((init_reg->addr == 0x03) && (init_reg->val == 0xFE))
			{
				init_reg++;
				i++;
				continue;
			}
			else if (init_reg->addr == 0xFE)
			{
				mdelay(init_reg->val);
				total_delay_ms += init_reg->val;
				init_reg++;
				i++;
			}
			else
			{
				error_cnt++;
				HI351_TRACE("[HI351]Failed parse setting!!! addr=%x, val=%x, type=%x \n", \
						init_reg->addr, init_reg->val, init_reg->type);
				init_reg++;
				i++;
			}
		}
		else
		{
			error_cnt++;
			HI351_TRACE("[HI351]Failed parse setting!!! addr=%x, val=%x, type=%x \n", \
					init_reg->addr, init_reg->val, init_reg->type);
			init_reg++;
			i++;
		}
	}

	HI351_TRACE("[HI351] Done! Apply setting=%d, setting_len=%d, total delay=%d(ms), error_cnt=%d \n", \
			i, len, total_delay_ms, error_cnt);
	return 0;
}

// samjinjang 20130311 move from HI351PreviewSetting to  sleep off function
void HI351_set_Sleep_off()
{
	HI351_TRACE("[HI351]:HI351_set_Sleep_off \n");

	HI351_write_cmos_sensor(0x03, 0x00);
	HI351_write_cmos_sensor(0x01, 0xf0); //sleep off

	HI351_write_cmos_sensor(0x03, 0xcf); //Adaptive On
	HI351_write_cmos_sensor(0x10, 0xaf);
	HI351_write_cmos_sensor(0x03, 0xc0);
	HI351_write_cmos_sensor(0x33, 0x00);
	HI351_write_cmos_sensor(0x32, 0x01); //DMA On

	mdelay(15); // STEVE DELAY
}

void HI351_set_dummy(kal_uint16 pixels, kal_uint16 lines)
{
	kal_uint8 temp_reg1, temp_reg2;
	kal_uint16 temp_reg;

	/* Increase the minimum default h-blanking & v-blanking */
	pixels += 0x6E;
	lines + 0xA4;
#if 1
	// HI351_write_cmos_sensor(0x03, 0x00); //dummy // Steve delete
	mdelay(1);  //Steve delay
#else
	HI351_write_cmos_sensor(0x03, 0x00);
	HI351_write_cmos_sensor(0x50, (pixels>>8)&0xff);
	HI351_write_cmos_sensor(0x51, (pixels>>0)&0xff);
	HI351_write_cmos_sensor(0x52, (lines>>8)&0xff);
	HI351_write_cmos_sensor(0x53, (lines>>0)&0xff);
#endif
}    /* HI351_set_dummy */

kal_uint32 HI351_read_exp_unit(void)
{
	kal_uint32 exp_unit;
	kal_uint32 shutter = 0;
	kal_uint32 opclk = 0;

	HI351_write_cmos_sensor(0x03, 0x20);
	if(HI351_read_cmos_sensor(0xAB) & 0x10)
	{
		exp_unit = 0;
		exp_unit |= HI351_read_cmos_sensor(0x30)<<16;
		exp_unit |= HI351_read_cmos_sensor(0x31)<<8;
		exp_unit |= HI351_read_cmos_sensor(0x32);
	}
	else
	{
		exp_unit = 0;
		exp_unit |= HI351_read_cmos_sensor(0x33)<<16;
		exp_unit |= HI351_read_cmos_sensor(0x34)<<8;
		exp_unit |= HI351_read_cmos_sensor(0x35);
	}

	return exp_unit;
}

kal_uint32 HI351_read_gain(void)
{
	kal_uint32 sensor_gain = 0;
	kal_uint16 temp_reg = 0;

	/* Analog gain = 1 / 32 * B[7:0]. 0x20(1X) ~ 0x40(2X) */
	HI351_write_cmos_sensor(0x03, 0x20);
	temp_reg = HI351_read_cmos_sensor(0xAB);

	/* sensor_gain is 1024 base */
	sensor_gain = (1024 * temp_reg) / 32;

	return sensor_gain;
}  /* HI351_read_gain */

kal_uint32 HI351_read_shutter(void)
{
	kal_uint32 shutter = 0;
	kal_uint32 opclk = 0;

	HI351_write_cmos_sensor(0x03, 0x20);
	shutter = HI351_read_cmos_sensor(0xA4)<<24;
	shutter |= HI351_read_cmos_sensor(0xA5)<<16;
	shutter |= HI351_read_cmos_sensor(0xA6)<<8;
	shutter |= HI351_read_cmos_sensor(0xA7);

	//    opclk = HI351_preview_pclk / 2;     /* OPCLK = (1/2)PCLK */
	//    shutter = shutter  / opclk;         /* After this line, shutter unit is 10us, before is opclk/ */

	return shutter;
}    /* HI351_read_shutter */

void HI351_write_gain(kal_uint32 sensor_gain)
{
	kal_uint16 temp_reg = 0;
	kal_uint16 manual_gain_min, manual_gain_max;

	HI351_write_cmos_sensor(0x03, 0x20);
	HI351_prev_gain = HI351_read_cmos_sensor(0xAB);

	HI351_write_cmos_sensor(0x03, 0x20);
	manual_gain_min = HI351_read_cmos_sensor(0x52);
	manual_gain_max = HI351_read_cmos_sensor(0x51);

	/* sensor_gain is 1024 base */
	temp_reg = (sensor_gain * 32) / 1024;

	if(temp_reg < manual_gain_min)
	{
		temp_reg = manual_gain_min;
	}
	else if(temp_reg > manual_gain_max)
	{
		temp_reg = manual_gain_max;
	}

	HI351_TRACE("[HI351]:Write sensor gain = %x, temp_reg = %x\n", sensor_gain, temp_reg);

	/* Analog gain = 1 / 32 * B[7:0]. 0x20(1X) ~ 0x40(2X) */
	HI351_write_cmos_sensor(0x03, 0x20);
	HI351_write_cmos_sensor(0x50, temp_reg);
}  /* HI351_write_gain */

void HI351_write_shutter(kal_uint32 shutter)
{
	kal_uint32 opclk = 0;
	kal_uint32 manual_min_shutter, manual_max_shutter;

	HI351_TRACE("[HI351]:Write shutter = %x\n", shutter);

	//    opclk = HI351_preview_pclk / 2;     /* OPCLK = (1/2)PCLK */
	//    shutter = shutter * opclk;         /* After this line, shutter unit is opclk, before is 10us/ */

	HI351_write_cmos_sensor(0x03, 0x20);
	HI351_prev_exp1 = HI351_read_cmos_sensor(0xA4);
	HI351_prev_exp2 = HI351_read_cmos_sensor(0xA5);
	HI351_prev_exp3 = HI351_read_cmos_sensor(0xA6);
	HI351_prev_exp4 = HI351_read_cmos_sensor(0xA7);

	HI351_write_cmos_sensor(0x03, 0x20);
	manual_min_shutter = 0;
	manual_min_shutter |= HI351_read_cmos_sensor(0x28)<<16;
	manual_min_shutter |= HI351_read_cmos_sensor(0x29)<<8;
	manual_min_shutter |= HI351_read_cmos_sensor(0x2A);


	HI351_write_cmos_sensor(0x03, 0x20);
	manual_max_shutter = HI351_read_cmos_sensor(0x24)<<24;
	manual_max_shutter |= HI351_read_cmos_sensor(0x25)<<16;
	manual_max_shutter |= HI351_read_cmos_sensor(0x26)<<8;
	manual_max_shutter |= HI351_read_cmos_sensor(0x27);

	if(shutter < manual_min_shutter)
	{
		shutter = manual_min_shutter;
	}
	else if(shutter > manual_max_shutter)
	{
		shutter = manual_max_shutter;
	}

	HI351_write_cmos_sensor(0x03, 0xc4); //AE off
	if (hi351_antibanding == HI351_50HZ)
	{
		HI351_write_cmos_sensor(0x10, 0x68);  // STEVE AE OFF   (50Hz : 0x68, 60hz : 0x60)
	}
	else
	{
		HI351_write_cmos_sensor(0x10, 0x60);
	}

	mdelay(20);

	HI351_write_cmos_sensor(0x03, 0x20);
	HI351_write_cmos_sensor(0x20, (shutter >> 24 & 0xFF));
	HI351_write_cmos_sensor(0x21, (shutter >> 16 & 0xFF));
	HI351_write_cmos_sensor(0x22, (shutter >> 8 & 0xFF));
	HI351_write_cmos_sensor(0x23, shutter & 0xFF);
}    /* HI351_write_shutter */
#if 1
void HI351YUVSensorInitialSetting(void)
{
	kal_uint32 len=0;

	HI351_TRACE("[HI351]:hi351_antibanding = %d\n", hi351_antibanding);
	len = sizeof(HI351_Initialize_Setting[hi351_antibanding]) / sizeof(HI351_Initialize_Setting[hi351_antibanding][0]);

	HI351_prev_gain = 0;
	HI351_prev_exp1 = 0;
	HI351_prev_exp2 = 0;
	HI351_prev_exp3 = 0;
	HI351_prev_exp4 = 0;

	HI351_table_write_cmos_sensor(HI351_Initialize_Setting[hi351_antibanding],len);
	Capture_delay_in_Flash = Capture_delay;
}
#endif


/* Register setting from capture to preview. */
void HI351PreviewSetting(void)
{
	kal_uint32 len=0;

	if (hi351_antibanding == HI351_50HZ)
	{

		HI351_write_cmos_sensor(0x03, 0x00);
		HI351_write_cmos_sensor(0x01, 0xf1); //Sleep on

		HI351_write_cmos_sensor(0x03, 0x30); //DMA&Adaptive Off
		HI351_write_cmos_sensor(0x36, 0xa3);

		mdelay(5);

		HI351_write_cmos_sensor(0x03, 0xc4); //AE off
		HI351_write_cmos_sensor(0x10, 0x68); // STEVE AE OFF   (50Hz : 0x68, 60hz : 0x60)

		if(HI351_prev_exp1 != 0 || HI351_prev_exp2 != 0 || HI351_prev_exp3 != 0 || HI351_prev_exp4 != 0)
		{
			HI351_write_cmos_sensor(0x03, 0x20);
			HI351_write_cmos_sensor(0x20, HI351_prev_exp1);
			HI351_write_cmos_sensor(0x21, HI351_prev_exp2);
			HI351_write_cmos_sensor(0x22, HI351_prev_exp3);
			HI351_write_cmos_sensor(0x23, HI351_prev_exp4);
		}

		if(HI351_prev_gain != 0)
		{
			HI351_write_cmos_sensor(0x03, 0x20);
			HI351_write_cmos_sensor(0x50, HI351_prev_gain);
		}
		len = sizeof(HI351_Preview_Setting[hi351_antibanding]) / sizeof(HI351_Preview_Setting[hi351_antibanding][0]);
		HI351_table_write_cmos_sensor(HI351_Preview_Setting[hi351_antibanding],len);
	}
	else	// 60Hz == 0
	{

		HI351_write_cmos_sensor(0x03, 0x00);
		HI351_write_cmos_sensor(0x01, 0xf1); //Sleep on

		HI351_write_cmos_sensor(0x03, 0x30); //DMA&Adaptive Off
		HI351_write_cmos_sensor(0x36, 0xa3);

		mdelay(5);

		HI351_write_cmos_sensor(0x03, 0xc4); //AE off
		HI351_write_cmos_sensor(0x10, 0x60); // STEVE AE OFF   (50Hz : 0x68, 60hz : 0x60)

		if(HI351_prev_exp1 != 0 || HI351_prev_exp2 != 0 || HI351_prev_exp3 != 0 || HI351_prev_exp4 != 0)
		{
			HI351_write_cmos_sensor(0x03, 0x20);
			HI351_write_cmos_sensor(0x20, HI351_prev_exp1);
			HI351_write_cmos_sensor(0x21, HI351_prev_exp2);
			HI351_write_cmos_sensor(0x22, HI351_prev_exp3);
			HI351_write_cmos_sensor(0x23, HI351_prev_exp4);
		}

		if(HI351_prev_gain != 0)
		{
			HI351_write_cmos_sensor(0x03, 0x20);
			HI351_write_cmos_sensor(0x50, HI351_prev_gain);
		}
		len = sizeof(HI351_Preview_Setting[hi351_antibanding]) / sizeof(HI351_Preview_Setting[hi351_antibanding][0]);
		HI351_table_write_cmos_sensor(HI351_Preview_Setting[hi351_antibanding],len);
	}
}

void HI351FullPreviewSetting(void)
{
	HI351_I2C_REG_STRUCT *init_reg = NULL;//HI351_Initialize_Setting;
	UINT32 i=0;
	kal_uint32 len=0;
	len = sizeof(HI351_FullPreview_Setting) / sizeof(HI351_FullPreview_Setting[0]);

	while (i < len)
	{
		init_reg = &HI351_FullPreview_Setting[i];
		if(BYTE_LEN == init_reg->type)
		{
			HI351_write_cmos_sensor(init_reg->addr, init_reg->val);
			i++;
		}
		else if(DELAY_TYPE != init_reg->type)
		{
			if ((init_reg->addr == 0x03) && (init_reg->val == 0xFE))
			{
				i++;
				continue;
			}
			else if (init_reg->addr == 0xFE)
			{
				mdelay(init_reg->val);
				i++;
			}
			else
			{
				HI351_TRACE("[HI351]Failed parse setting!!! addr=%x, val=%x, type=%x \n", \
						init_reg->addr, init_reg->val, init_reg->type);
				i++;
			}
		}
		else
		{
			HI351_TRACE("[HI351]Failed parse setting!!! addr=%x, val=%x, type=%x \n", \
					init_reg->addr, init_reg->val, init_reg->type);
			i++;
		}
	}
}

void HI351CaptureSetting(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_Capture_Setting[hi351_antibanding]) / sizeof(HI351_Capture_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_Capture_Setting[hi351_antibanding],len);
}


/*****************************************************************************/
/* Windows Mobile Sensor Interface */
/*****************************************************************************/
/*************************************************************************
 * FUNCTION
 *   HI351GetSensorID
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
extern int soc_antibanding; //                                                                     
extern int g_lge_camera; //                                                                                     
UINT32 HI351GetSensorID(UINT32 *sensorID)
{
	if(g_lge_camera==1) //                                                                      
	{
		*sensorID =0xffffffff;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	volatile signed char i = 0;
	int retry = 3; //                                                                     

	HI351_write_cmos_sensor(0x03, 0x00);    /* Page 0 */
	//Software Reset
	HI351_write_cmos_sensor(0x01, 0xF1);
	HI351_write_cmos_sensor(0x01, 0xF3);
	HI351_write_cmos_sensor(0x01, 0xF1);
	mdelay(5);

	spin_lock(&hi351mipi_drv_lock);
	HI351_zoom_factor = 0;
	spin_unlock(&hi351mipi_drv_lock);
	//                                                                       
	do
	{
		*sensorID = HI351_read_cmos_sensor(0x04);
		HI351_TRACE("[HI351]:Get Sensor ID:0x%x\n", *sensorID);

		mdelay(5);
		if(*sensorID == HI351_SENSOR_ID)
		{
			break;
		}
		retry--;
	}while(retry > 0);
	//                                                                       

	if(*sensorID != HI351_SENSOR_ID)
	{
		*sensorID = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	return ERROR_NONE;
}

/*************************************************************************
 * FUNCTION
 *	HI351Open
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
UINT32 HI351Open(void)
{
	UINT32 sensor_id = 0;

	HI351_write_cmos_sensor(0x03, 0x00);    /* Page 0 */
	//Software Reset
	HI351_write_cmos_sensor(0x01, 0xF1);
	HI351_write_cmos_sensor(0x01, 0xF3);
	HI351_write_cmos_sensor(0x01, 0xF1);
	mdelay(5);

	spin_lock(&hi351mipi_drv_lock);
	HI351_zoom_factor = 0;
	spin_unlock(&hi351mipi_drv_lock);
	sensor_id = HI351_read_cmos_sensor(0x04);
	HI351_TRACE("[HI351]:Open Sensor ID:0x%x\n", sensor_id);
	mdelay(5);
	hi351_antibanding = soc_antibanding; //                                                                     
	HI351_TRACE("[HI351]:antibanding = %d \n", hi351_antibanding);	// samjinjang 0 => 60hz , 1 => 50hz

	if(sensor_id != HI351_SENSOR_ID)
	{
		return ERROR_SENSOR_CONNECT_FAIL;
	}

	//HI351_TRACE("[HI351]:Start apply the initialize setting\n");
	HI351YUVSensorInitialSetting();
	//HI351_TRACE("[HI351]:Apply the initialize setting done\n");

	return ERROR_NONE;
}	/* HI351Open() */

/*************************************************************************
 * FUNCTION
 *	HI351Close
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
UINT32 HI351Close(void)
{
	return ERROR_NONE;
}	/* HI351Close() */

#if 0 	// samjinjang It does not need to fuction
/*************************************************************************
 * FUNCTION
 *	HI351_set_ae_enable
 *
 * DESCRIPTION
 *	Enable/disable the auto exposure.
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
void HI351_set_ae_enable(kal_bool AE_enable)
{
	HI351_TRACE("[HI351]:HI351_set_ae_enable: AE_enable=%d\n", AE_enable);
#if 1
	if (AE_enable == KAL_TRUE)
	{
		//HI351_write_cmos_sensor(0x03, 0x00); //Dummy
		mdelay(1);  // STEVE DELAY
	}
	else
	{
		//HI351_write_cmos_sensor(0x03, 0x00); //Dummy
		mdelay(1);  // STEVE DELAY
	}
#else
	if (AE_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0xc4); //AE en
		HI351_write_cmos_sensor(0x10, 0xe1);
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0xc4); //AE off
		HI351_write_cmos_sensor(0x10, 0x60);
	}
#endif
}

/*************************************************************************
 * FUNCTION
 *	HI351_set_awb_enable
 *
 * DESCRIPTION
 *	Enable/disable the auto white balance.
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
void HI351_set_awb_enable(kal_bool AWB_enable)
{
	HI351_TRACE("[HI351]:HI351_set_awb_enable: AWB_enable=%d\n", AWB_enable);
#if 1
	if (AWB_enable == KAL_TRUE)
	{
		//HI351_write_cmos_sensor(0x03, 0x00); //Dummy
		mdelay(1);  // STEVE DELAY
	}
	else
	{
		//HI351_write_cmos_sensor(0x03, 0x00); //Dummy
		mdelay(1);  // STEVE DELAY
	}
#else
	if (AWB_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0xc5); //AWB en
		HI351_write_cmos_sensor(0x10, 0xb0);
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0xc5); //AWB off
		HI351_write_cmos_sensor(0x10, 0x30);
	}
#endif
}
#endif

/*************************************************************************
 * FUNCTION
 *	HI351_set_mirror_flip
 *
 * DESCRIPTION
 *	mirror flip setting.
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
BOOL HI351_set_mirror_flip(kal_uint8 image_mirror)
{
	kal_uint8 iTemp = 0;

	/* 0x11 [page mode 0]: VDOCTL2 [default=0x90, r/w]
	   B[1] Vertical Flip Function (0:OFF, 1:ON)
	   B[0] Horizontal Flip Function (0:OFF, 1:ON)  */
	//HI351_write_cmos_sensor(0x03, 0x00); // Steve delete
	//iTemp = HI351_read_cmos_sensor(0x11); // Steve delete

	HI351_TRACE("[HI351]:Set start Mirror/Flip image_mirror=%d, iTemp=%x\n", image_mirror, iTemp);
	iTemp &= 0xFC;

	switch (image_mirror)
	{
		case IMAGE_NORMAL:
			//iTemp |= 0x00;
			iTemp |= 0x03;
			break;
		case IMAGE_H_MIRROR:
			iTemp |= 0x01;
			break;
		case IMAGE_V_MIRROR:	//Flip Register 0x04[6] and 0x04[4] (FF = 01)
			iTemp |= 0x02;
			break;
		case IMAGE_HV_MIRROR:
			//iTemp |= 0x03;
			iTemp |= 0x00;
			break;
		default:
			//iTemp |= 0x00;      /* Default, mirror & flip off. */
			iTemp |= 0x03;
			break;
	}

	HI351_TRACE("[HI351]:Set end Mirror/Flip image_mirror=%d, iTemp=%x\n", image_mirror, iTemp);

	//HI351_write_cmos_sensor(0x11, iTemp); // Steve delete
	mdelay(1);  // STEVE DELAY

	return TRUE;
} /* HI351_set_mirror_flip */


/*************************************************************************
 * FUNCTION
 *	HI351_set_param_wb
 *
 * DESCRIPTION
 *	wb setting.
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
BOOL HI351_set_param_wb(UINT16 para)
{
	HI351_TRACE("[HI351]:HI351_set_param_wb: =%d\n", para);

	Exif_info.awb = para;

	switch (para)
	{

		case AWB_MODE_OFF:
		case AWB_MODE_AUTO:
			mdelay(10);
			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0x30); //AWB Off

			//AWB target RG angle
			HI351_write_cmos_sensor(0xaa, 0x32);//aInRgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xab, 0x28);//aInRgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xac, 0x14);//aInRgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xad, 0x0a);//aInRgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xae, 0x00);//aInRgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xaf, 0x81);//aInRgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb0, 0x84);//aInRgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb1, 0x85);//aInRgTgtOfs_a07_n00

			//AWB target BG angle
			HI351_write_cmos_sensor(0xb2, 0xb2);//aInBgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xb3, 0xa8);//aInBgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xb4, 0x94);//aInBgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xb5, 0x8a);//aInBgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xb6, 0x00);//aInBgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xb7, 0x01);//aInBgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb8, 0x04);//aInBgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb9, 0x05);//aInBgTgtOfs_a07_n00

			HI351_write_cmos_sensor(0x03, 0xc6);
			HI351_write_cmos_sensor(0x18, 0x40);//bInRgainMin_a00_n00
			HI351_write_cmos_sensor(0x19, 0xf0);//bInRgainMax_a00_n00
			HI351_write_cmos_sensor(0x1a, 0x40);//bInBgainMin_a00_n00
			HI351_write_cmos_sensor(0x1b, 0xf0);//bInBgainMax_a00_n00

			HI351_write_cmos_sensor(0xb9, 0x66); // STEVE outdoor bluish //bOutRgainMin_a00_n00
			HI351_write_cmos_sensor(0xba, 0x88); // STEVE outdoor bluish //bOutRgainMax_a00_n00
			HI351_write_cmos_sensor(0xbb, 0x4c); // STEVE outdoor bluish //bOutBgainMin_a00_n00
			HI351_write_cmos_sensor(0xbc, 0x62); // STEVE outdoor bluish //bOutBgainMax_a00_n00

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0xb1);//AWB On

			///////////////////////////////////////////
			// D1 Page Adaptive R/B saturation
			///////////////////////////////////////////
			HI351_write_cmos_sensor(0x03, 0xd1);//Page d1

			//Cb
			HI351_write_cmos_sensor(0x2b, 0x90); //SATB_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x2c, 0x90); //SATB_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x2d, 0x90); //SATB_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x2e, 0x98); //SATB_03
			HI351_write_cmos_sensor(0x2f, 0x98); //SATB_04
			HI351_write_cmos_sensor(0x30, 0x98); //SATB_05
			HI351_write_cmos_sensor(0x31, 0xa0); //SATB_06
			HI351_write_cmos_sensor(0x32, 0xa0); //SATB_07
			HI351_write_cmos_sensor(0x33, 0xa0); //SATB_08
			HI351_write_cmos_sensor(0x34, 0xa8); //SATB_09
			HI351_write_cmos_sensor(0x35, 0xa8); //SATB_10
			HI351_write_cmos_sensor(0x36, 0xa8); //SATB_11

			//Cr

			HI351_write_cmos_sensor(0x37, 0x90);//SATR_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x38, 0x90);//SATR_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x39, 0x90);//SATR_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x3a, 0x98);//SATR_03
			HI351_write_cmos_sensor(0x3b, 0x98);//SATR_04
			HI351_write_cmos_sensor(0x3c, 0x98);//SATR_05
			HI351_write_cmos_sensor(0x3d, 0xa0);//SATR_06
			HI351_write_cmos_sensor(0x3e, 0xa0);//SATR_07
			HI351_write_cmos_sensor(0x3f, 0xa0);//SATR_08
			HI351_write_cmos_sensor(0x40, 0xa8);//SATR_09
			HI351_write_cmos_sensor(0x41, 0xa8);//SATR_10
			HI351_write_cmos_sensor(0x42, 0xa8);//SATR_11
			mdelay(5);
			break;

		case AWB_MODE_CLOUDY_DAYLIGHT:
			mdelay(10);

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0x30);

			//AWB target RG angle
			HI351_write_cmos_sensor(0xaa, 0x32);//aInRgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xab, 0x28);//aInRgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xac, 0x14);//aInRgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xad, 0x0a);//aInRgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xae, 0x00);//aInRgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xaf, 0x81);//aInRgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb0, 0x84);//aInRgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb1, 0x85);//aInRgTgtOfs_a07_n00

			//AWB target BG angle
			HI351_write_cmos_sensor(0xb2, 0xb2);//aInBgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xb3, 0xa8);//aInBgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xb4, 0x94);//aInBgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xb5, 0x8a);//aInBgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xb6, 0x00);//aInBgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xb7, 0x01);//aInBgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb8, 0x04);//aInBgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb9, 0x05);//aInBgTgtOfs_a07_n00

			HI351_write_cmos_sensor(0x03, 0xc6);
			HI351_write_cmos_sensor(0x18, 0x75);//bInRgainMin_a00_n00
			HI351_write_cmos_sensor(0x19, 0x8F);//bInRgainMax_a00_n00
			HI351_write_cmos_sensor(0x1a, 0x40);//bInBgainMin_a00_n00
			HI351_write_cmos_sensor(0x1b, 0x56);//bInBgainMax_a00_n00

			HI351_write_cmos_sensor(0xb9, 0x75);//bOutRgainMin_a00_n00
			HI351_write_cmos_sensor(0xba, 0x8F);//bOutRgainMax_a00_n00
			HI351_write_cmos_sensor(0xbb, 0x40);//bOutBgainMin_a00_n00
			HI351_write_cmos_sensor(0xbc, 0x56);//bOutBgainMax_a00_n00

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0xb1);

			///////////////////////////////////////////
			// D1 Page Adaptive R/B saturation
			///////////////////////////////////////////
			HI351_write_cmos_sensor(0x03, 0xd1);//Page d1

			//Cb
			HI351_write_cmos_sensor(0x2b, 0x90); //SATB_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x2c, 0x90); //SATB_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x2d, 0x90); //SATB_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x2e, 0x98); //SATB_03
			HI351_write_cmos_sensor(0x2f, 0x98); //SATB_04
			HI351_write_cmos_sensor(0x30, 0x98); //SATB_05
			HI351_write_cmos_sensor(0x31, 0xa0); //SATB_06
			HI351_write_cmos_sensor(0x32, 0xa0); //SATB_07
			HI351_write_cmos_sensor(0x33, 0xa0); //SATB_08
			HI351_write_cmos_sensor(0x34, 0xa8); //SATB_09
			HI351_write_cmos_sensor(0x35, 0xa8); //SATB_10
			HI351_write_cmos_sensor(0x36, 0xa8); //SATB_11

			//Cr

			HI351_write_cmos_sensor(0x37, 0x90);//SATR_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x38, 0x90);//SATR_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x39, 0x90);//SATR_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x3a, 0x98);//SATR_03
			HI351_write_cmos_sensor(0x3b, 0x98);//SATR_04
			HI351_write_cmos_sensor(0x3c, 0x98);//SATR_05
			HI351_write_cmos_sensor(0x3d, 0xa0);//SATR_06
			HI351_write_cmos_sensor(0x3e, 0xa0);//SATR_07
			HI351_write_cmos_sensor(0x3f, 0xa0);//SATR_08
			HI351_write_cmos_sensor(0x40, 0xa8);//SATR_09
			HI351_write_cmos_sensor(0x41, 0xa8);//SATR_10
			HI351_write_cmos_sensor(0x42, 0xa8);//SATR_11
			mdelay(5);
			break;

		case AWB_MODE_DAYLIGHT:
			mdelay(10);

			// case CAM_WB_DAYLIGHT:
			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0x30);//AWB Off

			//AWB target RG angle
			HI351_write_cmos_sensor(0xaa, 0x32);//aInRgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xab, 0x28);//aInRgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xac, 0x14);//aInRgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xad, 0x0a);//aInRgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xae, 0x00);//aInRgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xaf, 0x81);//aInRgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb0, 0x84);//aInRgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb1, 0x85);//aInRgTgtOfs_a07_n00

			//AWB target BG angle
			HI351_write_cmos_sensor(0xb2, 0xb2);//aInBgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xb3, 0xa8);//aInBgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xb4, 0x94);//aInBgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xb5, 0x8a);//aInBgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xb6, 0x00);//aInBgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xb7, 0x01);//aInBgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb8, 0x04);//aInBgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb9, 0x05);//aInBgTgtOfs_a07_n00

			HI351_write_cmos_sensor(0x03, 0xc6);
			HI351_write_cmos_sensor(0x18, 0x4B);//bInRgainMin_a00_n00
			HI351_write_cmos_sensor(0x19, 0x6A);//bInRgainMax_a00_n00
			HI351_write_cmos_sensor(0x1a, 0x5d);//bInBgainMin_a00_n00
			HI351_write_cmos_sensor(0x1b, 0x78);//bInBgainMax_a00_n00

			HI351_write_cmos_sensor(0xb9, 0x4B);//bOutRgainMin_a00_n00
			HI351_write_cmos_sensor(0xba, 0x6A);//bOutRgainMax_a00_n00
			HI351_write_cmos_sensor(0xbb, 0x5d);//bOutBgainMin_a00_n00
			HI351_write_cmos_sensor(0xbc, 0x78);//bOutBgainMax_a00_n00

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0xb1);//AWB On

			///////////////////////////////////////////
			// D1 Page Adaptive R/B saturation
			///////////////////////////////////////////
			HI351_write_cmos_sensor(0x03, 0xd1);//Page d1

			//Cb
			HI351_write_cmos_sensor(0x2b, 0x90); //SATB_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x2c, 0x90); //SATB_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x2d, 0x90); //SATB_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x2e, 0x98); //SATB_03
			HI351_write_cmos_sensor(0x2f, 0x98); //SATB_04
			HI351_write_cmos_sensor(0x30, 0x98); //SATB_05
			HI351_write_cmos_sensor(0x31, 0xa0); //SATB_06
			HI351_write_cmos_sensor(0x32, 0xa0); //SATB_07
			HI351_write_cmos_sensor(0x33, 0xa0); //SATB_08
			HI351_write_cmos_sensor(0x34, 0xa8); //SATB_09
			HI351_write_cmos_sensor(0x35, 0xa8); //SATB_10
			HI351_write_cmos_sensor(0x36, 0xa8); //SATB_11

			//Cr

			HI351_write_cmos_sensor(0x37, 0x90);//SATR_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x38, 0x90);//SATR_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x39, 0x90);//SATR_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x3a, 0x98);//SATR_03
			HI351_write_cmos_sensor(0x3b, 0x98);//SATR_04
			HI351_write_cmos_sensor(0x3c, 0x98);//SATR_05
			HI351_write_cmos_sensor(0x3d, 0xa0);//SATR_06
			HI351_write_cmos_sensor(0x3e, 0xa0);//SATR_07
			HI351_write_cmos_sensor(0x3f, 0xa0);//SATR_08
			HI351_write_cmos_sensor(0x40, 0xa8);//SATR_09
			HI351_write_cmos_sensor(0x41, 0xa8);//SATR_10
			HI351_write_cmos_sensor(0x42, 0xa8);//SATR_11
			mdelay(5);
			break;

		case AWB_MODE_INCANDESCENT:
			mdelay(10);
			// case CAM_WB_INCANDESCENCE:
			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0x30);//AWB Off

			//AWB target RG angle
			HI351_write_cmos_sensor(0xaa, 0x00);//aInRgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xab, 0x00);//aInRgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xac, 0x00);//aInRgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xad, 0x00);//aInRgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xae, 0x00);//aInRgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xaf, 0x00);//aInRgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb0, 0x00);//aInRgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb1, 0x00);//aInRgTgtOfs_a07_n00

			//AWB target BG angle
			HI351_write_cmos_sensor(0xb2, 0x00);//aInBgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xb3, 0x00);//aInBgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xb4, 0x00);//aInBgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xb5, 0x00);//aInBgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xb6, 0x00);//aInBgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xb7, 0x00);//aInBgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb8, 0x00);//aInBgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb9, 0x00);//aInBgTgtOfs_a07_n00

			HI351_write_cmos_sensor(0x03, 0xc6);
			HI351_write_cmos_sensor(0x18, 0x3C);//bInRgainMin_a00_n00
			HI351_write_cmos_sensor(0x19, 0x59);//bInRgainMax_a00_n00
			HI351_write_cmos_sensor(0x1a, 0x88);//bInBgainMin_a00_n00
			HI351_write_cmos_sensor(0x1b, 0xA0);//bInBgainMax_a00_n00

			HI351_write_cmos_sensor(0xb9, 0x3C);//bOutRgainMin_a00_n00
			HI351_write_cmos_sensor(0xba, 0x59);//bOutRgainMax_a00_n00
			HI351_write_cmos_sensor(0xbb, 0x88);//bOutBgainMin_a00_n00
			HI351_write_cmos_sensor(0xbc, 0xA0);//bOutBgainMax_a00_n00

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0xb1);//AWB On

			///////////////////////////////////////////
			// D1 Page Adaptive R/B saturation
			///////////////////////////////////////////
			HI351_write_cmos_sensor(0x03, 0xd1);//Page d1

			//Cb
			HI351_write_cmos_sensor(0x2b, 0x88); //SATB_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x2c, 0x88); //SATB_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x2d, 0x88); //SATB_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x2e, 0xd0); //SATB_03
			HI351_write_cmos_sensor(0x2f, 0xd0); //SATB_04
			HI351_write_cmos_sensor(0x30, 0xd0); //SATB_05
			HI351_write_cmos_sensor(0x31, 0xd0); //SATB_06
			HI351_write_cmos_sensor(0x32, 0xd0); //SATB_07
			HI351_write_cmos_sensor(0x33, 0xd0); //SATB_08
			HI351_write_cmos_sensor(0x34, 0xd0); //SATB_09
			HI351_write_cmos_sensor(0x35, 0xd0); //SATB_10
			HI351_write_cmos_sensor(0x36, 0xd0); //SATB_11

			//Cr

			HI351_write_cmos_sensor(0x37, 0x88);//SATR_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x38, 0x88);//SATR_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x39, 0x90);//SATR_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x3a, 0xb0);//SATR_03
			HI351_write_cmos_sensor(0x3b, 0xb0);//SATR_04
			HI351_write_cmos_sensor(0x3c, 0xb0);//SATR_05
			HI351_write_cmos_sensor(0x3d, 0xb0);//SATR_06
			HI351_write_cmos_sensor(0x3e, 0xb0);//SATR_07
			HI351_write_cmos_sensor(0x3f, 0xb0);//SATR_08
			HI351_write_cmos_sensor(0x40, 0xb8);//SATR_09
			HI351_write_cmos_sensor(0x41, 0xb8);//SATR_10
			HI351_write_cmos_sensor(0x42, 0xb8);//SATR_11
			mdelay(5);
			break;
		case AWB_MODE_FLUORESCENT:
			mdelay(10);
			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0x30);//AWB Off

			//AWB target RG angle
			HI351_write_cmos_sensor(0xaa, 0x32);//aInRgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xab, 0x28);//aInRgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xac, 0x14);//aInRgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xad, 0x0a);//aInRgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xae, 0x00);//aInRgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xaf, 0x81);//aInRgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb0, 0x84);//aInRgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb1, 0x85);//aInRgTgtOfs_a07_n00

			//AWB target BG angle
			HI351_write_cmos_sensor(0xb2, 0xb2);//aInBgTgtOfs_a00_n00
			HI351_write_cmos_sensor(0xb3, 0xa8);//aInBgTgtOfs_a01_n00
			HI351_write_cmos_sensor(0xb4, 0x94);//aInBgTgtOfs_a02_n00
			HI351_write_cmos_sensor(0xb5, 0x8a);//aInBgTgtOfs_a03_n00
			HI351_write_cmos_sensor(0xb6, 0x00);//aInBgTgtOfs_a04_n00
			HI351_write_cmos_sensor(0xb7, 0x01);//aInBgTgtOfs_a05_n00
			HI351_write_cmos_sensor(0xb8, 0x04);//aInBgTgtOfs_a06_n00
			HI351_write_cmos_sensor(0xb9, 0x05);//aInBgTgtOfs_a07_n00

			HI351_write_cmos_sensor(0x03, 0xc6);
			HI351_write_cmos_sensor(0x18, 0x43);//bInRgainMin_a00_n00
			HI351_write_cmos_sensor(0x19, 0x60);//bInRgainMax_a00_n00
			HI351_write_cmos_sensor(0x1a, 0x69);//bInBgainMin_a00_n00
			HI351_write_cmos_sensor(0x1b, 0x90);//bInBgainMax_a00_n00

			HI351_write_cmos_sensor(0xb9, 0x43);//bOutRgainMin_a00_n00
			HI351_write_cmos_sensor(0xba, 0x60);//bOutRgainMax_a00_n00
			HI351_write_cmos_sensor(0xbb, 0x69);//bOutBgainMin_a00_n00
			HI351_write_cmos_sensor(0xbc, 0x90);//bOutBgainMax_a00_n00

			HI351_write_cmos_sensor(0x03, 0xc5);
			HI351_write_cmos_sensor(0x10, 0xb1);//AWB On

			///////////////////////////////////////////
			// D1 Page Adaptive R/B saturation
			///////////////////////////////////////////
			HI351_write_cmos_sensor(0x03, 0xd1);//Page d1

			//Cb
			HI351_write_cmos_sensor(0x2b, 0x90); //SATB_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x2c, 0x90); //SATB_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x2d, 0x90); //SATB_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x2e, 0x98); //SATB_03
			HI351_write_cmos_sensor(0x2f, 0x98); //SATB_04
			HI351_write_cmos_sensor(0x30, 0x98); //SATB_05
			HI351_write_cmos_sensor(0x31, 0xa0); //SATB_06
			HI351_write_cmos_sensor(0x32, 0xa0); //SATB_07
			HI351_write_cmos_sensor(0x33, 0xa0); //SATB_08
			HI351_write_cmos_sensor(0x34, 0xa8); //SATB_09
			HI351_write_cmos_sensor(0x35, 0xa8); //SATB_10
			HI351_write_cmos_sensor(0x36, 0xa8); //SATB_11

			//Cr

			HI351_write_cmos_sensor(0x37, 0x90);//SATR_00 STEVE for Low Light
			HI351_write_cmos_sensor(0x38, 0x90);//SATR_01 STEVE for Low Light
			HI351_write_cmos_sensor(0x39, 0x90);//SATR_02 STEVE for Low Light
			HI351_write_cmos_sensor(0x3a, 0x98);//SATR_03
			HI351_write_cmos_sensor(0x3b, 0x98);//SATR_04
			HI351_write_cmos_sensor(0x3c, 0x98);//SATR_05
			HI351_write_cmos_sensor(0x3d, 0xa0);//SATR_06
			HI351_write_cmos_sensor(0x3e, 0xa0);//SATR_07
			HI351_write_cmos_sensor(0x3f, 0xa0);//SATR_08
			HI351_write_cmos_sensor(0x40, 0xa8);//SATR_09
			HI351_write_cmos_sensor(0x41, 0xa8);//SATR_10
			HI351_write_cmos_sensor(0x42, 0xa8);//SATR_11
			mdelay(5);
			break;
		default:
			return FALSE;
	}

	return TRUE;

} /* HI351_set_param_wb */

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_effect
 *
 * DESCRIPTION
 *	effect setting.
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
BOOL HI351_set_param_effect(UINT16 para)
{
	kal_uint32  ret = TRUE;
	HI351_TRACE("[HI351]:HI351_set_param_effect :%d \n", para);

	switch (para)
	{
		case MEFFECT_OFF:
			//Filter OFF
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x10); //constant OFF
			HI351_write_cmos_sensor(0x44, 0x80); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x80); //cr_constant
			break;
		case MEFFECT_MONO:
			// Mono
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x13); //constant ON
			HI351_write_cmos_sensor(0x44, 0x80); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x80); //cr_constant
			break;
		case MEFFECT_NEGATIVE:
			//Negative
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x18); //constant OFF
			HI351_write_cmos_sensor(0x44, 0x80); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x80); //cr_constant
			break;
		case MEFFECT_SEPIA:
			//Sepia
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x13); //constant ON
			HI351_write_cmos_sensor(0x44, 0x60); //cb_constant
			HI351_write_cmos_sensor(0x45, 0xA3); //cr_constant
			break;
		case MEFFECT_SEPIAGREEN:
			//Sepia_green
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x13); //constant ON
			HI351_write_cmos_sensor(0x44, 0x60); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x60); //cr_constant
			break;
		case MEFFECT_SEPIABLUE:
			//Sepia_blue
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x13); //constant ON
			HI351_write_cmos_sensor(0x44, 0xe0); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x60); //cr_constant
			break;
		case MEFFECT_AQUA:
			//aqua
			HI351_write_cmos_sensor(0x03, 0x10);
			HI351_write_cmos_sensor(0x12, 0x13); //constant ON
			HI351_write_cmos_sensor(0x44, 0xA3); //cb_constant
			HI351_write_cmos_sensor(0x45, 0x60); //cr_constant
			break;
		default:
			ret = FALSE;
	}

	return ret;
} /* HI351_set_param_effect */

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_banding
 *
 * DESCRIPTION
 *	banding setting.
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
BOOL HI351_set_param_banding(UINT16 para)
{
	//HI351_TRACE("[HI351]:HI351_set_param_banding :%d \n", para);
#if 0	// A©ªAO¨öA¡Æ¡Ì AUAI¡¾a
	switch (para)
	{
		case AE_FLICKER_MODE_50HZ:
			//HI351_write_cmos_sensor(0x03, 0x00); //Dummy	 // Steve delete
			mdelay(1);  // STEVE DELAY
			spin_lock(&hi351mipi_drv_lock);
			HI351_Banding_setting = AE_FLICKER_MODE_50HZ;
			spin_unlock(&hi351mipi_drv_lock);
			break;
		case AE_FLICKER_MODE_60HZ:
			// HI351_write_cmos_sensor(0x03, 0x00); //Dummy // Steve delete
			mdelay(1);  // STEVE DELAY
			spin_lock(&hi351mipi_drv_lock);
			HI351_Banding_setting = AE_FLICKER_MODE_60HZ;
			spin_unlock(&hi351mipi_drv_lock);
			break;
		default:
			// HI351_write_cmos_sensor(0x03, 0x00); //Dummy // Steve delete
			mdelay(1);  // STEVE DELAY
			break;
	}
#endif
	HI351_set_Sleep_off();

	return KAL_TRUE;
} /* HI351_set_param_banding */

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_exposure
 *
 * DESCRIPTION
 *	exposure setting.
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
BOOL HI351_set_param_exposure(UINT16 para)
{
	HI351_TRACE("[HI351]:HI351_set_param_exposure :%d \n", para);

	HI351_write_cmos_sensor(0x03, 0x10);
	HI351_write_cmos_sensor(0x13, 0x0A);
	switch (para)
	{
		case AE_EV_COMP_30:
			HI351_write_cmos_sensor(0x4A, 0xc8);
			break;
		case AE_EV_COMP_25:
			HI351_write_cmos_sensor(0x4A, 0xb8);
			break;
		case AE_EV_COMP_20:
			HI351_write_cmos_sensor(0x4A, 0xa8);
			break;
		case AE_EV_COMP_15:
			HI351_write_cmos_sensor(0x4A, 0x98);
			break;
		case AE_EV_COMP_10:
			HI351_write_cmos_sensor(0x4A, 0x90);
			break;
		case AE_EV_COMP_05:
			HI351_write_cmos_sensor(0x4A, 0x88);
			break;
		case AE_EV_COMP_00:
			HI351_write_cmos_sensor(0x4A, 0x80); // EV 0 (Normal)
			break;
		case AE_EV_COMP_n05:
			HI351_write_cmos_sensor(0x4A, 0x78);
			break;
		case AE_EV_COMP_n10:
			HI351_write_cmos_sensor(0x4A, 0x70);
			break;
		case AE_EV_COMP_n15:
			HI351_write_cmos_sensor(0x4A, 0x68);
			break;
		case AE_EV_COMP_n20:
			HI351_write_cmos_sensor(0x4A, 0x58);
			break;
		case AE_EV_COMP_n25:
			HI351_write_cmos_sensor(0x4A, 0x48);
			break;
		case AE_EV_COMP_n30:
			HI351_write_cmos_sensor(0x4A, 0x38);
			break;
		default:
			return FALSE;
	}

	return TRUE;

} /* HI351_set_param_exposure */

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_iso
 *
 * DESCRIPTION
 *	ISO setting.
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
BOOL HI351_set_param_iso(UINT16 para)
{
	HI351_TRACE("[HI351]:HI351_set_param_iso :%d \n", para);
	Exif_info.iso = para;

	switch (para)
	{
		case AE_ISO_AUTO:
			HI351_write_cmos_sensor(0x03, 0xD5);
			HI351_write_cmos_sensor(0x10, 0x06);
			break;
		case AE_ISO_100:
			HI351_write_cmos_sensor(0x03, 0xD5);
			HI351_write_cmos_sensor(0x10, 0x16);
			break;
		case AE_ISO_200:
			HI351_write_cmos_sensor(0x03, 0xD5);
			HI351_write_cmos_sensor(0x10, 0x26);
			break;
		case AE_ISO_400:
			HI351_write_cmos_sensor(0x03, 0xD5);
			HI351_write_cmos_sensor(0x10, 0x36);	// ISO 800 => ISO 400 (0x46 => 0x36)
			break;
		default:
			HI351_write_cmos_sensor(0x03, 0xD5);
			HI351_write_cmos_sensor(0x10, 0x06);    /* Default set as the ISO Auto */
			break;
	}

	return KAL_TRUE;
} /* HI351_set_param_iso */


void hi351_scene_normal_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_normal_Setting[hi351_antibanding]) / sizeof(HI351_scene_normal_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_normal_Setting[hi351_antibanding],len);
}

void hi351_scene_portrait_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_portrait_Setting[hi351_antibanding]) / sizeof(HI351_scene_portrait_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_portrait_Setting[hi351_antibanding],len);
}

void hi351_scene_landscape_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_landscape_Setting[hi351_antibanding]) / sizeof(HI351_scene_landscape_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_landscape_Setting[hi351_antibanding],len);
}

void hi351_scene_sport_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_sport_Setting[hi351_antibanding]) / sizeof(HI351_scene_sport_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_sport_Setting[hi351_antibanding],len);
}

void hi351_scene_sunset_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_sunset_Setting[hi351_antibanding]) / sizeof(HI351_scene_sunset_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_sunset_Setting[hi351_antibanding],len);
}

void hi351_scene_night_mode(void)
{
	kal_uint32 len=0;
	len = sizeof(HI351_scene_night_Setting[hi351_antibanding]) / sizeof(HI351_scene_night_Setting[hi351_antibanding][0]);
	HI351_table_write_cmos_sensor(HI351_scene_night_Setting[hi351_antibanding],len);
}

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_scene
 *
 * DESCRIPTION
 *	This function scene mode of HI351.
 *
 * PARAMETERS
 *	mode
 *
 * RETURNS
 *	BOOL
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
BOOL HI351_set_param_scene(UINT16 para)
{
	if ((HI351_VEDIO_encode_mode == KAL_TRUE) \
			|| (HI351_ZSD_Preview_state == KAL_TRUE)) {
		return FALSE;	//If video mode, return directely.
	}


	HI351_TRACE("[HI351]:HI351_set_param_scene: =%d\n", para);

	switch (para)
	{
		case SCENE_MODE_PORTRAIT:
			//		HI351_TRACE("[HI351]:SCENE_MODE_PORTRAIT: =%d\n", para);
			hi351_scene_portrait_mode();
			break;
		case SCENE_MODE_LANDSCAPE:
			//		HI351_TRACE("[HI351]:SCENE_MODE_LANDSCAPE: =%d\n", para);
			hi351_scene_landscape_mode();
			break;
		case SCENE_MODE_SPORTS:
			//		HI351_TRACE("[HI351]:SCENE_MODE_SPORTS: =%d\n", para);
			hi351_scene_sport_mode();
			break;
		case SCENE_MODE_SUNSET:
			//		HI351_TRACE("[HI351]:SCENE_MODE_SUNSET: =%d\n", para);
			hi351_scene_sunset_mode();
			break;
		case SCENE_MODE_NIGHTSCENE:
			//		HI351_TRACE("[HI351]:SCENE_MODE_NIGHTSCENE: =%d\n", para);
			hi351_scene_night_mode();
			break;

		//                                                                
		case SCENE_MODE_OFF:
		default:
			//		HI351_TRACE("[HI351]:SCENE_MODE_OFF: =%d\n", para);
			hi351_scene_normal_mode();
			break;
		//                                                                
	}

	return TRUE;
}	/* HI351_set_param_scene */

/*************************************************************************
 * FUNCTION
 *	HI351_set_param_wb_effect_scene
 *
 * DESCRIPTION
 *
 *
 * PARAMETERS
 *
 *
 * RETURNS
 *
 *
 * GLOBALS AFFECTED
 *
 *************************************************************************/
void HI351_set_param_value(void)
{
	if (HI351_para_wb == PARAM_INIT_VALUE || HI351_para_scene == PARAM_INIT_VALUE || HI351_para_effect == PARAM_INIT_VALUE)
		return;

	if (HI351_para_wb != AWB_MODE_OFF && HI351_para_wb != AWB_MODE_AUTO)
	{
		HI351_set_param_scene(SCENE_MODE_OFF);
		HI351_set_param_effect(MEFFECT_OFF);
		HI351_set_param_wb(HI351_para_wb);
	}
	else if (HI351_para_scene != SCENE_MODE_OFF)
	{
		HI351_set_param_wb(AWB_MODE_AUTO);
		HI351_set_param_effect(MEFFECT_OFF);
		HI351_set_param_scene(HI351_para_scene);
	}
	else
	{
		HI351_set_param_wb(AWB_MODE_AUTO);
		HI351_set_param_scene(SCENE_MODE_OFF);
		HI351_set_param_effect(HI351_para_effect);
	}
}

void HI351_set_param_wb_effect_scene(FEATURE_ID iCmd, UINT16 para)
{
	HI351_TRACE("[HI351]:HI351_set_param_wb_effect_scene: =%d, %d, %d\n", HI351_para_scene, HI351_para_wb, HI351_para_effect);

	switch (iCmd)
	{
		case FID_SCENE_MODE:
			if (HI351_para_scene == PARAM_INIT_VALUE)
			{
				HI351_para_scene = para;
			}
			else
			{
				HI351_para_scene = para;
				if (HI351_para_effect != MEFFECT_OFF)
					HI351_para_effect = PARAM_INIT_VALUE;
			}
			break;
		case FID_AWB_MODE:
			if (HI351_para_wb == PARAM_INIT_VALUE)
			{
				HI351_para_wb = para;
			}
			else
			{
				HI351_para_wb = para;
				if (HI351_para_scene != SCENE_MODE_OFF)
					HI351_para_scene = PARAM_INIT_VALUE;
				if (HI351_para_effect != MEFFECT_OFF)
					HI351_para_effect = PARAM_INIT_VALUE;

			}
			break;
		case FID_COLOR_EFFECT:
			if (HI351_para_effect == PARAM_INIT_VALUE)
			{
				HI351_para_effect = para;
			}
			else
			{
				HI351_para_effect = para;
				HI351_para_scene = SCENE_MODE_OFF;
				HI351_para_wb = AWB_MODE_AUTO;
			}
			break;
		default :
			break;
	}

	HI351_set_param_value();
}

/*************************************************************************
 * FUNCTION
 *	HI351_set_ae_enable
 *
 * DESCRIPTION
 *	Enable/disable the auto exposure.
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
void HI351_set_ae_enable(kal_bool AE_enable)
{
	HI351_TRACE("[HI351]:HI351_set_ae_enable: AE_enable=%d\n", AE_enable);
#if 0
	if (AE_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0x00); //Dummy
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0x00); //Dummy
	}
#else
	if (AE_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0xc4); //AE en

		if(HI351_Banding_setting == AE_FLICKER_MODE_50HZ)
			HI351_write_cmos_sensor(0x10, 0xe8); // 50Hz, 0xe9, 60Hz:e1
		else
			HI351_write_cmos_sensor(0x10, 0xe0); // 50Hz, 0xe9, 60Hz:e1
		/*atuo flicker :*/
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0xc4); //AE off
		HI351_write_cmos_sensor(0x10, 0x60);
	}
#endif
}

/*************************************************************************
 * FUNCTION
 *	HI351_set_awb_enable
 *
 * DESCRIPTION
 *	Enable/disable the auto white balance.
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
void HI351_set_awb_enable(kal_bool AWB_enable)
{
	HI351_TRACE("[HI351]:HI351_set_awb_enable: AWB_enable=%d\n", AWB_enable);
#if 0
	if (AWB_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0x00); //Dummy
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0x00); //Dummy
	}
#else
	if (AWB_enable == KAL_TRUE)
	{
		HI351_write_cmos_sensor(0x03, 0xc5); //AWB en
		HI351_write_cmos_sensor(0x10, 0xb1);
	}
	else
	{
		HI351_write_cmos_sensor(0x03, 0xc5); //AWB off
		HI351_write_cmos_sensor(0x10, 0x30);
	}
#endif
}

/*************************************************************************
 * FUNCTION
 *    HI351GetEvAwbRef
 *
 * DESCRIPTION
 *    This function get sensor Ev/Awb (EV05/EV13) for auto scene detect
 *
 * PARAMETERS
 *    Ref
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void HI351GetEvAwbRef(UINT32 pSensorAEAWBRefStruct/*PSENSOR_AE_AWB_REF_STRUCT Ref*/)
{
	PSENSOR_AE_AWB_REF_STRUCT Ref = (PSENSOR_AE_AWB_REF_STRUCT)pSensorAEAWBRefStruct;
	HI351_TRACE("[HI351]:GetEvAwbRef ref = 0x%x \n", Ref);

	Ref->SensorAERef.AeRefLV05Shutter = 9166;
	Ref->SensorAERef.AeRefLV05Gain = 600; /* 4.68x, 128 base */
	Ref->SensorAERef.AeRefLV13Shutter = 184;
	Ref->SensorAERef.AeRefLV13Gain = 160; /* 1.25x, 128 base */
	Ref->SensorAwbGainRef.AwbRefD65Rgain = 218; /* 1.70x, 128 base */
	Ref->SensorAwbGainRef.AwbRefD65Bgain = 188; /* 146x, 128 base */
	Ref->SensorAwbGainRef.AwbRefCWFRgain = 176; /* 1.38x, 128 base */
	Ref->SensorAwbGainRef.AwbRefCWFBgain = 258; /* 2.01x, 128 base */
}

/*************************************************************************
 * FUNCTION
 *    HI351GetCurAeAwbInfo
 *
 * DESCRIPTION
 *    This function get sensor cur Ae/Awb for auto scene detect
 *
 * PARAMETERS
 *    Info
 *
 * RETURNS
 *    None
 *
 * LOCAL AFFECTED
 *
 *************************************************************************/
static void HI351GetCurAeAwbInfo(UINT32 pSensorAEAWBCurStruct/*PSENSOR_AE_AWB_CUR_STRUCT Info*/)
{
	PSENSOR_AE_AWB_CUR_STRUCT Info = (PSENSOR_AE_AWB_CUR_STRUCT)pSensorAEAWBCurStruct;
	HI351_TRACE("[HI351]:GetCurAeAwbInfo Info = 0x%x \n", Info);

	Info->SensorAECur.AeCurShutter = HI351_read_shutter();
	Info->SensorAECur.AeCurGain = HI351_read_gain(); /* 128 base */

	/* AWB R/B gain = 1 / 64 * B[7:0]. 0x20(1X) ~ 0x40(2X) */
	HI351_write_cmos_sensor(0x03, 0x16); /* read R gain, set R gain index */
	Info->SensorAwbGainCur.AwbCurRgain = HI351_read_cmos_sensor(0xA2) << 1; /* 128 base */
	HI351_write_cmos_sensor(0x03, 0x16); /* read B gain, set B gain index */
	Info->SensorAwbGainCur.AwbCurBgain = HI351_read_cmos_sensor(0xA3) << 1; /* 128 base */

	HI351_TRACE("[HI351]Curr shutter=%d(10us), gain=%d(128base), RGain=%d(128base), BGain=%d(128base)\n", \
			Info->SensorAECur.AeCurShutter, Info->SensorAECur.AeCurGain, \
			Info->SensorAwbGainCur.AwbCurRgain, Info->SensorAwbGainCur.AwbCurBgain);
}

/*************************************************************************
 * FUNCTION
 *	HI351Preview
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
UINT32 HI351Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
		MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	UINT16 iStartX = 3, iStartY = 3;
	UINT16 	shutter = 0,pv_gain = 0;
	HI351_Flash_OnOff = 0; //                                                                           

	HI351_TRACE("[HI351 Preview] target size=%d x %d, mirror=%d\n", \
			image_window->ImageTargetWidth, image_window->ImageTargetHeight, sensor_config_data->SensorImageMirror);

	spin_lock(&hi351mipi_drv_lock);
	HI351_sensor_cap_state = KAL_FALSE;
	HI351_ZSD_Preview_state = KAL_FALSE;
	HI351_gPVmode = KAL_TRUE;

	memset(&Exif_info,0,sizeof(HI351_EXIF_INFO));
	spin_unlock(&hi351mipi_drv_lock);
	/*Step1. set output size*/
	HI351PreviewSetting();

	//Step2. Nedd to turn on the AE/AWB in preview mode.
	//HI351_set_ae_enable(HI351_AE_ENABLE);
	//HI351_set_awb_enable(HI351_AWB_ENABLE);

	spin_lock(&hi351mipi_drv_lock);
	HI351_VEDIO_encode_mode = KAL_FALSE;

	//4  <2> if preview of capture PICTURE
	HI351_PV_dummy_pixels = 0;
	HI351_PV_dummy_lines = 0;

	//Step 3. record preview ISP_clk
	HI351_preview_pclk = 720;
	spin_unlock(&hi351mipi_drv_lock);

	//4 <3> set mirror and flip
	//HI351_set_mirror_flip(sensor_config_data->SensorImageMirror);

	//4 <6> set dummy
	//HI351_set_dummy(HI351_PV_dummy_pixels, HI351_PV_dummy_lines);

	HI351_TRACE("[HI351 Preview] dummy pixels=%d, dummy lines=%d\n", HI351_PV_dummy_pixels, HI351_PV_dummy_lines);

	image_window->GrabStartX = IMAGE_SENSOR_PV_GRAB_START_X;
	image_window->GrabStartY = IMAGE_SENSOR_PV_GRAB_START_Y;
	image_window->ExposureWindowWidth = HI351_IMAGE_SENSOR_PV_WIDTH;
	image_window->ExposureWindowHeight = HI351_IMAGE_SENSOR_PV_HEIGHT;

	HI351_para_scene = 0xFFFF;
	HI351_para_wb = 0xFFFF;
	HI351_para_effect = 0xFFFF;

	return TRUE;
}	/* HI351_Preview */


UINT32 HI351FullPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
#if 0
	volatile kal_uint32 cap_shutter = 0;

	HI351_TRACE("[HI351 FullPreview] target size=%d x %d, ZoomFactor=%d\n", \
			image_window->ImageTargetWidth, image_window->ImageTargetHeight, image_window->ZoomFactor);

	HI351_TRACE("HI351_zoom_factor:%d;\n",HI351_zoom_factor);

	spin_lock(&hi351mipi_drv_lock);
	HI351_ZSD_Preview_state = KAL_TRUE;
	spin_unlock(&hi351mipi_drv_lock);

	/* Step 1. set output size:2048x1536 */
	HI351FullPreviewSetting();

	/* Step 2. Enable the ae/awb mode */
	//HI351_set_ae_enable(HI351_AE_ENABLE);
	//HI351_set_awb_enable(HI351_AWB_ENABLE);

	spin_lock(&hi351mipi_drv_lock);
	HI351_gPVmode = KAL_FALSE;

	//Step 3. record preview ISP_clk
	HI351_preview_pclk = 720;
	spin_unlock(&hi351mipi_drv_lock);

	//4 <3> set mirror and flip
	//HI351_set_mirror_flip(sensor_config_data->SensorImageMirror);

	/* Step 3. Set dummy pixels & dummy lines. */
	//HI351_set_dummy(HI351_FULL_dummy_pixels, HI351_FULL_dummy_lines);

	/* Step 4. calculate capture shutter */
	//HI351_write_shutter(cap_shutter);

	HI351_TRACE("[HI351 FullPreview] dummy_pixels=%d, dummy_pixels=%d, cap_shutter=%d\n", \
			HI351_FULL_dummy_pixels, HI351_FULL_dummy_lines, cap_shutter);

	/* Step 5. Config the exposure window. */
	image_window->GrabStartX = IMAGE_SENSOR_FULL_GRAB_START_X;
	image_window->GrabStartY = IMAGE_SENSOR_FULL_GRAB_START_Y;
	image_window->ImageTargetWidth = HI351_IMAGE_SENSOR_FULL_WIDTH;
	image_window->ImageTargetHeight = HI351_IMAGE_SENSOR_FULL_HEIGHT;
#endif
	return TRUE;
}	/* HI351_Capture */

UINT32 HI351Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window, MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	volatile kal_uint32 cap_shutter = 0, prev_shutter;  //HI351_exposure_lines;

	HI351_TRACE("[HI351 Capture] target size=%d x %d, ZoomFactor=%d\n", \
			image_window->ImageTargetWidth, image_window->ImageTargetHeight, image_window->ZoomFactor);

	HI351_TRACE("HI351_zoom_factor:%d;\n",HI351_zoom_factor);

	spin_lock(&hi351mipi_drv_lock);
	HI351_sensor_cap_state = KAL_TRUE;
	spin_unlock(&hi351mipi_drv_lock);

	/* Step 1. Disable the ae/awb mode */
	//    HI351_set_ae_enable(KAL_FALSE);
	//    HI351_set_awb_enable(KAL_FALSE);

	prev_shutter = HI351_read_shutter();
	spin_lock(&hi351mipi_drv_lock);
	HI351_PV_Shutter = prev_shutter;
	spin_unlock(&hi351mipi_drv_lock);
	HI351_TRACE("[HI351 Capture] prev_shutter=%d\n", prev_shutter);

	/* Step 2. set output size:2048x1536 */
	HI351CaptureSetting();
	spin_lock(&hi351mipi_drv_lock);
	HI351_gPVmode = KAL_FALSE;
	spin_unlock(&hi351mipi_drv_lock);

	//4 <3> set mirror and flip
	//HI351_set_mirror_flip(sensor_config_data->SensorImageMirror);

	/* Step 3. Set dummy pixels & dummy lines. */
	//HI351_set_dummy(HI351_FULL_dummy_pixels, HI351_FULL_dummy_lines);

	/* Step 4. calculate capture shutter */
	//HI351_write_shutter(cap_shutter);

	HI351_TRACE("[HI351 Capture] dummy_pixels=%d, dummy_pixels=%d, cap_shutter=%d\n", \
			HI351_FULL_dummy_pixels, HI351_FULL_dummy_lines, cap_shutter);

	/* Step 5. Config the exposure window. */
	image_window->GrabStartX = IMAGE_SENSOR_FULL_GRAB_START_X;
	image_window->GrabStartY = IMAGE_SENSOR_FULL_GRAB_START_Y;
	image_window->ImageTargetWidth = HI351_IMAGE_SENSOR_FULL_WIDTH;
	image_window->ImageTargetHeight = HI351_IMAGE_SENSOR_FULL_HEIGHT;

	HI351_para_scene = 0xFFFF;
	HI351_para_wb = 0xFFFF;
	HI351_para_effect = 0xFFFF;

	return TRUE;
}	/* HI351_Capture */

void HI351_get_AEAWB_lock(UINT32 *pAElockRet32, UINT32 *pAWBlockRet32)
{
	HI351_TRACE("[HI351]enter HI351_get_AEAWB_lock function:\n ");
	*pAElockRet32 =1;
	*pAWBlockRet32=1;
	HI351_TRACE("[HI351]HI351_get_AEAWB_lock,AE=%d,AWB=%d\n",*pAElockRet32,*pAWBlockRet32);
	HI351_TRACE("[HI351]exit HI351_get_AEAWB_lock function:\n ");
}

void HI351_GetDelayInfo(UINT32 delayAddr)
{
	HI351_TRACE("[HI351]enter HI351_GetDelayInfo function:\n ");
	SENSOR_DELAY_INFO_STRUCT *pDelayInfo=(SENSOR_DELAY_INFO_STRUCT*)delayAddr;
	pDelayInfo->InitDelay=1;
	pDelayInfo->EffectDelay=2;
	pDelayInfo->AwbDelay=2;
	//pDelayInfo->AFSwitchDelayFrame=50;
	HI351_TRACE("[HI351]exit HI351_GetDelayInfo function:\n ");
}
void HI351_3ACtrl(ACDK_SENSOR_3A_LOCK_ENUM action)
{
	HI351_TRACE("[HI351]enter HI351MIPI_3ACtrl function:action=%d\n",action);
	switch (action)
	{
		case SENSOR_3A_AE_LOCK:
			HI351_set_ae_enable(KAL_FALSE);
			break;
		case SENSOR_3A_AE_UNLOCK:
			HI351_set_ae_enable(KAL_TRUE);
			break;

		case SENSOR_3A_AWB_LOCK:
			HI351_set_awb_enable(KAL_FALSE);
			break;

		case SENSOR_3A_AWB_UNLOCK:
			HI351_set_awb_enable(KAL_TRUE);
			break;
		default:
			break;
	}
	HI351_TRACE("[HI351]exit HI351MIPI_3ACtrl function:action=%d\n",action);
	return;
}
//#endif

static void HI351_Get_AF_Max_Num_Focus_Areas(UINT32 *pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
	HI351_TRACE("[HI351]HI351_Get_AF_Max_Num_Focus_Areas,  *pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

static void HI351_Get_AE_Max_Num_Metering_Areas(UINT32 *pFeatureReturnPara32)
{
	*pFeatureReturnPara32 = 0;
	HI351_TRACE("[HI351] HI351_Get_AE_Max_Num_Metering_Areas,*pFeatureReturnPara32 = %d\n",  *pFeatureReturnPara32);
}

// need review
UINT32 HI351SetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate)
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
	HI351_TRACE("[HI351]HI351SetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
#if 0
			pclk = 72000000;
			lineLength = HI351_IMAGE_SENSOR_PV_WIDTH;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI351_IMAGE_SENSOR_PV_HEIGHT;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&hi351_drv_lock);
			HI351Sensor.SensorMode= SENSOR_MODE_PREVIEW;
			HI351Sensor.PreviewDummyLines = dummyLine;
			spin_unlock(&hi351_drv_lock);
			HI351_set_dummy(HI351Sensor.PreviewDummyPixels, dummyLine);
#endif
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
#if 0
			pclk = 72000000;
			lineLength = HI351_IMAGE_SENSOR_FULL_WIDTH;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - HI351_IMAGE_SENSOR_FULL_HEIGHT;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&hi351_drv_lock);
			HI351Sensor.CaptureDummyLines = dummyLine;
			HI351Sensor.SensorMode= SENSOR_MODE_CAPTURE;
			spin_unlock(&hi351_drv_lock);
			HI351_set_dummy(HI351Sensor.CaptureDummyPixels, dummyLine);
#endif
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
	HI351_TRACE("[HI351]exit OV5645MIPIMaxFramerateByScenario function:\n ");
	return ERROR_NONE;
}
UINT32 HI351GetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate)
{
	HI351_TRACE("[HI351]enter HI351GetDefaultFramerateByScenario function:\n ");
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			*pframeRate = 300;
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			*pframeRate = 88;
			break;
		case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
		case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
		case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added
			*pframeRate = 300;
			break;
		default:
			break;
	}
	HI351_TRACE("[HI351]exit HI351GetDefaultFramerateByScenario function:\n ");
	return ERROR_NONE;
}

UINT32 HI351SetTestPatternMode(kal_bool bEnable)
{
	HI351_TRACE("[OV5645MIPI_OV5645SetTestPatternMode]test pattern bEnable:=%d\n",bEnable);

	//return ERROR_NONE;

	if(bEnable)
	{
		HI351_write_cmos_sensor(0x03,0x00);
		HI351_write_cmos_sensor(0x60,0x05);
	}
	else
	{
		HI351_write_cmos_sensor(0x03,0x00);
		HI351_write_cmos_sensor(0x60,0x00);
	}
	return ERROR_NONE;
}

UINT32 HI351GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
	pSensorResolution->SensorPreviewWidth = HI351_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight = HI351_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth = HI351_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorVideoHeight = HI351_IMAGE_SENSOR_PV_HEIGHT;

	pSensorResolution->SensorFullWidth = HI351_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight = HI351_IMAGE_SENSOR_FULL_HEIGHT;

	return ERROR_NONE;
}	/* HI351GetResolution() */

UINT32 HI351GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
		MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
		MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX = HI351_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorPreviewResolutionY = HI351_IMAGE_SENSOR_FULL_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=15;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX = HI351_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY = HI351_IMAGE_SENSOR_PV_HEIGHT;
			pSensorInfo->SensorCameraPreviewFrameRate=30;
			break;
	}
	pSensorInfo->SensorFullResolutionX = HI351_IMAGE_SENSOR_FULL_WIDTH;
	pSensorInfo->SensorFullResolutionY = HI351_IMAGE_SENSOR_FULL_HEIGHT;

	pSensorInfo->SensorVideoFrameRate=30;
	pSensorInfo->SensorStillCaptureFrameRate=10;
	pSensorInfo->SensorWebCamCaptureFrameRate=15;

	pSensorInfo->SensorResetActiveHigh=FALSE;
	pSensorInfo->SensorResetDelayCount=1;
	pSensorInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_YVYU;
	pSensorInfo->SensorClockPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
	pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

	pSensorInfo->SensorInterruptDelayLines = 1;
	pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;



	pSensorInfo->CaptureDelayFrame = HI351_Flash_OnOff ? 1 : 0; //                                                                           
	pSensorInfo->PreviewDelayFrame = 3;
	pSensorInfo->VideoDelayFrame = 5;
	pSensorInfo->SensorMasterClockSwitch = 0;
	pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_6MA;//ISP_DRIVING_2MA;

	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_1_LANE;
	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 16; // samjinjang mipi signal timing change for preview black patch 20130425
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorWidthSampling = 0;  // 0 is default 1x
	pSensorInfo->SensorHightSampling = 0;   // 0 is default 1x
	pSensorInfo->SensorPacketECCOrder = 1;

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0;
			pSensorInfo->SensorGrabStartY = 0; //                                                                                              
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=	3;
			pSensorInfo->SensorClockRisingCount= 0;
			pSensorInfo->SensorClockFallingCount= 2;
			pSensorInfo->SensorPixelClockCount= 3;
			pSensorInfo->SensorDataLatchCount= 2;
			pSensorInfo->SensorGrabStartX = 0;
			pSensorInfo->SensorGrabStartY = 0; //                                                                                              
			break;
		default:
			pSensorInfo->SensorClockFreq=24;
			pSensorInfo->SensorClockDividCount=3;
			pSensorInfo->SensorClockRisingCount=0;
			pSensorInfo->SensorClockFallingCount=2;
			pSensorInfo->SensorPixelClockCount=3;
			pSensorInfo->SensorDataLatchCount=2;
			pSensorInfo->SensorGrabStartX = 0;
			pSensorInfo->SensorGrabStartY = 0; //                                                                                              
			break;
	}

	spin_lock(&hi351mipi_drv_lock);
	HI351_PixelClockDivider=pSensorInfo->SensorPixelClockCount;
	spin_unlock(&hi351mipi_drv_lock);
	memcpy(pSensorConfigData, &HI351SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

	Capture_delay_in_Flash = Capture_delay;

	return ERROR_NONE;
}	/* HI351GetInfo() */


UINT32 HI351Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
		MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	HI351_TRACE("[HI351]:Control Senario ID:%x\n", ScenarioId);

	spin_lock(&hi351mipi_drv_lock);
	HI351CurrentScenarioId = ScenarioId;
	spin_unlock(&hi351mipi_drv_lock);

	switch (ScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			//case MSDK_SCENARIO_ID_VIDEO_CAPTURE_MPEG4:
			HI351Preview(pImageWindow, pSensorConfigData);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			//case MSDK_SCENARIO_ID_CAMERA_CAPTURE_MEM:
			HI351Capture(pImageWindow, pSensorConfigData);
			break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			//HI351FullPreview(pImageWindow, pSensorConfigData);
		default:
			break;
	}
	return TRUE;
}	/* HI351Control() */

UINT32 HI351YUVSensorSetting(FEATURE_ID iCmd, UINT32 iPara)
{
	HI351_TRACE("[HI351 YUV Sensor setting]: FID=%d, para=%d\n", iCmd, iPara);

	switch (iCmd)
	{
		case FID_SCENE_MODE:
#if 0

			if (iPara == SCENE_MODE_OFF)
			{
				hi351_scene_night_mode(0);
			}
			else if (iPara == SCENE_MODE_NIGHTSCENE)
			{
				hi351_scene_night_mode(1);
			}
#else
			//HI351_set_param_scene(iPara);
			HI351_set_param_wb_effect_scene(iCmd, iPara);
#endif
			break;
		case FID_AWB_MODE:
			//HI351_set_param_wb(iPara);
			HI351_set_param_wb_effect_scene(iCmd, iPara);
			break;
		case FID_COLOR_EFFECT:
			//HI351_set_param_effect(iPara);
			HI351_set_param_wb_effect_scene(iCmd, iPara);
			break;
		case FID_AE_FLICKER:
			HI351_set_param_banding(iPara);
			break;
		case FID_AE_EV:
			HI351_set_param_exposure(iPara);
			break;
		case FID_AE_ISO:
			HI351_set_param_iso(iPara);
			break;
		case FID_AE_SCENE_MODE:
			if (MSDK_SCENARIO_ID_CAMERA_ZSD != HI351CurrentScenarioId)
			{
				if (iPara == AE_MODE_OFF)
				{
					//HI351_AE_ENABLE = KAL_FALSE;
				}
				else
				{
					//HI351_AE_ENABLE = KAL_TRUE;
				}
				//HI351_set_ae_enable(HI351_AE_ENABLE);
			}
			break;
		case FID_ZOOM_FACTOR:
			spin_lock(&hi351mipi_drv_lock);
			HI351_zoom_factor = iPara;
			spin_unlock(&hi351mipi_drv_lock);
			break;
		default:
			break;
	}
	return TRUE;
}   /* HI351YUVSensorSetting */

UINT32 HI351YUVSetVideoMode(UINT16 u2FrameRate)
{

	UINT16 u2FrameRate_temp;
	u2FrameRate_temp = u2FrameRate / 1000;		// fps range
	u2FrameRate = u2FrameRate_temp;
	kal_uint32 len=0;

#if 0
	kal_uint8 iTemp = 0;

	/* 0x11 [page mode 0]: VDOCTL2 [default=0x90, r/w]
	   B[1] Vertical Flip Function (0:OFF, 1:ON)
	   B[0] Horizontal Flip Function (0:OFF, 1:ON)  */
	HI351_write_cmos_sensor(0x03, 0x00);
	iTemp = HI351_read_cmos_sensor(0x11);
	iTemp &= 0x03;
#endif

	spin_lock(&hi351mipi_drv_lock);
	HI351_VEDIO_encode_mode = KAL_TRUE;
	spin_unlock(&hi351mipi_drv_lock);

	HI351_TRACE("[HI351]video mode: frame_rate=%d\n", u2FrameRate);

	if (u2FrameRate == 30)
	{
		len = sizeof(HI351_Video_30fps_Setting[hi351_antibanding]) / sizeof(HI351_Video_30fps_Setting[hi351_antibanding][0]);
		HI351_table_write_cmos_sensor(HI351_Video_30fps_Setting[hi351_antibanding],len);
	}
	else if (u2FrameRate == 15)
	{
		len = sizeof(HI351_Video_15fps_Setting[hi351_antibanding]) / sizeof(HI351_Video_15fps_Setting[hi351_antibanding][0]);
		HI351_table_write_cmos_sensor(HI351_Video_15fps_Setting[hi351_antibanding],len);
	}
	else
	{
		HI351_TRACE("[HI351] Wrong frame rate setting %d\n", u2FrameRate);
		return KAL_FALSE;
	}

	return TRUE;
}


static void HI351GetCurAeInfo(UINT32 pSensorAECurStruct)
{
	PSENSOR_FLASHLIGHT_AE_INFO_STRUCT Info = (PSENSOR_FLASHLIGHT_AE_INFO_STRUCT)pSensorAECurStruct;
	Info->Exposuretime = HI351_read_shutter();
	Info->Gain=HI351_read_gain();
	if(Info->GAIN_BASE == 0xAABBCCDD)
	{
		Capture_delay_in_Flash = 1;
	}
	Info->GAIN_BASE = HI351_read_exp_unit();

	HI351_TRACE("[HI351][HI351GetCurAeInfo]Exp:%d,Gain:%d\n",Info->Exposuretime,Info->Gain);

}
static void HI351GetExifInfo(UINT32 pSensorExifInfoStruct)
{
	PSENSOR_EXIF_INFO_STRUCT Info = (PSENSOR_EXIF_INFO_STRUCT)pSensorExifInfoStruct;
	Info->FNumber = 28;         //                                                        
	Info->AWBMode = Exif_info.awb;
	Info->CapExposureTime = 0;
	Info->FlashLightTimeus = Exif_info.flashlight;
	Info->AEISOSpeed = Exif_info.iso;
	Exif_info.iso_auto = HI351_read_gain()/8;
	Info->RealISOValue = Exif_info.iso_auto;
	Info->FocalLength = 270;    //                                                            
	HI351_TRACE("[HI351][HI351GetExifInfo]AWBMode:%d,AEISOSpeed:%d, CapExposureTime=%d\n",Info->AWBMode,Info->AEISOSpeed,Info->CapExposureTime);
}

//                                                                   
static void HI351_FlashTriggerCheck(unsigned int *pFeatureReturnPara32)
{
	HI351_Flash_OnOff = 0; //                                                                           
	UINT32 Luminance=0;
	UINT8 Y1,Y2,Y3,Y4;
	*pFeatureReturnPara32 = 0;

	//                                                                             
	if(HI351_flash_mode == FLASH_MODE_FORCE_ON)
	{
		*pFeatureReturnPara32 = 1;
	}
	else if(HI351_flash_mode == FLASH_MODE_FORCE_OFF)
	{
		return;
	}
	else if(HI351_flash_mode == FLASH_MODE_AUTO)
	{
		HI351_write_cmos_sensor(0x03, 0xD3);
		Y1 = HI351_read_cmos_sensor(0x92);
		Y2 = HI351_read_cmos_sensor(0x93);
		Y3 = HI351_read_cmos_sensor(0x94);
		Y4 = HI351_read_cmos_sensor(0x95);

		Luminance = (Y1<<24) | (Y2<<16) | (Y3<<8) | Y4;
		HI351_TRACE("[HI351][HI351_FlashTriggerCheck] Luminance : 0x%x\n",Luminance);
		if(Luminance >= 0x02600000)
		{
			*pFeatureReturnPara32 = 1;
		}
	}

	HI351_Flash_OnOff = *pFeatureReturnPara32;
	//                                                                             
	return;
}
//                                                                   

UINT32 HI351FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
		UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
	UINT16 u2Temp = 0;
	UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
	UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
	UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
	UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
	UINT32 u4Temp = 0; // temp flicker overflow
	MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
	MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;

	switch (FeatureId)
	{
		case SENSOR_FEATURE_GET_RESOLUTION:
			*pFeatureReturnPara16++=HI351_IMAGE_SENSOR_FULL_WIDTH;
			*pFeatureReturnPara16=HI351_IMAGE_SENSOR_FULL_HEIGHT;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_PERIOD:
			switch (HI351CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara16++=HI351_FULL_PERIOD_PIXEL_NUMS+HI351_FULL_dummy_pixels;
					*pFeatureReturnPara16=HI351_FULL_PERIOD_LINE_NUMS+HI351_FULL_dummy_lines;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara16++=HI351_PV_PERIOD_PIXEL_NUMS+HI351_PV_dummy_pixels;
					*pFeatureReturnPara16=HI351_PV_PERIOD_LINE_NUMS+HI351_PV_dummy_lines;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch (HI351CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = HI351_sensor_pclk/10;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = HI351_sensor_pclk/10;
					*pFeatureParaLen=4;
					break;
			}
			break;
		case SENSOR_FEATURE_SET_ESHUTTER:
			//			u2Temp = HI351_read_shutter();
			HI351_TRACE("[HI351][HI351FeatureControl]Shutter:%d \n", *pFeatureData32);
			HI351_write_shutter(*pFeatureData32);

			break;
		case SENSOR_FEATURE_SET_NIGHTMODE:
			HI351_TRACE("[HI351][HI351FeatureControl]SENSOR_FEATURE_SET_NIGHTMODE:%d \n", *pFeatureData16);
			if (pFeatureData16 == KAL_FALSE)
			{
				hi351_scene_normal_mode();
			}
			else if (pFeatureData16 == KAL_TRUE)
			{
				hi351_scene_night_mode();
			}
			break;
		case SENSOR_FEATURE_SET_GAIN:
			//			u2Temp = HI351_read_gain();
			//			HI351_TRACE("Gain:%d\n", u2Temp);
			//			HI351_TRACE("y_val:%d\n", HI351_read_cmos_sensor(0x301B));
			HI351_TRACE("[HI351][HI351FeatureControl]Gain:%d \n", *pFeatureData16);
			HI351_write_gain((UINT32) *pFeatureData16);
			break;
		case SENSOR_FEATURE_SET_FLASHLIGHT:
			//                                                                             
			HI351_flash_mode = *pFeatureData16;
			HI351_TRACE("[HI351][HI351FeatureControl]SET_FLASHLIGHT:%d\n",*pFeatureData16);
			//                                                                             
			break;
		//                                                                   
		case SENSOR_FEATURE_GET_TRIGGER_FLASHLIGHT_INFO:
			HI351_FlashTriggerCheck(pFeatureData32);
			break;
		//                                                                   
		case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
			spin_lock(&hi351mipi_drv_lock);
			HI351_isp_master_clock=*pFeatureData32;
			spin_unlock(&hi351mipi_drv_lock);
			break;
		case SENSOR_FEATURE_SET_REGISTER:
			HI351_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
			break;
		case SENSOR_FEATURE_GET_REGISTER:
			pSensorRegData->RegData = HI351_read_cmos_sensor(pSensorRegData->RegAddr);
			break;
		case SENSOR_FEATURE_GET_CONFIG_PARA:
			memcpy(pSensorConfigData, &HI351SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
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
			// get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
			// if EEPROM does not exist in camera module.
			*pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_CHECK_SENSOR_ID:
			HI351GetSensorID(pFeatureReturnPara32);
			break;
		case SENSOR_FEATURE_SET_YUV_CMD:
			HI351YUVSensorSetting((FEATURE_ID)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_SET_VIDEO_MODE:
			HI351YUVSetVideoMode(*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_EV_AWB_REF:
			HI351GetEvAwbRef(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_SHUTTER_GAIN_AWB_GAIN:
			HI351GetCurAeAwbInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_AE_FLASHLIGHT_INFO:
			HI351GetCurAeInfo(*pFeatureData32);
			break;
		case SENSOR_FEATURE_GET_EXIF_INFO:
			HI351GetExifInfo(*pFeatureData32);
			break;
			////////////////////////////////////////////
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			HI351SetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=HI351_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;

		case SENSOR_FEATURE_SET_YUV_3A_CMD:
			HI351_3ACtrl((ACDK_SENSOR_3A_LOCK_ENUM)*pFeatureData32);
			break;

		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			HI351SetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			HI351GetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32,(MUINT32 *)*(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_AE_AWB_LOCK_INFO:
			HI351_get_AEAWB_lock(*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DELAY_INFO:
			HI351_TRACE("SENSOR_FEATURE_GET_DELAY_INFO\n");
			HI351_GetDelayInfo(*pFeatureData32);
			break;
			//case SENSOR_FEATURE_AUTOTEST_CMD:
			//	HI351_TRACE("SENSOR_FEATURE_AUTOTEST_CMD\n");
			//	HI351_AutoTestCmd(*pFeatureData32,*(pFeatureData32+1));
			//	break;
		case SENSOR_FEATURE_INITIALIZE_AF:
			break;

		case SENSOR_FEATURE_GET_AF_MAX_NUM_FOCUS_AREAS:
			HI351_Get_AF_Max_Num_Focus_Areas(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_GET_AE_MAX_NUM_METERING_AREAS:
			HI351_Get_AE_Max_Num_Metering_Areas(pFeatureReturnPara32);
			*pFeatureParaLen=4;
			break;

		//                                                                 
		case SENSOR_FEATURE_GET_SENSOR_VIEWANGLE:
			{
				UINT32 *pHorFOV = (UINT32*)pFeatureReturnPara32[0];
				UINT32 *pVerFOV = (UINT32*)pFeatureReturnPara32[1];

				HI351_TRACE("SENSOR_FEATURE_GET_SENSOR_VIEWANGLE\n");
				*pHorFOV = 54;  // HorFOV = 54.7
				*pVerFOV = 42;  // VerFOV = 42.3
				*pFeatureParaLen = 8;
			}
			break;
		//                                                                 

		default:
			break;
	}
	return ERROR_NONE;
}	/* HI351FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncHI351=
{
	HI351Open,
	HI351GetInfo,
	HI351GetResolution,
	HI351FeatureControl,
	HI351Control,
	HI351Close
};

UINT32 HI351_MIPI_YUV_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&SensorFuncHI351;

	return ERROR_NONE;
}	/* SensorInit() */

