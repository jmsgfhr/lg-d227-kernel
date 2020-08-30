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

#ifdef BUILD_LK
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif


#include "lcm_drv.h"

#if 1

#define FRAME_WIDTH  (240)
#define FRAME_HEIGHT (320)
#define LCM_ID       (0x69)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

static __inline void send_ctrl_cmd(unsigned int cmd)
{
	lcm_util.send_cmd(cmd);
}

static __inline void send_data_cmd(unsigned int data)
{
	lcm_util.send_data(data&0xff);
}

static __inline unsigned int read_data_cmd(void)
{
    return 0xFF&lcm_util.read_data();
}

static __inline void set_lcm_register(unsigned int regIndex,
		unsigned int regData)
{
	send_ctrl_cmd(regIndex);
	send_data_cmd(regData);
}

static void sw_clear_panel(unsigned int color)
{
	unsigned short x0, y0, x1, y1, x, y;
	unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;

	x0 = (unsigned short)0;
	y0 = (unsigned short)0;
	x1 = (unsigned short)FRAME_WIDTH-1;
	y1 = (unsigned short)FRAME_HEIGHT-1;

	h_X_start=((x0&0xFF00)>>8);
	l_X_start=(x0&0x00FF);
	h_X_end=((x1&0xFF00)>>8);
	l_X_end=(x1&0x00FF);

	h_Y_start=((y0&0xFF00)>>8);
	l_Y_start=(y0&0x00FF);
	h_Y_end=((y1&0xFF00)>>8);
	l_Y_end=(y1&0x00FF);

	send_ctrl_cmd(0x2A);
	send_data_cmd(h_X_start);
	send_data_cmd(l_X_start);
	send_data_cmd(h_X_end);
	send_data_cmd(l_X_end);

	send_ctrl_cmd(0x2B);
	send_data_cmd(h_Y_start);
	send_data_cmd(l_Y_start);
	send_data_cmd(h_Y_end);
	send_data_cmd(l_Y_end);

	send_ctrl_cmd(0x29);

	send_ctrl_cmd(0x2C);
    for (y = y0; y <= y1; ++ y) {
        for (x = x0; x <= x1; ++ x) {
            lcm_util.send_data(color);
        }
    }
}

static void init_lcm_registers(void)
{
     send_ctrl_cmd(0xCF);	// EXTC Option
     send_data_cmd(0x20);
     send_data_cmd(0x21);
     send_data_cmd(0x20);

     send_ctrl_cmd(0xF2); // 3-Gamma Function Off
     send_data_cmd(0x02);

     send_ctrl_cmd(0xB4); // Inversion Control -> 2Dot inversion
     send_data_cmd(0x02);

     send_ctrl_cmd(0xC0); // Powr control 1
     send_data_cmd(0x15);
     send_data_cmd(0x15);

     send_ctrl_cmd(0xC1); // Power control 2
     send_data_cmd(0x05);

     send_ctrl_cmd(0xC2);	// Powr control 3
     send_data_cmd(0x32);

     send_ctrl_cmd(0xC5);	// Vcom control 1
     send_data_cmd(0xFC);

     send_ctrl_cmd(0xCB);	// V-core Setting
     send_data_cmd(0x31);
     send_data_cmd(0x24);
     send_data_cmd(0x00);
     send_data_cmd(0x34);

     send_ctrl_cmd(0xF6);	// Interface control
     send_data_cmd(0x41);
     send_data_cmd(0x00);
     send_data_cmd(0x00);

     send_ctrl_cmd(0xB7);	// Entry Mode Set
     send_data_cmd(0x06);

     send_ctrl_cmd(0xB1);	// Frame Rate Control
     send_data_cmd(0x00);
     send_data_cmd(0x1B);

     send_ctrl_cmd(0x36);	// Memory Access Control
     send_data_cmd(0x08); // seosc 08 -> C8

     send_ctrl_cmd(0xB5);	// Blanking Porch control
     send_data_cmd(0x02);
     send_data_cmd(0x02);
     send_data_cmd(0x0A);
     send_data_cmd(0x14);

     send_ctrl_cmd(0xB6);	// Display Function control
     send_data_cmd(0x02);
     send_data_cmd(0x82);
     send_data_cmd(0x27);
     send_data_cmd(0x00);

     send_ctrl_cmd(0x3A);	// Pixel Format->DBI(5=16bit)
     send_data_cmd(0x05);

     send_ctrl_cmd(0x51);//write display brightness
     send_data_cmd(0xff);//set brightness 0x00-0xff
     MDELAY(50);

     send_ctrl_cmd(0x53);//write ctrl display
     send_data_cmd(0x24);
     MDELAY(50);

     send_ctrl_cmd(0x55);
     send_data_cmd(0x02);//still picture
     MDELAY(50);

     send_ctrl_cmd(0x5e);//write CABC minumum brightness
     send_data_cmd(0x70);//
     MDELAY(50);

     send_ctrl_cmd(0x35);	// Tearing Effect Line On
     send_data_cmd(0x00);

     send_ctrl_cmd(0x44);	// Tearing Effect Control Parameter
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0xE0);	// Positive Gamma Correction
     send_data_cmd(0x00);
     send_data_cmd(0x06);
     send_data_cmd(0x07);
     send_data_cmd(0x03);
     send_data_cmd(0x0A);
     send_data_cmd(0x0A);
     send_data_cmd(0x41);
     send_data_cmd(0x59);
     send_data_cmd(0x4D);
     send_data_cmd(0x0C);
     send_data_cmd(0x18);
     send_data_cmd(0x0F);
     send_data_cmd(0x22);
     send_data_cmd(0x1D);
     send_data_cmd(0x0F);

     send_ctrl_cmd(0xE1);	// Negative Gamma Correction
     send_data_cmd(0x06);
     send_data_cmd(0x23);
     send_data_cmd(0x24);
     send_data_cmd(0x01);
     send_data_cmd(0x0F);
     send_data_cmd(0x01);
     send_data_cmd(0x36);
     send_data_cmd(0x23);
     send_data_cmd(0x41);
     send_data_cmd(0x07);
     send_data_cmd(0x0F);
     send_data_cmd(0x0F);
     send_data_cmd(0x30);
     send_data_cmd(0x27);
     send_data_cmd(0x0E);

     send_ctrl_cmd(0x2A);	// Column address
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0x2B);	// Page address
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0x01);
     send_data_cmd(0x3F);

     send_ctrl_cmd(0xE8);
     send_data_cmd(0x84);
     send_data_cmd(0x1A);
     send_data_cmd(0x68);

     send_ctrl_cmd(0x11);
     MDELAY(120);

     send_ctrl_cmd(0X29);

}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DBI;
	params->ctrl   = LCM_CTRL_PARALLEL_DBI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->io_select_mode = 1;

	params->dbi.port                    = 0;
	params->dbi.clock_freq              = LCM_DBI_CLOCK_FREQ_52M;
	params->dbi.data_width              = LCM_DBI_DATA_WIDTH_16BITS;
	params->dbi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dbi.data_format.trans_seq   = LCM_DBI_TRANS_SEQ_MSB_FIRST;
	params->dbi.data_format.padding     = LCM_DBI_PADDING_ON_MSB;
	params->dbi.data_format.format      = LCM_DBI_FORMAT_RGB565;
	params->dbi.data_format.width       = LCM_DBI_DATA_WIDTH_16BITS;
	params->dbi.cpu_write_bits          = LCM_DBI_CPU_WRITE_16_BITS;
	params->dbi.io_driving_current      = 0;

	params->dbi.parallel.write_setup    = 2;
	params->dbi.parallel.write_hold     = 2;
	params->dbi.parallel.write_wait     = 4;
	params->dbi.parallel.read_setup     = 2;
	params->dbi.parallel.read_latency   = 31;
	params->dbi.parallel.wait_period    = 9;

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

}



static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	LCM_PRINT("[seosctest] lcm_init+++\n");
	init_lcm_registers();
	LCM_PRINT("[seosctest] lcm_init+++\n");
}


static void lcm_suspend(void)
{
    //sw_clear_panel(0);
	send_ctrl_cmd(0x10);
	MDELAY(5);
	LCM_PRINT("[seosctest] lcm_suspend+++\n");
}


static void lcm_resume(void)
{
	//send_ctrl_cmd(0x11);

	LCM_PRINT("[seosctest] lcm_resume+++\n");
	lcm_init();
	MDELAY(120);
}

static void lcm_update(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height)
{
	unsigned short x0, y0, x1, y1;
	unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;

	x0 = (unsigned short)x;
	y0 = (unsigned short)y;
	x1 = (unsigned short)x+width-1;
	y1 = (unsigned short)y+height-1;


    h_X_start=((x0&0xFF00)>>8);
    l_X_start=(x0&0x00FF);
    h_X_end=((x1&0xFF00)>>8);
    l_X_end=(x1&0x00FF);

    h_Y_start=((y0&0xFF00)>>8);
    l_Y_start=(y0&0x00FF);
    h_Y_end=((y1&0xFF00)>>8);
    l_Y_end=(y1&0x00FF);

	send_ctrl_cmd(0x2A);
	send_data_cmd(h_X_start);
	send_data_cmd(l_X_start);
	send_data_cmd(h_X_end);
	send_data_cmd(l_X_end);

	send_ctrl_cmd(0x2B);
	send_data_cmd(h_Y_start);
	send_data_cmd(l_Y_start);
	send_data_cmd(h_Y_end);
	send_data_cmd(l_Y_end);

	send_ctrl_cmd(0x29);

	send_ctrl_cmd(0x2C);

	LCM_PRINT("[seosctest] lcm_update+++\n");
}

static void lcm_setbacklight(unsigned int level)
{
	if(level > 255) level = 255;
#if 0
	send_ctrl_cmd(0x51);
	send_data_cmd(level);
#else
    send_ctrl_cmd(0xBE);
    send_data_cmd(0x0F);
#endif
}
static unsigned int lcm_compare_id(void)
{
#if 0
    send_ctrl_cmd(0xB9);  // SET password
	send_data_cmd(0xFF);
	send_data_cmd(0x83);
	send_data_cmd(0x69);
    send_ctrl_cmd(0xC3);
	send_data_cmd(0xFF);

	send_ctrl_cmd(0xF4);
	read_data_cmd();
    return (LCM_ID == read_data_cmd())?1:0;
#else
    return 1;
#endif
}

static void lcm_set_pwm(unsigned int divider)
{
#if 0
 send_ctrl_cmd(0xBE);
 send_data_cmd(0xFF);

 send_ctrl_cmd(0xBF);
 send_data_cmd(0x07);
#endif
}


#else
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (240)
#define FRAME_HEIGHT (320)
#define LCM_ID       (0x69)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

static __inline void send_ctrl_cmd(unsigned int cmd)
{
	lcm_util.send_cmd(cmd);
}

static __inline void send_data_cmd(unsigned int data)
{
	lcm_util.send_data(data&0xff);
}

static __inline unsigned int read_data_cmd(void)
{
    return 0xFF&lcm_util.read_data();
}

static __inline void set_lcm_register(unsigned int regIndex,
		unsigned int regData)
{
	send_ctrl_cmd(regIndex);
	send_data_cmd(regData);
}

static void sw_clear_panel(unsigned int color)
{
	unsigned short x0, y0, x1, y1, x, y;
	unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;

	x0 = (unsigned short)0;
	y0 = (unsigned short)0;
	x1 = (unsigned short)FRAME_WIDTH-1;
	y1 = (unsigned short)FRAME_HEIGHT-1;

	h_X_start=((x0&0xFF00)>>8);
	l_X_start=(x0&0x00FF);
	h_X_end=((x1&0xFF00)>>8);
	l_X_end=(x1&0x00FF);

	h_Y_start=((y0&0xFF00)>>8);
	l_Y_start=(y0&0x00FF);
	h_Y_end=((y1&0xFF00)>>8);
	l_Y_end=(y1&0x00FF);

	send_ctrl_cmd(0x2A);
	send_data_cmd(h_X_start);
	send_data_cmd(l_X_start);
	send_data_cmd(h_X_end);
	send_data_cmd(l_X_end);

	send_ctrl_cmd(0x2B);
	send_data_cmd(h_Y_start);
	send_data_cmd(l_Y_start);
	send_data_cmd(h_Y_end);
	send_data_cmd(l_Y_end);

	send_ctrl_cmd(0x29);

	send_ctrl_cmd(0x2C);
    for (y = y0; y <= y1; ++ y) {
        for (x = x0; x <= x1; ++ x) {
            lcm_util.send_data(color);
        }
    }
}

static void init_lcm_registers(void)
{
     send_ctrl_cmd(0xCF);	// EXTC Option
     send_data_cmd(0x00);
     send_data_cmd(0x21);

     send_ctrl_cmd(0xF2); // 3-Gamma Function Off
     send_data_cmd(0x02);

     send_ctrl_cmd(0xB4); // Inversion Control -> 2Dot inversion
     send_data_cmd(0x02);

     send_ctrl_cmd(0xC0); // Powr control 1
     send_data_cmd(0x14);
     send_data_cmd(0x0f);

     send_ctrl_cmd(0xC1); // Power control 2
     send_data_cmd(0x04);

     send_ctrl_cmd(0xC2);	// Powr control 3
     send_data_cmd(0x32);

     send_ctrl_cmd(0xC5);	// Vcom control 1
     send_data_cmd(0xFC);

#if 0 //                                            
     send_ctrl_cmd(0xCB);	// V-core Setting
     send_data_cmd(0x31);
     send_data_cmd(0x24);
     send_data_cmd(0x00);
     send_data_cmd(0x34);
#endif

     send_ctrl_cmd(0xF6);	// Interface control
     send_data_cmd(0x41);
     send_data_cmd(0x30);
     send_data_cmd(0x00);

     send_ctrl_cmd(0xB7);	// Entry Mode Set
     send_data_cmd(0x06);

     send_ctrl_cmd(0xB1);	// Frame Rate Control
     send_data_cmd(0x00);
     send_data_cmd(0x1f);

     send_ctrl_cmd(0x36);	// Memory Access Control
     send_data_cmd(0x08);

     send_ctrl_cmd(0xB5);	// Blanking Porch control
     send_data_cmd(0x02);
     send_data_cmd(0x02);
     send_data_cmd(0x0A);
     send_data_cmd(0x14);

     send_ctrl_cmd(0xB6);	// Display Function control
     send_data_cmd(0x0a);
     send_data_cmd(0x02);
     send_data_cmd(0x27);
     send_data_cmd(0x04);

     send_ctrl_cmd(0x3A);	// Pixel Format->DBI(5=16bit)
     send_data_cmd(0x55);

#if 0 //                                            
     send_ctrl_cmd(0x51);//write display brightness
     send_data_cmd(0xff);//set brightness 0x00-0xff
     MDELAY(50);

     send_ctrl_cmd(0x53);//write ctrl display
     send_data_cmd(0x24);
     MDELAY(50);

     send_ctrl_cmd(0x55);
     send_data_cmd(0x02);//still picture
     MDELAY(50);

     send_ctrl_cmd(0x5e);//write CABC minumum brightness
     send_data_cmd(0x70);//
     MDELAY(50);
#endif

     send_ctrl_cmd(0x35);	// Tearing Effect Line On
     send_data_cmd(0x00);

     send_ctrl_cmd(0x44);	// Tearing Effect Control Parameter
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0xE0);	// Positive Gamma Correction
	 send_ctrl_cmd(0x08);
	 send_ctrl_cmd(0x0e);
	 send_ctrl_cmd(0x12);
	 send_ctrl_cmd(0x04);
	 send_ctrl_cmd(0x0f);
	 send_ctrl_cmd(0x05);
	 send_ctrl_cmd(0x35);
	 send_ctrl_cmd(0x32);
	 send_ctrl_cmd(0x4f);
	 send_ctrl_cmd(0x03);
	 send_ctrl_cmd(0x0c);
	 send_ctrl_cmd(0x0a);
	 send_ctrl_cmd(0x2f);
	 send_ctrl_cmd(0x35);
	 send_ctrl_cmd(0x0f);


     send_ctrl_cmd(0xE1);	// Negative Gamma Correction
     send_data_cmd(0x08);
	 send_data_cmd(0x0e);
	 send_data_cmd(0x12);
	 send_data_cmd(0x03);
	 send_data_cmd(0x0e);
	 send_data_cmd(0x03);
	 send_data_cmd(0x35);
	 send_data_cmd(0x00);
	 send_data_cmd(0x4d);
	 send_data_cmd(0x0a);
	 send_data_cmd(0x12);
	 send_data_cmd(0x0f);
	 send_data_cmd(0x31);
	 send_data_cmd(0x38);
	 send_data_cmd(0x0f);


     send_ctrl_cmd(0x2A);	// Column address
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0x2B);	// Page address
     send_data_cmd(0x00);
     send_data_cmd(0x00);
     send_data_cmd(0x01);
     send_data_cmd(0x3f);

     send_ctrl_cmd(0xE8);
     send_data_cmd(0x84);
     send_data_cmd(0x1A);
     send_data_cmd(0x38);

     send_ctrl_cmd(0x11);
     MDELAY(120);

	 send_ctrl_cmd(0x2c); // Write memory start
	 {
			   int x, y;
	 for(y = 0; y < FRAME_HEIGHT; y++) {
			   for(x = 0; x < FRAME_WIDTH; x++) {
						  send_ctrl_cmd(0x77);
			   						}
	 					}
	 }
 MDELAY(80);

     send_ctrl_cmd(0X29);

}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DBI;
	params->ctrl   = LCM_CTRL_PARALLEL_DBI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->io_select_mode = 1;

	params->dbi.port                    = 0;
	params->dbi.clock_freq              = LCM_DBI_CLOCK_FREQ_52M;
	params->dbi.data_width              = LCM_DBI_DATA_WIDTH_16BITS;
	params->dbi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dbi.data_format.trans_seq   = LCM_DBI_TRANS_SEQ_MSB_FIRST;
	params->dbi.data_format.padding     = LCM_DBI_PADDING_ON_MSB;
	params->dbi.data_format.format      = LCM_DBI_FORMAT_RGB565;
	params->dbi.data_format.width       = LCM_DBI_DATA_WIDTH_16BITS;
	params->dbi.cpu_write_bits          = LCM_DBI_CPU_WRITE_16_BITS;
	params->dbi.io_driving_current      = 0;

	params->dbi.parallel.write_setup    = 2;
	params->dbi.parallel.write_hold     = 2;
	params->dbi.parallel.write_wait     = 4;
	params->dbi.parallel.read_setup     = 2;
	params->dbi.parallel.read_latency   = 31;
	params->dbi.parallel.wait_period    = 9;

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

}



static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	printf("\n[seosctest] lcm_init");

	init_lcm_registers();
}


static void lcm_suspend(void)
{
    sw_clear_panel(0);
	send_ctrl_cmd(0x10);

	printf("\n[seosctest] lcm_suspend");
	MDELAY(5);
}


static void lcm_resume(void)
{
	//send_ctrl_cmd(0x11);
	lcm_init();

	printf("\n[seosctest] lcm_resume");
	MDELAY(120);
}

static void lcm_update(unsigned int x, unsigned int y,
		unsigned int width, unsigned int height)
{
	unsigned short x0, y0, x1, y1;
	unsigned short h_X_start,l_X_start,h_X_end,l_X_end,h_Y_start,l_Y_start,h_Y_end,l_Y_end;

	x0 = (unsigned short)x;
	y0 = (unsigned short)y;
	x1 = (unsigned short)x+width-1;
	y1 = (unsigned short)y+height-1;


    h_X_start=((x0&0xFF00)>>8);
    l_X_start=(x0&0x00FF);
    h_X_end=((x1&0xFF00)>>8);
    l_X_end=(x1&0x00FF);

    h_Y_start=((y0&0xFF00)>>8);
    l_Y_start=(y0&0x00FF);
    h_Y_end=((y1&0xFF00)>>8);
    l_Y_end=(y1&0x00FF);

	send_ctrl_cmd(0x2A);
	send_data_cmd(h_X_start);
	send_data_cmd(l_X_start);
	send_data_cmd(h_X_end);
	send_data_cmd(l_X_end);

	send_ctrl_cmd(0x2B);
	send_data_cmd(h_Y_start);
	send_data_cmd(l_Y_start);
	send_data_cmd(h_Y_end);
	send_data_cmd(l_Y_end);

	send_ctrl_cmd(0x29);

	send_ctrl_cmd(0x2C);
}

static void lcm_setbacklight(unsigned int level)
{
	if(level > 255) level = 255;
#if 0
	send_ctrl_cmd(0x51);
	send_data_cmd(level);
#else
    send_ctrl_cmd(0xBE);
    send_data_cmd(0x0F);
#endif
}
static unsigned int lcm_compare_id(void)
{
#if 0
    send_ctrl_cmd(0xB9);  // SET password
	send_data_cmd(0xFF);
	send_data_cmd(0x83);
	send_data_cmd(0x69);
    send_ctrl_cmd(0xC3);
	send_data_cmd(0xFF);

	send_ctrl_cmd(0xF4);
	read_data_cmd();
    return (LCM_ID == read_data_cmd())?1:0;
#else
    return 1;
#endif
}

static void lcm_set_pwm(unsigned int divider)
{
#if 0
 send_ctrl_cmd(0xBE);
 send_data_cmd(0xFF);

 send_ctrl_cmd(0xBF);
 send_data_cmd(0x07);
#endif
}

#endif

LCM_DRIVER nt35510_dsi_cmd_6572_drv =
{
    .name			= " nt35510_dsi_cmd_6572",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.update         = lcm_update,
	.set_backlight	= lcm_setbacklight,
	.set_pwm        = lcm_set_pwm,
	.compare_id     = lcm_compare_id,
};

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
/*
LCM_DRIVER nt35510_dsi_cmd_6572_drv = {
    .name = "nt35510_dsi_cmd_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    //.set_backlight = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    //.compare_id = lcm_compare_id,
    .update = lcm_update
};
*/
#if 0  /*                                                     */
#ifdef BUILD_LK
    #include <string.h>
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


#if defined(BUILD_LK)
    #define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
    #define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)  // pixel
#define FRAME_HEIGHT (800)  // pixel

#define PHYSICAL_WIDTH  (56)  // mm
#define PHYSICAL_HEIGHT (93)  // mm

#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA    // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_ID       (0x55)
#define LCM_ID1       (0xBC)
#define LCM_ID2       (0xc0)
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
        case REGFLAG_DELAY:
            MDELAY(table[i].count);
            break;

        case REGFLAG_END_OF_TABLE:
            break;

        default:
            dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);

            if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                //#if defined(BUILD_UBOOT)
                //  printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                //#endif
                while (read_reg(cmd) != table[i].para_list[0]);
            }
        }
    }
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{

    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;

    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // physical size
    params->physical_width = PHYSICAL_WIDTH;
    params->physical_height = PHYSICAL_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

    params->dsi.mode = CMD_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

    params->dsi.intermediat_buffer_num = 0; //because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count = 480 * 3;   //DSI CMD mode need set these two bellow params, different to 6577
    params->dsi.vertical_active_line = 800;
    params->dsi.compatibility_for_nvk = 0;  // this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's

    // Bit rate calculation
    params->dsi.PLL_CLOCK = 221; //dsi clock customization: should config clock value directly
}

static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    //*************Enable TE  *******************//
    data_array[0]= 0x00053902;
    data_array[1]= 0x2555aaff;
    data_array[2]= 0x00000001;
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0]= 0x00093902;
    data_array[1]= 0x000201f8;
    data_array[2]= 0x00133320;
    data_array[3]= 0x00000048;
    dsi_set_cmdq(data_array, 4, 1);
    
    //*************Enable CMD2 Page1  *******************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 3, 1);
    
    //************* AVDD: manual  *******************//
    data_array[0]=0x00043902;
    data_array[1]=0x0d0d0db0;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b6;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x0d0d0db1;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b7;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x000000b2;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x242424b8;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x000001bf;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x0f0f0fb3;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x343434b9;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x080808b5;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x000003c2;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x242424ba;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x007800bd;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x006400be;
    dsi_set_cmdq(data_array, 2, 1);
    
    //*************Gamma Table  *******************//
    data_array[0]=0x00353902;
    data_array[1]=0x003300D1;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D2;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D3;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D4;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D5;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    
    data_array[0]=0x00353902;
    data_array[1]=0x003300D6;
    data_array[2]=0x003A0034;
    data_array[3]=0x005C004A;
    data_array[4]=0x00A60081;
    data_array[5]=0x011301E5;
    data_array[6]=0x01820154;
    data_array[7]=0x020002CA;
    data_array[8]=0x02340201;
    data_array[9]=0x02840267;
    data_array[10]=0x02B702A4;
    data_array[11]=0x02DE02CF;
    data_array[12]=0x03FE02F2;
    data_array[13]=0x03330310;
    data_array[14]=0x0000006D;
    dsi_set_cmdq(data_array, 15, 1);
    MDELAY(10);

    // ********************  EABLE CMD2 PAGE 0 **************//
    data_array[0]=0x00063902;
    data_array[1]=0x52aa55f0;
    data_array[2]=0x00000008;
    dsi_set_cmdq(data_array, 3, 1);
    
    // ********************  EABLE DSI TE **************//
    data_array[0]=0x00033902;
    data_array[1]=0x0000fcb1;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00023902;
    data_array[1]=0x000005b6;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00033902;
    data_array[1]=0x007070b7;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00053902;
    data_array[1]=0x030301b8;
    data_array[2]=0x00000003;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]=0x00043902;
    data_array[1]=0x000002bc;
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0]=0x00063902;
    data_array[1]=0x5002d0c9;
    data_array[2]=0x00005050;
    dsi_set_cmdq(data_array, 3, 1);
    
    // ********************  EABLE DSI TE packet **************//
    data_array[0]=0x00351500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]=0x773a1500;
    dsi_set_cmdq(data_array, 1, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0100002a;
    data_array[2]= 0x000000df;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0]= 0x00053902;
    data_array[1]= 0x0300002b;
    data_array[2]= 0x0000001f;
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    
    data_array[0]= 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(20);

    init_lcm_registers();
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    data_array[0] = 0x014F1500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(40);
}


static void lcm_resume(void)
{
    lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
    unsigned int x0 = x;
    unsigned int y0 = y;
    unsigned int x1 = x0 + width - 1;
    unsigned int y1 = y0 + height - 1;

    unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
    unsigned char x0_LSB = (x0 & 0xFF);
    unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
    unsigned char x1_LSB = (x1 & 0xFF);
    unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
    unsigned char y0_LSB = (y0 & 0xFF);
    unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
    unsigned char y1_LSB = (y1 & 0xFF);

    unsigned int data_array[16];


    data_array[0] = 0x00053902;
    data_array[1] = (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
    data_array[2] = (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00053902;
    data_array[1] = (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
    data_array[2] = (y1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x002c3909;
    dsi_set_cmdq(data_array, 1, 0);
}


static void lcm_setbacklight(unsigned int level)
{
    unsigned int data_array[16];


    if (level > 255)
        level = 255;

    data_array[0] = 0x00023902;
    data_array[1] = (0x51 | (level << 8));
    dsi_set_cmdq(data_array, 2, 1);
}


static void lcm_setpwm(unsigned int divider)
{
    // TBD
}


static unsigned int lcm_getpwm(unsigned int divider)
{
    // ref freq = 15MHz, B0h setting 0x80, so 80.6% * freq is pwm_clk;
    // pwm_clk / 255 / 2(lcm_setpwm() 6th params) = pwm_duration = 23706
    unsigned int pwm_clk = 23706 / (1 << divider);


    return pwm_clk;
}


static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0, id2 = 0;
    unsigned char buffer[2];
    unsigned int data_array[16];


    SET_RESET_PIN(1);           //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);

    /*    
       data_array[0] = 0x00110500;        // Sleep Out
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(120);
     */

    //*************Enable CMD2 Page1  *******************//
    data_array[0] = 0x00063902;
    data_array[1] = 0x52AA55F0;
    data_array[2] = 0x00000108;
    dsi_set_cmdq(data_array, 3, 1);
    MDELAY(10);

    data_array[0] = 0x00023700; // read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    read_reg_v2(0xC5, buffer, 2);
    id = buffer[0];             //we only need ID
    id2 = buffer[1];            //we test buffer 1

    return (LCM_ID == id) ? 1 : 0;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35510_dsi_cmd_6572_drv = {
    .name = "nt35510_dsi_cmd_6572",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .set_backlight = lcm_setbacklight,
    //.set_pwm        = lcm_setpwm,
    //.get_pwm        = lcm_getpwm,
    .compare_id = lcm_compare_id,
    .update = lcm_update
};

#endif /*              */
