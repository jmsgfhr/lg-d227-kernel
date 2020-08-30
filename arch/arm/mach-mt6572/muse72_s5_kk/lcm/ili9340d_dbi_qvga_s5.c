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
#include <platform/mt_gpio.h>
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif
#endif


#include "lcm_drv.h"

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
#if 1  /*                                                                        */
#include <mach/mt_pm_ldo.h>
#endif  /*                                                                        */
#endif

#if 1  /*                                                                    */
#define LCD_MAKER_ID        GPIO98

static int lcd_first_check = true;
static int is_INN_LCD = true;

static unsigned int lcm_compare_id_INN(void);
static void init_lcm_registers_TCL(void);  // TCL
#endif  /*                                                                    */



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

static void init_lcm_registers_INN(void)
{
        send_ctrl_cmd(0x3A); //16BIT 65K
        send_data_cmd(0x55);

        send_ctrl_cmd(0xCF); //EXTC option
    send_data_cmd(0x00);
        send_data_cmd(0xA1);

        send_ctrl_cmd(0xB1); // Frame rate
     send_data_cmd(0x00);
        send_data_cmd(0x1E); //0x1E is 60Hz, 70Hz is 0x1A

        send_ctrl_cmd(0XB4); //inversion control
        send_data_cmd(0x02); //0x02 is 2-dot inversion, 0x00 is column inversion

        send_ctrl_cmd(0X36);
     send_data_cmd(0x00);

        send_ctrl_cmd(0XB6);
     send_data_cmd(0x02);

        send_ctrl_cmd(0XC0); //power control
        send_data_cmd(0x0F); //
        send_data_cmd(0x0D); //

        send_ctrl_cmd(0xC1); //power control
        send_data_cmd(0x01); //VGH =6x VCI VGL= -4xVCI

        send_ctrl_cmd(0xC2); //power control
        send_data_cmd(0x33);

        send_ctrl_cmd(0xC5); // VCOMDC=-1
        send_data_cmd(0xEB);

        #if 1  /*                                                                                                        */
     send_ctrl_cmd(0x35);	// Tearing Effect Line On
    send_data_cmd(0x00);

     send_ctrl_cmd(0x44);	// Tearing Effect Control Parameter
     send_data_cmd(0x00);
     send_data_cmd(0xEF);
        #endif  /*                                                                                                        */

        send_ctrl_cmd(0xE0); // Set P Gamma //value up brightness up 
        send_data_cmd(0x00); // 0
        send_data_cmd(0x08); // 0x08
        send_data_cmd(0x0E); // 0x0E
        send_data_cmd(0x04); //4
        send_data_cmd(0x12); //6
        send_data_cmd(0x06); //13 
        send_data_cmd(0x2E); //20  32
        send_data_cmd(0x87); //36 27  67
        send_data_cmd(0x40); //43  --3f
        send_data_cmd(0x0A); //50 --08
        send_data_cmd(0x0E); //57 --0e
        send_data_cmd(0x0B); //59  --0a
        send_data_cmd(0x0F); //61---10
        send_data_cmd(0x13); //62
        send_data_cmd(0x0f); //63

        send_ctrl_cmd(0XE1); //Set N Gamma value down brightness down
        send_data_cmd(0x00); //63
        send_data_cmd(0x22); //62
        send_data_cmd(0x23); //61
        send_data_cmd(0x02); //59 //03
        send_data_cmd(0x0D); //57 //0f
        send_data_cmd(0x02); //50 //05
        send_data_cmd(0x39); //43  --3a
        send_data_cmd(0x37); //27 36
        send_data_cmd(0x50); //20 4C
        send_data_cmd(0x04); //13
        send_data_cmd(0x0d); //6
        send_data_cmd(0x0b); //4
        send_data_cmd(0x33);
        send_data_cmd(0x36);  // 36
        send_data_cmd(0x0f); //0

    send_ctrl_cmd(0x11);  // Exit Sleep
     MDELAY(120);

        send_ctrl_cmd(0XF6);
     send_data_cmd(0x41);

        send_ctrl_cmd(0XEE);
        send_data_cmd(0x0C);//Prevent tearing effect

        send_ctrl_cmd(0x29); //display on
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

static void lcm_init_INN(void)
{
#if 1  /*                                                                              */
    if(lcd_first_check == true)  // compare ID only once
    {
        if(lcm_compare_id_INN())  // innolux lcd
        {
            is_INN_LCD = true;
        }
        else
        {
            is_INN_LCD = false;
        }
        LCM_PRINT("[LCD] LCD ID compare, is_INN_LCD : %d\n", is_INN_LCD);            
        lcd_first_check = false;
    }
#endif  /*                                                                              */
        // no power on/off control for TOVIS LCD sleep current consumption issue
        //hwPowerOn(MT6323_POWER_LDO_VCAM_AF, VOL_2800, "2V8_LCD_VCC_MTK_S");
        //MDELAY(1);
        //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_1800, "1V8_LCD_VIO_MTK_S");
        //MDELAY(1);        
    SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
#if 1   /*                                                                              */
        if(is_INN_LCD == true)
        {
            init_lcm_registers_INN();
        }
        else
        {
            init_lcm_registers_TCL();
        }
#else
	init_lcm_registers_INN();
#endif  /*                                                                              */
	LCM_PRINT("[LCD] lcm_init \n");
}


static void lcm_suspend_INN(void)
{

	send_ctrl_cmd(0x28);
	MDELAY(50);
	
	send_ctrl_cmd(0x10);
	MDELAY(120);

        // no power on/off control for TOVIS LCD sleep current consumption issue
        //hwPowerDown(MT6323_POWER_LDO_VCAM_AF, "2V8_LCD_VCC_MTK_S");
        //hwPowerDown(MT6323_POWER_LDO_VGP1, "1V8_LCD_VIO_MTK_S");
	LCM_PRINT("[LCD] lcm_suspend \n");
}


static void lcm_resume_INN(void)
{
	lcm_init_INN();
	MDELAY(120);
	LCM_PRINT("[LCD] lcm_resume \n");
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
static unsigned int lcm_compare_id_INN(void)
{
    if(mt_get_gpio_in(LCD_MAKER_ID) == GPIO_IN_ONE)
    {
        LCM_PRINT("[LCD] 1st LCD detected\n");
        return TRUE;
    }
    else
    {
        return FALSE;
    }
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
#if 1  /*                                                                                */
static struct __LCM_register_table__ {
    unsigned char address;
    unsigned char count;
    unsigned char param_list[15];
} LCM_register_table[] =
{
    {0x09, 5, {0x84, 0x53, 0x06, 0x00, 0x00}},  // Display Status, register for ESD check from ilitek engineer
    {0x0A, 2, {0x9C, 0x00}},  // Display Power mode, register for ESD check from ilitek engineer,
    {0xCF, 3, {0x00, 0xA1, 0x20}},
    {0xF2, 1, {0x02}},
    {0xB4, 1, {0x02}},
    {0xC0, 2, {0x16, 0x16}},
    {0xC1, 1, {0x04}},
    {0xC2, 1, {0x32}},
    {0xC5, 1, {0xFC}},
    {0xCB, 4, {0x31, 0x24, 0x00, 0x34}},
    {0xF6, 3, {0x41, 0x30, 0x00}},
    {0xB7, 1, {0x06}},
    {0xB1, 2, {0x00, 0x1B}},
    {0xB5, 4, {0x02, 0x02, 0x0A, 0x14}},
    {0xB6, 4, {0x0A, 0x82, 0x27, 0x00}},
    {0x35, 1, {0x00}},
    {0xE0, 15, {0x00, 0x08, 0x0A, 0x07, 0x10, 0x08, 0x3F, 0x30, 0x4A, 0x04, 0x0B, 0x08, 0x1C, 0x1E, 0x0F}},
    {0xE1, 15, {0x00, 0x21, 0x23, 0x00, 0x0C, 0x01, 0x31, 0x03, 0x43, 0x02, 0x0C, 0x07, 0x34, 0x36, 0x0F}},
    {0xE8, 3, {0x84, 0x1A, 0x68}},
};

static bool register_data_read(unsigned char addr, unsigned char size, unsigned char data[])
{
    int i;
    
    send_ctrl_cmd(addr);
    read_data_cmd();  // dummy command for correct read
    for(i = 0; i < size; i++)
    {
        data[i] = read_data_cmd();        
    }
    return TRUE;
}

static bool register_compare(unsigned char addr, unsigned char data[], unsigned char count)
{
    unsigned char index = 0;
    unsigned char i;
    
    switch(addr)
    {
        case 0x09 : index = 0; break;
        case 0x0A : index = 1; break;
        case 0xCF : index = 2; break;
        case 0xF2 : index = 3; break;
        case 0xB4 : index = 4; break;
        case 0xC0 : index = 5; break;
        case 0xC1 : index = 6; break;
        case 0xC2 : index = 7; break;        
        case 0xC5 : index = 8; break;
        case 0xCB : index = 9; break;
        case 0xF6 : index = 10; break;
        case 0xB7 : index = 11; break;
        case 0xB1 : index = 12; break;
        case 0xB5 : index = 13; break;
        case 0xB6 : index = 14; break;
        case 0x35 : index = 15; break;
        case 0xE0 : index = 16; break;
        case 0xE1 : index = 17; break;
        case 0xE8 : index = 18; break;
        default :         
            LCM_PRINT("[LCD] register_compare, wrong address : 0x%x\n", addr);
            return TRUE;
            break;        
    }

    for(i = 0; i < count; i++)
    {
        if(LCM_register_table[index].param_list[i] != data[i])
        {
                LCM_PRINT("[LCD] register data changed, addr : 0x%x, data : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
                    addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);
            return FALSE;
        }
    }
    return TRUE;
}

bool lcm_register_check(unsigned char addr)
{
    unsigned char data[15] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
    unsigned char param_cnt;
    unsigned char i;
    bool result;

    switch(addr)
    {
        case 0xF2 :  // Fall through
        case 0xB4 :  // Fall through
        case 0xC1 :  // Fall through
        case 0xC2 :  // Fall through
        case 0xC5 :  // Fall through
        case 0xB7 :  // Fall through
        case 0x36 :  // Fall through
        case 0x3A :  // Fall through
        case 0x35 :  // 1 parameters
            param_cnt = 1;
            break;
        case 0x0A :
        case 0xC0 :  // Fall through
        case 0xB1 :  // Fall through
        case 0x44 :  // 2 parameters
            param_cnt = 2;
            break;
        case 0xCF :  // Fall through
        case 0xF6 :  // Fall through
        case 0xE8 :  // 3 parameters      
            param_cnt = 3;
            break;    
        case 0xCB :  // Fall through
        case 0xB5 :  // Fall through
        case 0xB6 :  // Fall through
        case 0x2A :  // Fall through
        case 0x2B :  // 4th parameters
            param_cnt = 4;
            break;
        case 0x09 :
            param_cnt = 5;
            break;
        case 0xE0 :  // Fall through
        case 0xE1 :  // 15th parameters
            param_cnt = 15;
            break;
        default :
            LCM_PRINT("[LCD] lcm_register_check, wrong address : 0x%x\n", addr);
            return TRUE;
            break;            
    }

        result = register_data_read(addr, param_cnt, &data);

        if(result == TRUE)
        {
            return register_compare(addr, data, param_cnt); 
        }
        else
        {
            LCM_PRINT("[LCD] register data read fail, return TRUE\n");
            return TRUE;
        }
}
#endif  /*                                                                                */

#if 1  /*                                                                         */
static unsigned int lcm_esd_check(void)
{   
    //LCM_PRINT("[LCD] lcm_esd_check\n");
    if( lcm_register_check(0x09) && lcm_register_check(0x0A) && lcm_register_check(0xCF))
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

static unsigned int lcm_esd_recover(void)
{
    LCM_PRINT("\n[LCD] lcm_esd_recover, LCD re-initialize \n");
    lcm_suspend_INN();
    MDELAY(200);
    lcm_init_INN();
    return TRUE;
}

int dump_lcm_register(char *buf)
{
    int i, j;
    unsigned char param_cnt;
    unsigned char addr;
    unsigned char data[15];
    int table_size = sizeof(LCM_register_table)/sizeof(struct __LCM_register_table__);
    int ret = FALSE;  // register data not changed

    for(i = 0; i < table_size; i++)
    {
        memset(data, 0xFF, sizeof(data));
        addr = LCM_register_table[i].address;
        param_cnt = LCM_register_table[i].count;
        register_data_read(addr, param_cnt, &data);
        LCM_PRINT("[LCD] DUMP LCM REGISTER ADDRESS : 0x%x, data : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
            addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14]);

        if(register_compare(addr, data, param_cnt) == FALSE)
        {
            sprintf(buf, "%s 0x%x", buf, addr);
            ret = TRUE;  // register data changed
        }
    }
    sprintf(buf, "%s\0", buf);

    return ret;
}
#endif  /*                                                                         */

#if 1  /*                                                                    */
static void init_lcm_registers_TCL(void)  // TCL
{
    send_ctrl_cmd(0xCF);  
    send_data_cmd(0x00); 
        send_data_cmd(0x01); 
    
    send_ctrl_cmd(0xC0);//Power control
        send_data_cmd(0x12);//VRH[5:0]
    send_data_cmd(0x12);//
    
    send_ctrl_cmd(0xC1);//Power control
        send_data_cmd(0x05);//SAP[2:0];BT[3:0]
    
    
    send_ctrl_cmd(0xC2); 
    send_data_cmd(0x32);
    
    send_ctrl_cmd(0xC5); 
        send_data_cmd(0xf5); 
     
     
    send_ctrl_cmd(0x36);// Memory Access Control
    send_data_cmd(0x08);
    
    send_ctrl_cmd(0x3A); 
    send_data_cmd(0x55);
    
    send_ctrl_cmd(0xB1);// Frame Rate Control
    send_data_cmd(0x00);
        send_data_cmd(0x1B);//0x17--83Hz,0x1b--70Hz
    
    send_ctrl_cmd(0xb4); 
        send_data_cmd(0x00);  // column inversion
    
    send_ctrl_cmd(0x20); 
    
    send_ctrl_cmd(0xb7); 
    send_data_cmd(0x06);    
     
    send_ctrl_cmd(0xE0);//Set Gamma
    send_data_cmd(0x00);
    send_data_cmd(0x08);
    send_data_cmd(0x08);
        send_data_cmd(0x03);
        send_data_cmd(0x12);
            send_data_cmd(0x07);
            send_data_cmd(0x3C);
            send_data_cmd(0x77);
            send_data_cmd(0x4C);
        send_data_cmd(0x1D);
            send_data_cmd(0x13);
        send_data_cmd(0x0F);
            send_data_cmd(0x17);
            send_data_cmd(0x18);
            send_data_cmd(0x00);
    
    send_ctrl_cmd(0XE1);//Set Gamma
            send_data_cmd(0x0F);
        send_data_cmd(0x1D);
            send_data_cmd(0x24);
        send_data_cmd(0x06);
        send_data_cmd(0x12);
        send_data_cmd(0x06);
            send_data_cmd(0x3D);
            send_data_cmd(0x13);
            send_data_cmd(0x4F);
            send_data_cmd(0x04);
        send_data_cmd(0x0F);
        send_data_cmd(0x0D);
        send_data_cmd(0x38);
            send_data_cmd(0x3C);
    send_data_cmd(0x0F);
    
        send_ctrl_cmd(0x35);  // TE on
        send_data_cmd(0x00);
        
    send_ctrl_cmd(0x11);//Exit Sleep
        MDELAY(150);
    
        send_ctrl_cmd(0xf6);  
        send_data_cmd(0x41); 
        send_data_cmd(0x30); 
        send_data_cmd(0x00); 
    
    send_ctrl_cmd(0XEE);//Set MADCTL
        send_data_cmd(0x0C);
    
    send_ctrl_cmd(0x29);//display on
    MDELAY(20);
    
    send_ctrl_cmd(0x2c);//Memory Write        
}


static void lcm_init_TCL(void)
{
        // no power on/off control for TOVIS LCD sleep current consumption issue
        //hwPowerOn(MT6323_POWER_LDO_VCAM_AF, VOL_2800, "2V8_LCD_VCC_MTK_S");
        //MDELAY(1);
        //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_1800, "1V8_LCD_VIO_MTK_S");
        //MDELAY(1);
            
        SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	init_lcm_registers_TCL();


	LCM_PRINT("[LCD] lcm_init_2nd \n");
}

static void lcm_suspend_TCL(void)
{
	send_ctrl_cmd(0x28);
	MDELAY(50);	
	
	send_ctrl_cmd(0x10);
	MDELAY(120);
    // no power on/off control for TOVIS LCD sleep current consumption issue
    //hwPowerDown(MT6323_POWER_LDO_VCAM_AF, "2V8_LCD_VCC_MTK_S");
    //hwPowerDown(MT6323_POWER_LDO_VGP1, "1V8_LCD_VIO_MTK_S");
	LCM_PRINT("[LCD] lcm_suspend_2nd \n");
}


static void lcm_resume_TCL(void)
{
	lcm_init_TCL();
	MDELAY(120);
	LCM_PRINT("[LCD] lcm_resume_2nd \n");
}


static unsigned int lcm_compare_id_TCL(void)
{
    if(mt_get_gpio_in(LCD_MAKER_ID) == GPIO_IN_ZERO )
    {
        LCM_PRINT("[LCD] 2nd LCD detected\n");
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}
#endif  /*                                                                    */

LCM_DRIVER ili9340d_dbi_qvga_drv =
{
    .name			= " ili9340d_dbi_qvga_INN",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init_INN,
	.suspend        = lcm_suspend_INN,
	.resume         = lcm_resume_INN,
	.update         = lcm_update,
	.set_backlight	= lcm_setbacklight,
	.set_pwm        = lcm_set_pwm,
	.compare_id     = lcm_compare_id_INN,
#if 0  /*                                                                         */
        .esd_check   = lcm_esd_check,
        .esd_recover   = lcm_esd_recover,	
#endif  /*                                                                         */
};

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
#if 1  /*                                                                    */
LCM_DRIVER ili9340d_dbi_qvga_drv_2nd =
{
        .name			= " ili9340d_dbi_qvga_TCL",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init_TCL,
    .suspend = lcm_suspend_TCL,
    .resume = lcm_resume_TCL,
	.update         = lcm_update,
	.set_backlight	= lcm_setbacklight,
	.set_pwm        = lcm_set_pwm,
        .compare_id     = lcm_compare_id_TCL,
#if 0  /*                                                                         */
        .esd_check   = lcm_esd_check,
        .esd_recover   = lcm_esd_recover,	
#endif  /*                                                                         */
};
#endif  /*                                                                    */
