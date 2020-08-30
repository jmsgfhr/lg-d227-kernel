#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#else
#include <linux/string.h>
#if defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
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
#if 1  /*                                                                           */
#include <linux/delay.h>
#define MDELAY(n) (mdelay(n))
#else
#define MDELAY(n) (lcm_util.mdelay(n))
#endif  /*                                                                           */

#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

#if 1  /*                                                                    */
#define LCD_MAKER_ID        GPIO98
#endif  /*                                                                    */

#if 1  /*                                                              */
static int esd_check_period = 0;
#endif  /*                                                              */

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
#if 1 //                                                                                                          
     send_ctrl_cmd(0xCF);	// EXTC Option
    send_data_cmd(0x00);
        send_data_cmd(0xA1);  // 0x21 -> 0xA1 : low temp white screen issue fix
    send_data_cmd(0x20);

    send_ctrl_cmd(0xF2); // 3-Gamma Function Off
        send_data_cmd(0x02);  // 0x02

     send_ctrl_cmd(0xB4); // Inversion Control -> 2Dot inversion
     send_data_cmd(0x02);

     send_ctrl_cmd(0xC0); // Powr control 1
        send_data_cmd(0x15);  // 0x11 -> 0x16  -> 0x15
        send_data_cmd(0x15); // 0x11 -> 0x16  -> 0x15

     send_ctrl_cmd(0xC1); // Power control 2
    send_data_cmd(0x04);  // 0x07 -> 0x04

     send_ctrl_cmd(0xC2);	// Powr control 3
     send_data_cmd(0x32);  // 0x43->0x32

     send_ctrl_cmd(0xC5);	// Vcom control 1
     send_data_cmd(0xFC);  // 0xFD -> 0xFC

    send_ctrl_cmd(0xCB);	// V-core Setting
    send_data_cmd(0x31);
    send_data_cmd(0x24);
    send_data_cmd(0x00);
    send_data_cmd(0x34);

     send_ctrl_cmd(0xF6);	// Interface control
     send_data_cmd(0x41);
    send_data_cmd(0x30);
     send_data_cmd(0x00);

     send_ctrl_cmd(0xB7);	// Entry Mode Set
     send_data_cmd(0x06);

     send_ctrl_cmd(0xB1);	// Frame Rate Control
     send_data_cmd(0x00);
        send_data_cmd(0x1D);  // 0x1B  -> 0x1D

     send_ctrl_cmd(0x36);	// Memory Access Control
     send_data_cmd(0x08); 

     send_ctrl_cmd(0xB5);	// Blanking Porch control
     send_data_cmd(0x02);
     send_data_cmd(0x02);
     send_data_cmd(0x0A);
     send_data_cmd(0x14);

     send_ctrl_cmd(0xB6);	// Display Function control
     send_data_cmd(0x02);  // 0x0A -> 0x02 : shaking LCD issue fix         
    send_data_cmd(0x82);
     send_data_cmd(0x27);
    send_data_cmd(0x00);

     send_ctrl_cmd(0x3A);	// Pixel Format->DBI(5=16bit)
    send_data_cmd(0x05);

     send_ctrl_cmd(0x35);	// Tearing Effect Line On
     send_data_cmd(0x00);

     send_ctrl_cmd(0x44);	// Tearing Effect Control Parameter
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0xE0);	// Positive Gamma Correction
    send_data_cmd(0x00);
        send_data_cmd(0x0A);  // 0x0A -> 0x08
        send_data_cmd(0x10);  // 0x11 -> 0x0A
        send_data_cmd(0x05);  // 0x02 -> 0x07
        send_data_cmd(0x14);
        send_data_cmd(0x0A);  // 0x07 -> 0x08
        send_data_cmd(0x3E);   // 0x3E -> 0x3F
        send_data_cmd(0x67);  // 0x43 -> 0x30
        send_data_cmd(0x4D);  // 0x4B -> 0x4A
        send_data_cmd(0x06);  // 0x05 -> 0x04
    send_data_cmd(0x0B);  // 0x0C -> 0x0b
        send_data_cmd(0x0A);
        send_data_cmd(0x1D);  // 0x1D -> 0x1C
        send_data_cmd(0x24);  // 0x23 -> 0x1E
     send_data_cmd(0x0F);

     send_ctrl_cmd(0xE1);	// Negative Gamma Correction
    send_data_cmd(0x00);  // 0x06 -> 0x00
        send_data_cmd(0x1E);  // 0x1A -> 0x21
    send_data_cmd(0x23);  // 0x24 -> 0x23
     send_data_cmd(0x00);  // 0x03 -> 0x00
        send_data_cmd(0x0B);  // 0x0D ->  0x0C
        send_data_cmd(0x02);  // 0x05 -> 0x01
        send_data_cmd(0x32);  // 0x34 -> 0x31
        send_data_cmd(0x15);  // 0x22 -> 0x03
        send_data_cmd(0x41);  // 0x45 -> 0x43
        send_data_cmd(0x03);  // 0x01 -> 0x02
        send_data_cmd(0x09);  // 0x09 -> 0x0c
        send_data_cmd(0x09);  // 0x08 -> 0x07
        send_data_cmd(0x2C);  // 0x30 -> 0x34
        send_data_cmd(0x31);  // 0x35 -> 0x36
    send_data_cmd(0x0F);

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
     send_data_cmd(0x68);  // 0x38 -> 0x68

    send_ctrl_cmd(0x11);  // Exit Sleep
     MDELAY(120);

    send_ctrl_cmd(0x2C);

    MDELAY(80);
    send_ctrl_cmd(0x29);  // LCD on
#else  //                                                      
     send_ctrl_cmd(0xCF);	// EXTC Option
     send_data_cmd(0x20);
     send_data_cmd(0x21);

     send_ctrl_cmd(0xB4); // Inversion Control -> 2Dot inversion
     send_data_cmd(0x02);

     send_ctrl_cmd(0xC0); // Powr control 1
     send_data_cmd(0x14);
     send_data_cmd(0x0F);

     send_ctrl_cmd(0xC1); // Power control 2
     send_data_cmd(0x04);

     send_ctrl_cmd(0xC2);	// Powr control 3
     send_data_cmd(0x32);

     send_ctrl_cmd(0xC5);	// Vcom control 1
     send_data_cmd(0xFC);

     send_ctrl_cmd(0xF6);	// Interface control
     send_data_cmd(0x41);
     send_data_cmd(0x30);
     send_data_cmd(0x00);

     send_ctrl_cmd(0xB7);	// Entry Mode Set
     send_data_cmd(0x06);

     send_ctrl_cmd(0xB1);	// Frame Rate Control
     send_data_cmd(0x00);
     send_data_cmd(0x1F);

     send_ctrl_cmd(0x36);	// Memory Access Control
     send_data_cmd(0x08); // seosc 08 -> C8

     send_ctrl_cmd(0xB5);	// Blanking Porch control
     send_data_cmd(0x02);
     send_data_cmd(0x02);
     send_data_cmd(0x0A);
     send_data_cmd(0x14);

     send_ctrl_cmd(0xB6);	// Display Function control
     send_data_cmd(0x0A);
     send_data_cmd(0x02);
     send_data_cmd(0x27);
     send_data_cmd(0x04);

     send_ctrl_cmd(0x3A);	// Pixel Format->DBI(5=16bit)
     send_data_cmd(0x55);

     send_ctrl_cmd(0x35);	// Tearing Effect Line On
     send_data_cmd(0x00);

     send_ctrl_cmd(0x44);	// Tearing Effect Control Parameter
     send_data_cmd(0x00);
     send_data_cmd(0xEF);

     send_ctrl_cmd(0xE0);	// Positive Gamma Correction
     send_data_cmd(0x08);
     send_data_cmd(0x11);
     send_data_cmd(0x17);
     send_data_cmd(0x07);
     send_data_cmd(0x12);
     send_data_cmd(0x07);
     send_data_cmd(0x39);
     send_data_cmd(0x12);
     send_data_cmd(0x4D);
     send_data_cmd(0x02);
     send_data_cmd(0x0A);
     send_data_cmd(0x09);
     send_data_cmd(0x2E);
     send_data_cmd(0x35);
     send_data_cmd(0x0F);

     send_ctrl_cmd(0xE1);	// Negative Gamma Correction
     send_data_cmd(0x08);
     send_data_cmd(0x0E);
     send_data_cmd(0x13);
     send_data_cmd(0x04);
     send_data_cmd(0x10);
     send_data_cmd(0x04);
     send_data_cmd(0x37);
     send_data_cmd(0x01);
     send_data_cmd(0x49);
     send_data_cmd(0x08);
     send_data_cmd(0x0F);
     send_data_cmd(0x0C);
     send_data_cmd(0x2C);
     send_data_cmd(0x35);
     send_data_cmd(0x0F);

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
#endif  /*                                                                        */
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

#if 1  /*                                                                             */
        params->dbi.parallel.write_setup    = 6;
        params->dbi.parallel.write_hold     = 2;
        params->dbi.parallel.write_wait     = 12;  
#else
	params->dbi.parallel.write_setup    = 2;
	params->dbi.parallel.write_hold     = 2;
	params->dbi.parallel.write_wait     = 4;
#endif  /*                                                                             */
	params->dbi.parallel.read_setup     = 2;
	params->dbi.parallel.read_latency   = 31;
	params->dbi.parallel.wait_period    = 9;

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;
}

static void lcm_init(void)
{
#if 1 //                                                                                                          
        // no power on/off control for TOVIS LCD sleep current consumption issue
        //hwPowerOn(MT6323_POWER_LDO_VCAM_AF, VOL_2800, "2V8_LCD_VCC_MTK_S");
        //MDELAY(1);
        //hwPowerOn(MT6323_POWER_LDO_VGP1, VOL_1800, "1V8_LCD_VIO_MTK_S");
        //MDELAY(1);
#endif  /*                                                                        */
    SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	init_lcm_registers();
	LCM_PRINT("[LCD] lcm_init \n");
}


static void lcm_suspend(void)
{
    #if 1 //                                                                                                           
	send_ctrl_cmd(0x28);
	MDELAY(50);
	
	send_ctrl_cmd(0x10);
	MDELAY(120);

        // no power on/off control for TOVIS LCD sleep current consumption issue
        //hwPowerDown(MT6323_POWER_LDO_VCAM_AF, "2V8_LCD_VCC_MTK_S");
        //hwPowerDown(MT6323_POWER_LDO_VGP1, "1V8_LCD_VIO_MTK_S");
    #else
    //sw_clear_panel(0);
	send_ctrl_cmd(0x10);
	MDELAY(5);
    #endif  /*                                                                        */
	

	LCM_PRINT("[LCD] lcm_suspend \n");
}


static void lcm_resume(void)
{
	lcm_init();
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
static unsigned int lcm_compare_id(void)
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
    unsigned char param_list[16];
} LCM_register_table[] =
{
    {0x09, 5, {0x84, 0x53, 0x06, 0x00, 0x00}},  // Display Status, register for ESD check from ilitek engineer
    {0x0A, 2, {0x9C, 0x00}},  // Display Power mode, register for ESD check from ilitek engineer
    {0xCF, 3, {0x00, 0xA1, 0x20}},
    {0xF2, 1, {0x02}},
    {0xB4, 1, {0x02}},
    {0xC0, 2, {0x15, 0x15}},
    {0xC1, 1, {0x04}},
    {0xC2, 1, {0x32}},
    {0xC5, 1, {0xFC}},
    {0xCB, 4, {0x31, 0x24, 0x00, 0x34}},
    {0xF6, 3, {0x41, 0x30, 0x00}},
    {0xB7, 1, {0x06}},
    {0xB1, 2, {0x00, 0x1D}},
    {0x36, 1, {0x08}},  // no need to check
    {0xB5, 4, {0x02, 0x02, 0x0A, 0x14}},
    {0xB6, 4, {0x0A, 0x82, 0x27, 0x00}},
    {0x3A, 1, {0x05}},  // no need to check  
    {0x35, 1, {0x00}},
    {0x44, 2, {0x00, 0xEF}},  // no need to check
    {0xE0, 15, {0x00, 0x0A, 0x10, 0x05, 0x14, 0x0A, 0x3E, 0x67, 0x4D, 0x06, 0x0B, 0x0A, 0x1D, 0x24, 0x0F}},
    {0xE1, 15, {0x00, 0x1E, 0x23, 0x00, 0x0B, 0x02, 0x32, 0x15, 0x41, 0x03, 0x09, 0x09, 0x2C, 0x31, 0x0F}},
    {0xE2, 16, {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},  // param changed from 0x91 to 0x00, no need to check, color changed
    {0x2A, 4, {0x00, 0x00, 0x00, 0xEF}},  // no need to check
    {0x2B, 4, {0x00, 0x00, 0x01, 0x3F}},  // no need to check
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
        case 0x36 : index = 13; break;        
        case 0xB5 : index = 14; break;
        case 0xB6 : index = 15; break;
        case 0x3A : index = 16; break;        
        case 0x35 : index = 17; break;
        case 0x44 : index = 18; break;                
        case 0xE0 : index = 19; break;
        case 0xE1 : index = 20; break;
        case 0xE2 : index = 21; break;        
        case 0x2A : index = 22; break;        
        case 0x2B : index = 23; break;        
        case 0xE8 : index = 24; break;
        default :         
            LCM_PRINT("[LCD] register_compare, wrong address : 0x%x\n", addr);
            return TRUE;
            break;        
    }

    for(i = 0; i < count; i++)
    {
        if(LCM_register_table[index].param_list[i] != data[i])
        {
                LCM_PRINT("[LCD] register data changed, addr : 0x%x, data : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
                   addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);
            return FALSE;
        }
    }
    return TRUE;
}

bool lcm_register_check(unsigned char addr)
{
    unsigned char data[16] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, 0xFF};
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
        case 0xE2 :  // 16th parameters
            param_cnt = 16;
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
    if(esd_check_period >= 3)
    {
        esd_check_period = 0;    
        if(lcm_register_check(0xF2) && lcm_register_check(0xF6) && lcm_register_check(0xB5) && lcm_register_check(0xE0) && lcm_register_check(0xE1))
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}
    else
    {
        esd_check_period++;
        return FALSE;
    }
}

static unsigned int lcm_esd_recover(void)
{
    LCM_PRINT("\n[LCD] lcm_esd_recover, LCD re-initialize \n");
    lcm_suspend();
    MDELAY(200);
    lcm_init();
    return TRUE;
}

int dump_lcm_register(char *buf)
{
    int i, j;
    unsigned char param_cnt;
    unsigned char addr;
    unsigned char data[16];
    int table_size = sizeof(LCM_register_table)/sizeof(struct __LCM_register_table__);
    int ret = FALSE;  // register data not changed

    for(i = 0; i < table_size; i++)
    {
        memset(data, 0xFF, sizeof(data));
        addr = LCM_register_table[i].address;
        param_cnt = LCM_register_table[i].count;
        register_data_read(addr, param_cnt, &data);
        LCM_PRINT("[LCD] DUMP LCM REGISTER ADDRESS : 0x%x, data : 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
            addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

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
static unsigned int lcm_compare_id_2nd(void)
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
    .name			= " ili9340d_dbi_qvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.update         = lcm_update,
	.set_backlight	= lcm_setbacklight,
	.set_pwm        = lcm_set_pwm,
	.compare_id     = lcm_compare_id,
#if 1  /*                                                                         */
        .esd_check   = lcm_esd_check,
        .esd_recover   = lcm_esd_recover,	
#endif  /*                                                                         */
};

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
#if 0  /*                                                                    */
LCM_DRIVER ili9340d_dbi_qvga_drv_2nd =
{
    .name			= " ili9340d_dbi_qvga_2nd",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
	.update         = lcm_update,
	.set_backlight	= lcm_setbacklight,
	.set_pwm        = lcm_set_pwm,
	.compare_id     = lcm_compare_id_2nd,
#if 0  /*                                                                         */
        .esd_check   = lcm_esd_check,
        .esd_recover   = lcm_esd_recover,	
#endif  /*                                                                         */
};
#endif  /*                                                                    */
