#include <cust_leds.h>
#include <cust_leds_def.h>
#include <mach/mt_pwm.h>

#include <linux/kernel.h>
#include <mach/pmic_mt6329_hw_bank1.h>
#include <mach/pmic_mt6329_sw_bank1.h>
#include <mach/pmic_mt6329_hw.h>
#include <mach/pmic_mt6329_sw.h>
#include <mach/upmu_common_sw.h>
#include <mach/upmu_hw.h>
#if 1  /*                                                                */
#include <mach/mt_gpio.h>
#include <linux/delay.h>

static DEFINE_SPINLOCK(led_spin);

#if 1  /*                                                                          */
// PP2
unsigned char bright_arr[] = {
31,31,31,31,31,31,31,31,31,31,
31,31,31,31,31,30,30,30,30,30,
30,30,30,30,30,30,30,30,30,30,
30,30,30,29,29,29,29,29,29,29,
29,29,28,28,28,28,28,28,27,27,
27,27,26,26,26,26,25,25,25,24,
24,23,23,23,22,22,22,21,21,20,
20,20,19,19,19,18,18,17,17,16,
15,15,15,14,14,13,13,12,12,11,
10,9,9,8,8,7,7,6,5,4,3};
#else  // PP1
unsigned char bright_arr[] = {
30,30,30,30,30,30,30,30,30,30,
30,30,30,30,30,29,29,29,29,29,
29,29,29,29,29,29,29,29,29,29,
29,29,29,28,28,28,28,28,28,28,
28,28,27,27,27,27,27,27,26,26,
26,26,25,25,25,25,24,24,24,24,
24,23,23,23,22,22,22,22,21,21,
21,20,20,20,19,19,19,18,18,18,
17,17,16,15,15,15,14,14,13,13,
12,11,11,10,10,9,9,8,8,7,7};
#endif  /*                                                                          */

static unsigned char previous_blu_level=0;
static unsigned char previous_blu_cnt=0;
static unsigned char current_blu_cnt=0;
static unsigned char isbooting = 0;

int Cust_SetBacklight(int level)
{
		int cnt = 0;
		int blu_percent = 0;
		int i =0;

		if ((previous_blu_level == level) && (level != 0))
		{
			printk("[LED] Cust_SetBacklight [no need to update!! - same setting pre[%d],curr[%d] ]  \n",previous_blu_level,level);
			return 0;
		}

		printk("[LED] Cust_SetBacklight level[%d] \n", level);

		if(level > 0 )
		{
			blu_percent = (level*100)/255; // change from 255 levels to 100 levels
			current_blu_cnt = bright_arr[blu_percent];
			printk("[LED] Cust_SetBacklight Turn-On - blu_percent [%d] \n", blu_percent);

			if(previous_blu_level == 0) // when off -> on , previous_blu_cnt == 0
			{
				cnt = current_blu_cnt;
				printk("[LED] previous level = 0\n ");

			}
			else // no need to update same cnt
			{
				if (previous_blu_cnt > current_blu_cnt) // increasing brightness level
				{
					cnt = 32 - (previous_blu_cnt - current_blu_cnt);
					printk("[LED] increasing cnt[%d] = 32 - (prev_cnt[%d] - curr_cnt[%d]) \n",cnt, previous_blu_cnt,current_blu_cnt);
				}
				else if(previous_blu_cnt < current_blu_cnt) // decreasing brightness level
				{
					cnt = current_blu_cnt -previous_blu_cnt;
					printk("[LED] decreasing cnt[%d] = curr_cnt[%d] - prev_cnt[%d] \n",cnt, current_blu_cnt,previous_blu_cnt);
				}
				else // previous_blu_cnt == current _blu cnt
				{
						previous_blu_level = level;
						previous_blu_cnt = current_blu_cnt;
						printk("[LED] return curr_cnt[%d] == prev_cnt[%d] \n",current_blu_cnt,previous_blu_cnt);
						return 0;
				}
			}

			if(isbooting == 0) // in the  power on case(only one time), lk is alway 100% on.
			{
				unsigned int gpio_status= 0;

				printk("[LED] isbooting == 1\n");
				previous_blu_cnt = 0;
				isbooting =1;

				gpio_status = mt_get_gpio_out(GPIO144);
				printk("[LED] CHARE_EN is [%d]\n",gpio_status);

				if (gpio_status == 0)
				{
					printk("[LED] error!! error!! error!!when Power on, GPIO should high \n");
					//ASSERT(0);
				}

				if (cnt == 0) // Setting value from kernel is 100% case, no need to update.
				{
					printk("[LED] power on case, no need to update \n");
				}
				else // update
				{
					printk("[LED] power on case, update\n");
                                    spin_lock(&led_spin);
					mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);

					if((cnt>0) && (previous_blu_cnt != current_blu_cnt))
					{
						for ( i=0;i<cnt;i++)
						{
							mt_set_gpio_out(GPIO144, GPIO_OUT_ZERO);
							udelay(1);
							mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);
							udelay(2);
						}
					}
                                    spin_unlock(&led_spin);					
				}
			}
			else // Sleep out case Or adjusging brightness case,
			{
				printk("[LED] sleep out case Or adjusging brightness case, cnt : %d\n", cnt);

                            spin_lock(&led_spin);
				mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);

				if (previous_blu_level > 0)
				{
				  udelay(5);
				}
				else
				{
	 			 udelay(35); // first High duration is over 30 us
				}

                            if((cnt>0) && (previous_blu_cnt != current_blu_cnt))
				{
					for ( i=0;i<cnt;i++)
					{
						mt_set_gpio_out(GPIO144, GPIO_OUT_ZERO);
						udelay(1);
						mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);
						udelay(2);
					}
				}
                            spin_unlock(&led_spin);
				
			}

			previous_blu_level = level;
			previous_blu_cnt = current_blu_cnt;
		}
		else
		{
			printk("[LED] Cust_SetBacklight - TURN_OFF \n ");
			mt_set_gpio_out(GPIO144, GPIO_OUT_ZERO);
			mdelay(4);
			previous_blu_level = level;
			previous_blu_cnt = 0;
		}

	return 0;
}

#endif /*              */

//extern int mtkfb_set_backlight_level(unsigned int level);
//extern int mtkfb_set_backlight_pwm(int div);
extern int disp_bls_set_backlight(unsigned int level);
/*
#define ERROR_BL_LEVEL 0xFFFFFFFF

unsigned int brightness_mapping(unsigned int level)
{
	return ERROR_BL_LEVEL;
}
*/
unsigned int brightness_mapping(unsigned int level)
{
    unsigned int mapped_level;

    mapped_level = level;

	return mapped_level;
}

#if 0 //                                            
unsigned int Cust_SetBacklight(int level, int div)
{
    //mtkfb_set_backlight_pwm(div);
    //mtkfb_set_backlight_level(brightness_mapping(level));
    disp_bls_set_backlight(brightness_mapping(level));
    return 0;
}
#endif

#if 1  /*                                                                             */
int Cust_SetBacklight_test(int cnt)  // RT9393 test interface, cnt 0~31
{
    int i =0;

    printk("[LED][TEST] adjusting LCD backlight brightness : %d\n", cnt);

    mt_set_gpio_out(GPIO144, GPIO_OUT_ZERO);  // Charge pump off
    mdelay(5);
    spin_lock(&led_spin);
    mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);
    udelay(35);

    if((cnt > 0) && (cnt <= 31))  // set the brightness
    {
        for(i = 0; i < cnt; i++)
        {
            mt_set_gpio_out(GPIO144, GPIO_OUT_ZERO);
            udelay(1);
            mt_set_gpio_out(GPIO144, GPIO_OUT_ONE);
            udelay(2);
        }        
    }
    spin_unlock(&led_spin); 

    previous_blu_cnt = cnt;

    for(i = 0; i <= 100; i++)  // find 100% unit level by cnt
    {
        if(bright_arr[i] < cnt)
        {
            break;
        }
    }
    if(i >= 100)
    {
        previous_blu_level = 255;
    }
    else
    {
        previous_blu_level = ((i-1)*255)/100;  // convert to 0~255
    }

    printk("[LED][TEST] save cnt : %d and prevous blu level : %d\n", previous_blu_cnt, previous_blu_level);

    return 0;
}
#endif  /*                                                                             */

static struct cust_mt65xx_led cust_led_list[MT65XX_LED_TYPE_TOTAL] = {
	{"red",               MT65XX_LED_MODE_NONE, -1, {0}},
	{"green",             MT65XX_LED_MODE_NONE, -1, {0}},
	{"blue",              MT65XX_LED_MODE_NONE, -1, {0}},
	{"jogball-backlight", MT65XX_LED_MODE_NONE, -1, {0}},
	{"keyboard-backlight",MT65XX_LED_MODE_NONE, -1, {0}},
	{"button-backlight",  MT65XX_LED_MODE_NONE, -1, {0}},
#if 1  //                                            
	{"lcd-backlight",	  MT65XX_LED_MODE_CUST_LCM, (int)Cust_SetBacklight,{0}},
#else
	{"lcd-backlight",     MT65XX_LED_MODE_PMIC, MT65XX_LED_PMIC_LCD_ISINK, {0}},
#endif
};

struct cust_mt65xx_led *get_cust_led_list(void)
{
	return cust_led_list;
}

#if 1  /*                                                                             */
int get_cust_backlight_test(int cnt)
{
    return (int)Cust_SetBacklight_test(cnt);
}
#endif  /*                                                     */

