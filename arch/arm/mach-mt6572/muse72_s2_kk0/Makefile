
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36283),y)
obj-y	+=  alsps/cm36283/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009),y)
obj-y   +=  accelerometer/KXTJ2_1009/
endif

# [LGE_BSP_COMMON] CHANGE_S: 2013-10-15 jongwoo82.lee@lge.com  Accel Driver porting
#ifeq ($(CONFIG_BOSCH_BMA255),y)
#obj-y   +=  accelerometer/bma255/
#endif
# [LGE_BSP_COMMON] CHANGE_E: 2013-10-15 jongwoo82.lee@lge.com  Accel Driver porting

# [LGE_BSP_COMMON] CHANGE_S: 2014-01-15 jongwoo82.lee@lge.com  Accel Driver porting
ifeq ($(CONFIG_LGE_L30_REVA),y)
obj-$(CONFIG_STM_K2DH) += accelerometer/k2dh/
else
obj-$(CONFIG_BOSCH_BMA255) += accelerometer/bma255/
endif
# [LGE_BSP_COMMON] CHANGE_E: 2014-01-15 jongwoo82.lee@lge.com  Accel Driver porting

ifeq ($(CONFIG_MTK_MPU3050C),y)
obj-y   +=  gyroscope/mpu3050c/
endif

ifeq ($(CONFIG_MTK_AKM8963),y)
obj-y	+=  magnetometer/akm8963/
endif

ifeq ($(CONFIG_MTK_MT5193),y)
obj-y	+=  matv/mt5193/
endif

ifneq ($(CONFIG_ARCH_MTK_PROJECT),)
ARCH_MTK_PROJECT := $(shell echo $(CONFIG_ARCH_MTK_PROJECT))
obj-y	+= core/
obj-y	+= dct/dct/
endif

obj-y   +=  lcm/
#obj-y	+=  hdmi/
obj-y	+=  camera/camera/
obj-y	+=  imgsensor/t4k28_yuv/

obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/
obj-y	+=  wmt/
