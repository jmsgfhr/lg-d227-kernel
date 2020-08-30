
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Module project related part

ifeq ($(CONFIG_MTK_CM36283),y)
obj-y	+=  alsps/cm36283/
endif

ifeq ($(CONFIG_MTK_APDS9190),y)
obj-y	+=  alsps/apds9190/
endif

ifeq ($(CONFIG_BOSCH_BMA255),y)
obj-y	+=  accelerometer/bma255/
endif

ifeq ($(CONFIG_LGE_L50_REVA),y)
obj-$(CONFIG_STM_K2DH) += accelerometer/k2dh/
else
obj-$(CONFIG_MTK_BMA255) += accelerometer/bma255_l50/
endif

ifeq ($(CONFIG_LGE_L50_REVA),y)
obj-$(CONFIG_ALPS_HSCDTD008) +=  magnetometer/hscdtd008/
else
obj-$(CONFIG_BOSCH_BMM050) +=  magnetometer/bmm050/
endif

ifeq ($(CONFIG_MTK_KXTJ2_1009),y)
obj-y   +=  accelerometer/KXTJ2_1009/
endif

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
obj-y	+=  imgsensor/isx012_mipi_yuv/
obj-y	+=  imgsensor/hi351_mipi_yuv/
obj-y	+=  imgsensor/hi707_yuv/

obj-y	+=  vibrator/
obj-y	+=  leds/
obj-y	+=  keypad/
obj-y	+=  sound/
obj-y	+=  accdet/
obj-y	+=  wmt/
