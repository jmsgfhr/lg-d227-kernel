include $(srctree)/drivers/misc/mediatek/Makefile.custom

#obj-y	:=  yusu_android_speaker.o
# [LGE_BSP_COMMON] CHANGE_E: 2014-01-17 Max.chung@lge.com  HEADSET AMP Driver porting
ifeq ($(CONFIG_LGE_L30_REVA),y)
obj-y	+=  amp_tpa6136a2/
else
obj-y	+=  amp_tpa2058d3/
endif
# [LGE_BSP_COMMON] CHANGE_E: 2014-01-17 Max.chung@lge.com  HEADSET AMP Driver porting

