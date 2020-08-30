#include <linux/module.h>
#include "partition_define.h"
struct excel_info PartInfo[PART_NUM]={
			{"preloader",262144,0x0, EMMC, 0,BOOT_1},
			{"_ext_pl",12320768,0x40000, EMMC, 0,BOOT_1},
			{"mbr",524288,0xc00000, EMMC, 0,USER},
			{"ebr1",524288,0xc80000, EMMC, 1,USER},
			{"misc2",8388608,0xd00000, EMMC, 0,USER},
			{"pro_info",3145728,0x1500000, EMMC, 0,USER},
			{"nvram",5242880,0x1800000, EMMC, 0,USER},
			{"protect_f",10485760,0x1d00000, EMMC, 2,USER},
			{"protect_s",10485760,0x2700000, EMMC, 3,USER},
			{"seccfg",131072,0x3100000, EMMC, 0,USER},
			{"uboot",393216,0x3120000, EMMC, 0,USER},
			{"bootimg",16777216,0x3180000, EMMC, 0,USER},
			{"recovery",16777216,0x4180000, EMMC, 0,USER},
			{"sec_ro",262144,0x5180000, EMMC, 0,USER},
			{"misc",524288,0x51c0000, EMMC, 0,USER},
			{"logo",3145728,0x5240000, EMMC, 0,USER},
			{"expdb",10485760,0x5540000, EMMC, 0,USER},
			{"android",681574400,0x5f40000, EMMC, 4,USER},
			{"cache",394264576,0x2e940000, EMMC, 5,USER},
			{"usrdata",1379926016,0x46140000, EMMC, 6,USER},
			{"fat",0,0x98540000, EMMC, 7,USER},
			{"bmtpool",22020096,0xFFFF00a8, EMMC, 0,USER},
 };
EXPORT_SYMBOL(PartInfo);

#if defined(MTK_EMMC_SUPPORT) || defined(CONFIG_MTK_EMMC_SUPPORT)
struct MBR_EBR_struct MBR_EBR_px[MBR_COUNT]={
	{"mbr", {1, 2, 3, 4, }},
	{"ebr1", {5, 6, 7, }},
};

EXPORT_SYMBOL(MBR_EBR_px);
#endif

