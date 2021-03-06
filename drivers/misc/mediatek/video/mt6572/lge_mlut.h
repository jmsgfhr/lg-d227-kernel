/* drivers/video/msm/lge_mtklut.h
 * Copyright (c) 2011, LG Electronics. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

extern unsigned int p_lg_mtk_lcdc_lut[];

#define NUM_QLUT 256
#if 0 //def LUT_10BIT_MASK

#define R_MASK 0x3ff00000
#define G_MASK 0x000003ff
#define B_MASK 0x000ffc00
#define R_SHIFT 20
#define G_SHIFT 0
#define B_SHIFT 10

#endif
#define R_MASK 0x00ff0000
#define G_MASK 0x000000ff
#define B_MASK 0x0000ff00
#define R_SHIFT 16
#define G_SHIFT 0
#define B_SHIFT 8

extern int g_kcal_r;
extern int g_kcal_g;
extern int g_kcal_b;

#define MAX_KCAL_V (NUM_QLUT-1)
#define lut2r(lut) ((lut & R_MASK) >> R_SHIFT)
#define lut2g(lut) ((lut & G_MASK) >> G_SHIFT)
#define lut2b(lut) ((lut & B_MASK) >> B_SHIFT)

#define scaled_by_kcal(rgb, kcal) \
        (((((unsigned int)(rgb) * (unsigned int)(kcal)) << 16) / (unsigned int)MAX_KCAL_V) >> 16)
