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

#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
    #include <platform/mt_pmic.h>
	#undef printk
	#define printk printf
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
	#undef printk
	#define printk printf	
#else
    #include <mach/mt_pm_ldo.h>
	#include <mach/mt_gpio.h>
	#undef printk
	#define printk printk	
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define REGFLAG_DELAY             							0xAB
#define REGFLAG_END_OF_TABLE      							0xAA   // END OF REGISTERS MARKER
#define GPIO_LCM_DCDC_ENP_PIN    (GPIO92 | 0x80000000)  
//#define LCM_DCDC_ENN_PIN    (GPIO93 | 0x80000000)  
#define ON    1
#define OFF   0  
#define LCM_ID_NT35590 (0x90)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define LCM_HW_BYD_ID   (0x00)
#define LCM_HW_TRY_ID   (0x01)


#define GPIO_LCM_RST_PIN         (GPIO112 | 0x80000000)          //131
#define GPIO_LCM_IDO_PIN         (GPIO70  | 0x80000000)          //11//74
//#define LCM_ID1_PIN         (GPIO41  | 0x80000000)          //76

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
static unsigned int lcm_compare_id(void);
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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   

#define   LCM_DSI_CMD_MODE							1

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...

	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	
	{0xFF,	4,	{0xAA,0x55,0xA5,0x80}},
	{0x6F,	1,	{0x13}},		
	{0xF7,	1,	{0x00}},
	///////////2014.7.18 解决低温-20°开机闪花屏
	{0x6F,	1,	{0x08}},		
	{0xFc,	1,	{0x04}},
	////////////
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x00}},
	//{REGFLAG_DELAY, 5, {}},
	
	{0xB1,	2,	{0x68,0x21}},////68->cmd mode,78->video mod
	{0xB5,	1,	{0xC8}},		
	{0xB6,	1,	{0x12}},
	{0xB8,	4,	{0x00,0x00,0x0A,0x00}},
	{0xB9,	1,	{0x00}},		
	{0xBA,	1,	{0x02}},	
	{0xBB,	2,	{0x63,0x63}},
	{0xBC,	2,	{0x00,0x00}},	//00 00	
	{0xBD,	5,	{0x02,0x7F,0x0D,0x0B,0x00}},
		
	{0xCC,	16,	{0x41,0x36,0x87,0x54,0x46,0x65,0x10,0x12,0x14,0x10,0x12,0x14,0x40,0x08,0x15,0x05}},	
	{0xD0,	1,	{0x00}},			
	{0xD1,	16,	{0x00,0x04,0x08,0x0C,0x10,0x14,0x18,0x1C,0x20,0x24,0x28,0x2C,0x30,0x34,0x38,0x3C}},	
	{0xD3,	1,	{0x00}},		
	{0xD6,	2,	{0x44,0x44}},		
	{0xD7,	12,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},	
	{0xD8,	13,	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xD9,	2,	{0x00,0x28}},
	
	{0xE5,	2,	{0x00,0xFF}},
	{0xE6,	4,	{0xF3,0xEC,0xE7,0xDF}},
	{0xE7,	10,	{0xF3,0xD9,0xCC,0xCD,0xB3,0xA6,0x99,0x99,0x99,0x95}},
	{0xE8,	10,	{0xF3,0xD9,0xCC,0xCD,0xB3,0xA6,0x99,0x99,0x99,0x95}},
	{0xE9,	2,	{0x00,0x04}},
	{0xEA,	1,	{0x00}},
	{0xEE,	4,	{0x87,0x78,0x00,0x00}},
	{0xEF,	2,	{0x07,0xFF}},
	
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x01}},
	//{REGFLAG_DELAY, 5, {}},
	{0xB0,	2,	{0x0D,0x0D}},
	{0xB1,	2,	{0x0D,0x0D}},
	{0xB3,	2,	{0x2d,0x2d}},//2d
	{0xB4,	2,	{0x13,0x13}},//19 0f
	{0xB5,	2,	{0x06,0x06}},
	{0xB6,	2,	{0x05,0x05}},
	{0xB7,	2,	{0x05,0x05}},
	{0xB8,	2,	{0x05,0x05}},
	{0xB9,	2,	{0x44,0x44}},
	{0xBA,	2,	{0x24,0x24}},//24
	{0xBC,	2,	{0x50,0x00}},
	{0xBD,	2,	{0x50,0x00}},
	
	// ADD C4H
	{0xc4,	2,	{0x1e,0x1e}},//VGL_REG=-7V；
	
	{0x6F,	1,	{0x11}},
	{0xF3,	1,	{0x01}},
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x01}},
	{0xBE,	1,	{0x28}},//vcom default 20 28
	{0xBF,	1,	{0x20}},
	
	{0xC0,	1,	{0x0C}},
	{0xC1,	1,	{0x00}},
	{0xC2,	2,	{0x19,0x19}},
	{0xC3,	2,	{0x0A,0x0A}},
	{0xC4,	2,	{0x23,0x23}},
	{0xC7,	3,	{0x00,0x00,0x00}},
	{0xC9,	6,	{0x00,0x00,0x00,0x00,0x00,0x00}},
	{0xCA,	1,	{0x01}},
	{0xCB,	2,	{0x0B,0x53}},
	{0xCC,	1,	{0x00}},
	{0xCD,	3,	{0x0B,0x52,0x53}},
	{0xCE,	1,	{0x44}},
	{0xCF,	3,	{0x00,0x50,0x50}},
	
	{0xD0,	2,	{0x50,0x50}},
	{0xD1,	2,	{0x50,0x50}},
	{0xD2,	1,	{0x39}},
	{0xD3,	1,	{0x39}},
	
	
	//{REGFLAG_DELAY, 5, {}},
	{0xEE, 1, {0x02}},//0x01
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x02}},

	
	/////////////gamma 0.295,0.305
	
	{0xB0,	16,	{0x00,0xD2,0x00,0xD8,0x00,0xF3,0x01,0x01,0x01,0x16,0x01,0x33,0x01,0x4A,0x01,0x6C}},
	{0xB1,	16,	{0x01,0x8A,0x01,0xC0,0x01,0xEB,0x02,0x2D,0x02,0x62,0x02,0x64,0x02,0x95,0x02,0xCB}},
	{0xB2,	16,	{0x02,0xEE,0x03,0x1C,0x03,0x3C,0x03,0x61,0x03,0x7C,0x03,0xAB,0x03,0xC3,0x03,0xDB}},
	{0xB3,	4,	{0x03,0xF3,0x03,0xFF}},	
	
	
	
	{0xB4,	16,	{0x01,0x1C,0x01,0x1F,0x01,0x2F,0x01,0x37,0x01,0x45,0x01,0x5A,0x01,0x6C,0x01,0x88}},		
	{0xB5,	16,	{0x01,0xA2,0x01,0xD1,0x01,0xF7,0x02,0x35,0x02,0x69,0x02,0x6B,0x02,0x9B,0x02,0xD0}},	
	{0xB6,	16,	{0x02,0xF3,0x03,0x22,0x03,0x41,0x03,0x6D,0x03,0x87,0x03,0xB2,0x03,0xC8,0x03,0xDE}},	
	{0xB7,	4,	{0x03,0xF4,0x03,0xFF}},	
		
	{0xB8,	16,	{0x01,0x30,0x01,0x32,0x01,0x40,0x01,0x47,0x01,0x53,0x01,0x65,0x01,0x75,0x01,0x90}},		
	{0xB9,	16,	{0x01,0xA9,0x01,0xD5,0x01,0xFA,0x02,0x36,0x02,0x69,0x02,0x6B,0x02,0x9B,0x02,0xD0}},	
	{0xBA,	16,	{0x02,0xF4,0x03,0x25,0x03,0x49,0x03,0x80,0x03,0xD6,0x03,0xE4,0x03,0xEB,0x03,0xF3}},	
	{0xBB,	4,	{0x03,0xFB,0x03,0xFF}},	
	
	{0xBC,	16,	{0x00,0xD2,0x00,0xD8,0x00,0xF3,0x01,0x01,0x01,0x16,0x01,0x33,0x01,0x4A,0x01,0x6C}},		
	{0xBD,	16,	{0x01,0x8A,0x01,0xC0,0x01,0xEB,0x02,0x2D,0x02,0x62,0x02,0x64,0x02,0x95,0x02,0xCB}},	
	{0xBE,	16,	{0x02,0xEE,0x03,0x1C,0x03,0x3C,0x03,0x61,0x03,0x7C,0x03,0xAB,0x03,0xC3,0x03,0xDB}},	
	{0xBF,	4,	{0x03,0xF3,0x03,0xFF}},		
		
	{0xC0,	16,	{0x01,0x1C,0x01,0x1F,0x01,0x2F,0x01,0x37,0x01,0x45,0x01,0x5A,0x01,0x6C,0x01,0x88}},		
	{0xC1,	16,	{0x01,0xA2,0x01,0xD1,0x01,0xF7,0x02,0x35,0x02,0x69,0x02,0x6B,0x02,0x9B,0x02,0xD0}},	
	{0xC2,	16,	{0x02,0xF3,0x03,0x22,0x03,0x41,0x03,0x6D,0x03,0x87,0x03,0xB2,0x03,0xC8,0x03,0xDE}},	
	{0xC3,	4,	{0x03,0xF4,0x03,0xFF}},
	
	{0xC4,	16,	{0x01,0x30,0x01,0x32,0x01,0x40,0x01,0x47,0x01,0x53,0x01,0x65,0x01,0x75,0x01,0x90}},		
	{0xC5,	16,	{0x01,0xA9,0x01,0xD5,0x01,0xFA,0x02,0x36,0x02,0x69,0x02,0x6B,0x02,0x9B,0x02,0xD0}},	
	{0xC6,	16,	{0x02,0xF4,0x03,0x25,0x03,0x49,0x03,0x80,0x03,0xD6,0x03,0xE4,0x03,0xEB,0x03,0xF3}},	
	{0xC7,	4,	{0x03,0xFB,0x03,0xFF}},	
			
	
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x03}},
	//{REGFLAG_DELAY, 5, {}},
	{0xB0, 2, {0x00,0x00}},
	{0xB1, 2, {0x00,0x00}},
	{0xB2, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB3, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB4, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB5, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB6, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB7, 5, {0x03,0x00,0x00,0x00,0x00}},
	{0xB8, 5, {0x03,0x00,0x00,0x00,0x00}},	
	{0xB9, 5, {0x03,0x00,0x00,0x00,0x00}},	
	{0xBA, 5, {0x35,0x10,0x00,0x00,0x00}},	
	{0xBB, 5, {0x35,0x10,0x00,0x00,0x00}},
	{0xBC, 5, {0x35,0x10,0x00,0x00,0x00}},
	{0xBD, 5, {0x35,0x10,0x00,0x00,0x00}},
	{0xC0, 4, {0x00,0x34,0x00,0x00}},
	{0xC1, 4, {0x00,0x34,0x00,0x00}},
	{0xC2, 4, {0x00,0x34,0x00,0x00}},
	{0xC3, 4, {0x00,0x34,0x00,0x00}},
	{0xC4, 1, {0x40}},
	{0xC5, 1, {0x40}},
	{0xC6, 1, {0x40}},
	{0xC7, 1, {0x40}},
	{0xEF, 1, {0x00}},
	
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x05}},
	//{REGFLAG_DELAY, 5, {}},
	{0xB0, 2, {0x1B,0x10}},
	{0xB1, 2, {0x1B,0x10}},
	{0xB2, 2, {0x1B,0x10}},
	{0xB3, 2, {0x1B,0x10}},
	{0xB4, 2, {0x1B,0x10}},
	{0xB5, 2, {0x1B,0x10}},
	{0xB6, 2, {0x1B,0x10}},
	{0xB7, 2, {0x1B,0x10}},
	{0xB8, 1, {0x00}},
	{0xB9, 1, {0x00}},
	{0xBA, 1, {0x00}},
	{0xBB, 1, {0x00}},
	{0xBC, 1, {0x00}},
	{0xBD, 5, {0x03,0x03,0x03,0x00,0x01}},
	
	{0xC0, 1, {0x03}},
	{0xC1, 1, {0x05}},
	{0xC2, 1, {0x03}},
	{0xC3, 1, {0x05}},
	{0xC4, 1, {0x80}},
	{0xC5, 1, {0xA2}},
	{0xC6, 1, {0x80}},
	{0xC7, 1, {0xA2}},
	{0xC8, 2, {0x01,0x20}},
	{0xC9, 2, {0x00,0x20}},
	{0xCA, 2, {0x01,0x00}},
	{0xCB, 2, {0x00,0x00}},
	{0xCC, 3, {0x00,0x00,0x01}},
	{0xCD, 3, {0x00,0x00,0x01}},
	{0xCE, 3, {0x00,0x00,0x01}},
	{0xCF, 3, {0x00,0x00,0x01}},
	
	{0xD0, 1, {0x00}},
	{0xD1, 5, {0x03,0x00,0x00,0x07,0x10}},
	{0xD2, 5, {0x13,0x00,0x00,0x07,0x11}},
	{0xD3, 5, {0x23,0x00,0x00,0x07,0x10}},
	{0xD4, 5, {0x33,0x00,0x00,0x07,0x11}},
	{0xE5, 1, {0x06}},
	{0xE6, 1, {0x06}},
	{0xE7, 1, {0x06}},
	{0xE8, 1, {0x06}},
	{0xE9, 1, {0x06}},
	{0xEA, 1, {0x06}},
	{0xEB, 1, {0x06}},
	{0xEC, 1, {0x06}},
	{0xED, 1, {0x31}},
	
	{0xF0,	5,	{0x55,0xAA,0x52,0x08,0x06}},
	//{REGFLAG_DELAY, 5, {}},
	{0xB0, 2, {0x10,0x11}},
	{0xB1, 2, {0x12,0x13}},
	{0xB2, 2, {0x08,0x00}},
	{0xB3, 2, {0x2D,0x2D}},
	{0xB4, 2, {0x2D,0x34}},
	{0xB5, 2, {0x34,0x2D}},
	{0xB6, 2, {0x2D,0x34}},
	{0xB7, 2, {0x34,0x34}},
	{0xB8, 2, {0x02,0x0A}},
	{0xB9, 2, {0x00,0x08}},
	{0xBA, 2, {0x09,0x01}},
	{0xBB, 2, {0x0B,0x03}},
	
	{0xBC,	2,	{0x34,0x34}},
	{0xBD,	2,	{0x34,0x2D}},
	{0xBE,	2,	{0x2D,0x34}},
	{0xBF,	2,	{0x34,0x2D}},
	
	{0xC0, 2, {0x2D,0x2D}},
	{0xC1, 2, {0x01,0x09}},
	{0xC2, 2, {0x19,0x18}},
	{0xC3, 2, {0x17,0x16}},
	{0xC4, 2, {0x19,0x18}},
	{0xC5, 2, {0x17,0x16}},
	{0xC6, 2, {0x01,0x09}},
	{0xC7, 2, {0x2D,0x2D}},
	{0xC8, 2, {0x2D,0x34}},
	{0xC9, 2, {0x34,0x2D}},
	{0xCA, 2, {0x2D,0x34}},
	{0xCB, 2, {0x34,0x34}},
	{0xCC, 2,	{0x0B,0x03}},
	{0xCD, 2,	{0x09,0x01}},
	{0xCE, 2,	{0x00,0x08}},
	{0xCF, 2,	{0x02,0x0A}},
	
	{0xD0, 2,	{0x34,0x34}},
	{0xD1, 2,	{0x34,0x2D}},
	{0xD2, 2,	{0x2D,0x34}},
	{0xD3, 2,	{0x34,0x2D}},
	{0xD4, 2,	{0x2D,0x2D}},
	{0xD5, 2,	{0x08,0x00}},
	{0xD6, 2,	{0x10,0x11}},
	{0xD7, 2,	{0x12,0x13}},
	{0xD8,	5,	{0x55,0x55,0x55,0x55,0x00}},
	{0xD9,	5,	{0x55,0x55,0x55,0x55,0x00}},
	
	{0xE5, 2,	{0x34,0x34}},
	{0xE6, 2,	{0x34,0x34}},
	{0xE7, 1, {0x05}},


	{0x35, 1,	{0x00}},
	{0x44, 2,	{0x01,0xa0}},
	
	{0x11, 1, {0x00}},
  {REGFLAG_DELAY, 150, {}},//120
    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},//20
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.
	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};



static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	
	// cmd 3
	data_array[0] = 0x00053902;                          
    data_array[1] = 0xA555AAFF; 
    data_array[2] = 0x00000080;              
    dsi_set_cmdq(data_array, 3, 1); 
    
  data_array[0] = 0x00023902;        
    data_array[1] = 0x0000136F;             
    dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00023902;        
    data_array[1] = 0x000000F7;             
    dsi_set_cmdq(data_array, 2, 1);
    
    // page 00
	data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000008;              
    dsi_set_cmdq(data_array, 3, 1); 

	data_array[0] = 0x00033902;                       
    data_array[1] = 0x002168B1;                 
    dsi_set_cmdq(data_array, 2, 1);     
    	
    data_array[0] = 0x00023902;              
    data_array[1] = 0x0000C8B5;                 
    dsi_set_cmdq(data_array, 2, 1);    
    	
    data_array[0] = 0x00023902;    
    data_array[1] = 0x000012B6;                  
    dsi_set_cmdq(data_array, 2, 1);   
    	
    data_array[0] = 0x00053902;
    data_array[1] = 0x0A0000B8;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);   
    	
    data_array[0] = 0x00023902; 
    data_array[1] = 0x000000B9;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x000002BA;               
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00033902;
    data_array[1] = 0x006363BB;           
    dsi_set_cmdq(data_array, 2, 1); 
    
	
    data_array[0] = 0x00033902; 
    data_array[1] = 0x000000BC;     //00:column, 02:two dot          
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00063902;
    data_array[1] = 0x0D7F02BD; 
    data_array[2] = 0x0000000B;                
    dsi_set_cmdq(data_array, 3, 1); 
    
    data_array[0] = 0x00113902;
    data_array[1] = 0x873641CC; 
    data_array[2] = 0x10654654; 
    data_array[3] = 0x12101412; 
    data_array[4] = 0x15084014;
    data_array[5] = 0x00000005;            
    dsi_set_cmdq(data_array, 6, 1);
     
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x000000D0;                 
    dsi_set_cmdq(data_array, 2, 1);  
    	
    data_array[0] = 0x00113902;     
    data_array[1] = 0x080400D1; 
    data_array[2] = 0x1814100C; 
    data_array[3] = 0x2824201C; 
    data_array[4] = 0x3834302C;
    data_array[5] = 0x0000003C;              
    dsi_set_cmdq(data_array, 6, 1);  
    	
    data_array[0] = 0x00023902;
    data_array[1] = 0x000000D3;                 
    dsi_set_cmdq(data_array, 2, 1);  

    data_array[0] = 0x00033902; 
    data_array[1] = 0x004444D6;                 
    dsi_set_cmdq(data_array, 2, 1); 

	  data_array[0] = 0x000D3902;
    data_array[1] = 0x000000D7; 
    data_array[2] = 0x00000000; 
    data_array[3] = 0x00000000; 
    data_array[4] = 0x00000000;              
    dsi_set_cmdq(data_array, 5, 1); 
    
    data_array[0] = 0x000E3902;
    data_array[1] = 0x000000D8; 
    data_array[2] = 0x00000000; 
    data_array[3] = 0x00000000; 
    data_array[4] = 0x00000000;              
    dsi_set_cmdq(data_array, 5, 1); 
    
    	
    data_array[0] = 0x00033902;  
    data_array[1] = 0x002800D9;                 
    dsi_set_cmdq(data_array, 2, 1);
	
    data_array[0] = 0x00033902;    
    data_array[1] = 0x00FF00E5;                 
    dsi_set_cmdq(data_array, 2, 1); 
     

    data_array[0] = 0x00053902;
    data_array[1] = 0xE7ECF3E6; 
    data_array[2] = 0x000000DF;        
    dsi_set_cmdq(data_array, 3, 1);   

  	data_array[0] = 0x000B3902; 
    data_array[1] = 0xCCD9F3E7;
    data_array[2] = 0x99A6B3CD; 
    data_array[3] = 0x00959999;                   
    dsi_set_cmdq(data_array, 4, 1); 
    
    data_array[0] = 0x000B3902; 
    data_array[1] = 0xCCD9F3E8;
    data_array[2] = 0x99A6B3CD; 
    data_array[3] = 0x00959999;                   
    dsi_set_cmdq(data_array, 4, 1);
     
                
    data_array[0] = 0x00033902;                   
    data_array[1] = 0x000400E9;                   
    dsi_set_cmdq(data_array, 2, 1);  
                
    data_array[0] = 0x00023902;                   
    data_array[1] = 0x000000EA;                   
    dsi_set_cmdq(data_array, 2, 1); 
                 
    data_array[0] = 0x00053902;                   
    data_array[1] = 0x007887EE;  
    data_array[2] = 0x00000000;   
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00033902;     
    data_array[1] = 0x00FF07EF;     
    dsi_set_cmdq(data_array, 2, 1);
	
    
    #if 1
    // page 01
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000108;              
    dsi_set_cmdq(data_array, 3, 1); 
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000D0DB0;             
    dsi_set_cmdq(data_array, 2, 1); 
	
	  data_array[0] = 0x00033902;        
    data_array[1] = 0x000D0DB1;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D2DB3;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001919B4;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000606B5;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000505B6;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000505B7;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000505B8;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x004444B9;       //0x33      
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003636BA;       //0x16      
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00032902;        
    data_array[1] = 0x000050BC;             
    dsi_set_cmdq(data_array, 2, 1);
     
    data_array[0] = 0x00032902;        
    data_array[1] = 0x000050BD;             
    dsi_set_cmdq(data_array, 2, 1);
 
  // for AVDD 5v   
	data_array[0] = 0x00023902;        
    data_array[1] = 0x0000116F;             
    dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00023902;        
    data_array[1] = 0x000001F3;             
    dsi_set_cmdq(data_array, 2, 1);
	
	// page 01
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000108;              
    dsi_set_cmdq(data_array, 3, 1); 
	
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000020BE;  //29           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000020BF;  //29           
    dsi_set_cmdq(data_array, 2, 1);
    
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x00000CC0;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000C1;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001919C2;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000A0AC3;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002323C4;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x000000C7;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00073902;        
    data_array[1] = 0x000000C9;
    data_array[2] = 0x00000000;             
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000001CA;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00530BCB;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000CC;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x53520BCD;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000044CE;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x505000CF;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x005050D0;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x005050D1;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000039D2;             
    dsi_set_cmdq(data_array, 2, 1);
       
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000039D3;             
    dsi_set_cmdq(data_array, 2, 1);
    #endif
    
#if 0	 // (0.30, 0.32)gamma2.2
    //  page 02 setting gamma
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000208;              
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00B0; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501B1; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02B2; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03B3;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00B4; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501B5; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02B6; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03B7;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00B8; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501B9; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02BA; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03BB;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00BC; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501BD; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02BE; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03BF;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00C0; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501C1; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02C2; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03C3;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00AC00C4; 
    data_array[2] = 0x00D900BA;
    data_array[3] = 0x010101ED; 
    data_array[4] = 0x013A011E;
    data_array[5] = 0x00000062;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018501C5; 
    data_array[2] = 0x02E401B8;
    data_array[3] = 0x025B0227; 
    data_array[4] = 0x028C025D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03DF02C6; 
    data_array[2] = 0x032A030C;
    data_array[3] = 0x036D0351; 
    data_array[4] = 0x03A4038D;
    data_array[5] = 0x000000BE;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03CC03C7;
    data_array[2] = 0x000000CC;             
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000EE;             
    dsi_set_cmdq(data_array, 2, 1);
#endif 
	#if 1	// (0.30, 0.31) gamma2.3
    //  page 02 setting gamma
    
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000208;              
    dsi_set_cmdq(data_array, 3, 1);
    //R+
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00B000B0; 
    data_array[2] = 0x00D700C3;
    data_array[3] = 0x010201F5; 
    data_array[4] = 0x01370122;
    data_array[5] = 0x00000063;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018801B1; 
    data_array[2] = 0x02E501BD;
    data_array[3] = 0x025F0228; 
    data_array[4] = 0x02920261;
    data_array[5] = 0x000000C7;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03E802B2; 
    data_array[2] = 0x032E0312;
    data_array[3] = 0x037D034E; 
    data_array[4] = 0x03C303AC;
    data_array[5] = 0x000000DB;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F303B3;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //G+
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x012801B4; 
    data_array[2] = 0x01390130;
    data_array[3] = 0x014F0148; 
    data_array[4] = 0x01700162;
    data_array[5] = 0x00000091;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x01AD01B5; 
    data_array[2] = 0x02FB01D7;
    data_array[3] = 0x026A0237; 
    data_array[4] = 0x029B026D;
    data_array[5] = 0x000000CF;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03F002B6; 
    data_array[2] = 0x033B031D;
    data_array[3] = 0x038C0363; 
    data_array[4] = 0x03CA03B5;
    data_array[5] = 0x000000DF;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F403B7;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //B+
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x012101B8; 
    data_array[2] = 0x01320129;
    data_array[3] = 0x01480141; 
    data_array[4] = 0x016A015C;
    data_array[5] = 0x0000008C;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x01A701B9; 
    data_array[2] = 0x02F601D2;
    data_array[3] = 0x02660233; 
    data_array[4] = 0x02980269;
    data_array[5] = 0x000000CD;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03EF02BA; 
    data_array[2] = 0x0342031F;
    data_array[3] = 0x03A20381; 
    data_array[4] = 0x03D403C3;
    data_array[5] = 0x000000E5;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F603BB;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //R-
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x00B000BC; 
    data_array[2] = 0x00D700C3;
    data_array[3] = 0x010201F5; 
    data_array[4] = 0x01370122;
    data_array[5] = 0x00000063;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x018801BD; 
    data_array[2] = 0x02E501BD;
    data_array[3] = 0x025F0228; 
    data_array[4] = 0x02920261;
    data_array[5] = 0x000000C7;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03E802BE; 
    data_array[2] = 0x032E0312;
    data_array[3] = 0x037D034E; 
    data_array[4] = 0x03C303AC;
    data_array[5] = 0x000000DB;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F303BF;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //G-
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x012801C0; 
    data_array[2] = 0x01390130;
    data_array[3] = 0x014F0148; 
    data_array[4] = 0x01700162;
    data_array[5] = 0x00000091;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x01AD01C1; 
    data_array[2] = 0x02FB01D7;
    data_array[3] = 0x026A0237; 
    data_array[4] = 0x029B026D;
    data_array[5] = 0x000000CF;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03F002C2; 
    data_array[2] = 0x033B031D;
    data_array[3] = 0x038C0363; 
    data_array[4] = 0x03CA03B5;
    data_array[5] = 0x000000DF;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F403C3;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
    //B-
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x012101C4; 
    data_array[2] = 0x01320129;
    data_array[3] = 0x01480141; 
    data_array[4] = 0x016A015C;
    data_array[5] = 0x0000008C;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x01A701C5; 
    data_array[2] = 0x02F601D2;
    data_array[3] = 0x02660233; 
    data_array[4] = 0x02980269;
    data_array[5] = 0x000000CD;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00113902;                          
    data_array[1] = 0x03EF02C6; 
    data_array[2] = 0x0342031F;
    data_array[3] = 0x03A20381; 
    data_array[4] = 0x03D403C3;
    data_array[5] = 0x000000E5;            
    dsi_set_cmdq(data_array, 6, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x03F603C7;
    data_array[2] = 0x000000FF;             
    dsi_set_cmdq(data_array, 3, 1);
    
#endif 
    
    // page 03
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000308;              
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000000B0;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000000B1;             
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B2;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B3;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B4;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B5;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B6;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B7;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B8;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003B9;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x001035BA;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x001035BB;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x001035BC;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x001035BD;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x003400C0;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x003400C1;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x003400C2;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00053902;        
    data_array[1] = 0x003400C3;
    data_array[2] = 0x00000000;                
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000040C4;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000040C5;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000040C6;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000040C7;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000EF;              
    dsi_set_cmdq(data_array, 2, 1);
	
	
    // page 05
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000508;              
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB0;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB1;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB2;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB3;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB4;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB5;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB6;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00101BB7;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000B8;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000B9;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000BA;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000BB;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000BC;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x030303BD;  
    data_array[2] = 0x00000100;            
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000003C0;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000005C1;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000003C2;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000005C3;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000080C4;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x0000A2C5;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000080C6;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x0000A2C7;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002001C8;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002000C9;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000001CA;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000000CB;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x010000CC;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x010000CD;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x010000CE;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00043902;        
    data_array[1] = 0x010000CF;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000000D0;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000003D1;
    data_array[2] = 0x00001007;           
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000013D2;
    data_array[2] = 0x00001107;           
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000023D3;
    data_array[2] = 0x00001007;           
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x000033D4;
    data_array[2] = 0x00001107;           
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006E5;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006E6;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006E7;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006E8;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006E9;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006EA;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006EB;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000006EC;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00023902;        
    data_array[1] = 0x000031ED;           
    dsi_set_cmdq(data_array, 2, 1);
	    
    // page 06 begin
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000608;              
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001110B0;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001312B1;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000008B2;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D2DB3;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DB4;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34B5;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DB6;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434B7;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000A02B8;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000800B9;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000109BA;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00030BBB;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434BC;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34BD;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DBE;              
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34BF;              
    dsi_set_cmdq(data_array, 2, 1);
       
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D2DC0;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000901C1;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001819C2;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001617C3;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001819C4;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001617C5;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000901C6;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D2DC7;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DC8;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34C9;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DCA;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434CB;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00030BCC;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000109CD;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000800CE;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000A02CF;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434D0;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34D1;           
    dsi_set_cmdq(data_array, 2, 1);
   
    data_array[0] = 0x00033902;        
    data_array[1] = 0x00342DD2;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D34D3;           
    dsi_set_cmdq(data_array, 2, 1);
   
    data_array[0] = 0x00033902;        
    data_array[1] = 0x002D2DD4;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000008D5;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001110D6;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x001312D7;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x555555D8; 
    data_array[2] = 0x00000055;         
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00063902;        
    data_array[1] = 0x555555D9; 
    data_array[2] = 0x00000055;         
    dsi_set_cmdq(data_array, 3, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434E5;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x003434E6;           
    dsi_set_cmdq(data_array, 2, 1);
    
    data_array[0] = 0x00033902;        
    data_array[1] = 0x000005E7;           
    dsi_set_cmdq(data_array, 2, 1);
	
	
	
  
   #if 0
    // page 06 end
//REGW 0xF0,0x55,0xAA,0x52,0x08,0x00
//REGW 0xEF,0x07,0xFF
//REGW 0xEE,0x87,0x78,0x02,0x40
    data_array[0] = 0x00063902;                          
    data_array[1] = 0x52AA55F0; 
    data_array[2] = 0x00000008;              
    dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0] = 0x00033902;        
    data_array[1] = 0x00EF07EF;           
    dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00053902;        
    data_array[1] = 0x027887EE;   
    data_array[2] = 0x00000040;	
    dsi_set_cmdq(data_array, 3, 1);
	#endif
	
	
	data_array[0] = 0x00351500;                
    dsi_set_cmdq(data_array, 1, 1); 
   
 // {0x11, 1, {0x00}},
 // {REGFLAG_DELAY, 120, {}},
    data_array[0] = 0x00110500;                
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(120); 
    // Display ON
//	{0x29, 1, {0x00}},
//	{REGFLAG_DELAY, 20, {}},
    data_array[0] = 0x00290500;                
    dsi_set_cmdq(data_array, 1, 1); 
    MDELAY(20); 

	
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);				
       	}	
    }
	
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
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
		params->dsi.mode   = BURST_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif
	
		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE; //LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
	//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		
		// Video mode setting		
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		#if 0
		params->dsi.vertical_sync_active				= 1;// 3    2
		params->dsi.vertical_backporch					= 1;// 20   1
		params->dsi.vertical_frontporch					= 2; //2 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 2;// 50  2
		params->dsi.horizontal_backporch				= 12;
		params->dsi.horizontal_frontporch				= 80;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
        #endif
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
		params->dsi.packet_size=256;
params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
params->dsi.vertical_active_line = FRAME_HEIGHT;
params->dsi.intermediat_buffer_num = 2;

params->dsi.word_count=720*3;
params->dsi.vertical_sync_active = 2;//2;
params->dsi.vertical_backporch = 10;//8;//10
params->dsi.vertical_frontporch = 20;//7;
params->dsi.vertical_active_line = FRAME_HEIGHT; 
params->dsi.horizontal_sync_active = 50;//60;
params->dsi.horizontal_backporch = 12;//109;
params->dsi.horizontal_frontporch = 80;//129; 
params->dsi.horizontal_active_pixel = FRAME_WIDTH;
	//    params->dsi.LPX=3; 
//params->dsi.PLL_CLOCK = LCM_DSI_6589_PLL_CLOCK_234;//LCM_DSI_6589_PLL_CLOCK_227_5;//this value must be in MTK suggested table

#if 0
		// Bit rate calculation
		//1 Every lane speed
		params->dsi.pll_select=1;
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
#if (LCM_DSI_CMD_MODE)
		params->dsi.fbk_div =19;
#else
		params->dsi.fbk_div =16; //20;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
#endif
#endif

        params->dsi.PLL_CLOCK=246;//279.5;
		params->dsi.ssc_range =0;
        params->dsi.ssc_disable = 1; //0
		//params->dsi.compatibility_for_nvk = 1;		// this parameter would be set to 1 if DriverIC is NTK's and when force match DSI clock for NTK's
		
		params->dsi.HS_TRAIL=2;  //（THS-TRAIL要求>60ns+4UI即可，一般75ns左右）
}

static void lcm_dcdc_control(int state)
{
   if (ON == state)
   {
	   #if 1
	    mt_set_gpio_mode(GPIO_LCM_DCDC_ENP_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENP_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENP_PIN,GPIO_OUT_ONE);
	    #else
	    
	    mt_set_gpio_mode(GPIO_LCM_DCDC_ENP_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENP_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENP_PIN,GPIO_OUT_ONE);
		
		mt_set_gpio_mode(GPIO_LCM_DCDC_ENN_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENN_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENN_PIN,GPIO_OUT_ONE);
	    #endif
		
    }
   else if (OFF == state)
   {
   	#if 1
        mt_set_gpio_mode(GPIO_LCM_DCDC_ENP_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENP_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENP_PIN,GPIO_OUT_ZERO);
	    
	    #else
	    
	    mt_set_gpio_mode(GPIO_LCM_DCDC_ENP_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENP_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENP_PIN,GPIO_OUT_ZERO);
   
        mt_set_gpio_mode(GPIO_LCM_DCDC_ENN_PIN,GPIO_MODE_GPIO);
	    mt_set_gpio_dir(GPIO_LCM_DCDC_ENN_PIN,GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_DCDC_ENN_PIN,GPIO_OUT_ZERO);
	    #endif
    }

}

static void lcm_init(void)
{	

   printk("%s: try NT35520 JDI init\n", __func__);

	lcm_dcdc_control(ON);
	MDELAY(10);   
	printk("%s: try NT35520 cys lcm_dcdc_control(ON);\n", __func__);
	mt_set_gpio_mode(GPIO_LCM_RST_PIN,GPIO_MODE_GPIO);	//add by sam 2013-12-05
	mt_set_gpio_dir(GPIO_LCM_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ONE);
	MDELAY(5); 
	printk("%s: try NT35520 cys reset = 1 start;\n", __func__);
    /*	
	SET_RESET_PIN(1);
	MDELAY(5);    
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);*/
	
	//mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_GPIO);
	//mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_LCM_ID1_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
   // mt_set_gpio_pull_select(76, GPIO_PULL_DOWN);

	mt_set_gpio_mode(GPIO_LCM_IDO_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_IDO_PIN, GPIO_DIR_IN);  
	mt_set_gpio_pull_enable(GPIO_LCM_IDO_PIN, GPIO_PULL_DISABLE);
	
	
	mt_set_gpio_mode(GPIO_LCM_RST_PIN,GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_RST_PIN,GPIO_DIR_OUT);

	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ZERO);
	MDELAY(10);
	printk("%s: try NT35520 cys reset = 0;\n", __func__);
	mt_set_gpio_out(GPIO_LCM_RST_PIN, GPIO_OUT_ONE);
	MDELAY(30);
	printk("%s: try NT35520 cys reset = 1 end;\n", __func__);
	
		//init_lcm_registers();	    
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

     #ifdef BUILD_LK
	  printf("lcm init finish*******\n");	
	 #endif
}



static void lcm_suspend(void)
{
printk("%s: try NT35520 suspend\n", __func__);
  //  lcm_compare_id();
    
	
    lcm_dcdc_control(OFF);
		
	//mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_GPIO);
	//mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(GPIO_LCM_ID1_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
   // mt_set_gpio_pull_select(76, GPIO_PULL_DOWN);

	mt_set_gpio_mode(GPIO_LCM_IDO_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_IDO_PIN, GPIO_DIR_IN);  
	mt_set_gpio_pull_enable(GPIO_LCM_IDO_PIN, GPIO_PULL_DISABLE);
	
	MDELAY(10);
	
	mt_set_gpio_mode(GPIO_LCM_RST_PIN,GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ZERO);

	
	
	//MDELAY(10); 
	//mt_set_gpio_out(131,GPIO_OUT_ONE);
	
	
}


static void lcm_resume(void)
{ 
   //printk("lcm 2014 0121 2031\n");	
  //  lcm_compare_id();
 printk("%s: try NT35520 resume enter lcm init\n", __func__);
	lcm_init();
	//MDELAY(10);
	printk("%s: try NT35520 resume leave lcm init\n", __func__);
 
}
         
#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

	
	
	
}
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int data_array[16];  
	unsigned int id1;
	unsigned int id2;
	unsigned int id3;
	unsigned int id4;
	unsigned int dbit, bits;  

	
	dbit = 1;
	bits = 0;       
	mt_set_gpio_mode(GPIO_LCM_IDO_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_IDO_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCM_IDO_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
	//mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_GPIO);
	//mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);  
	//mt_set_gpio_pull_enable(GPIO_LCM_ID1_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
	//dbit = mt_get_gpio_in(GPIO_LCM_IDO_PIN);
	//bits |= dbit;
	//bits <<= 1;
	dbit = mt_get_gpio_in(GPIO_LCM_IDO_PIN);
	//bits |= dbit;
	printk("[LCM]TRY JDI %s: [bits = %d] \n", __func__, dbit);

	
#if 1
	if(LCM_HW_TRY_ID == dbit)
        {
        lcm_dcdc_control(ON);
	MDELAY(20);  
        mt_set_gpio_mode(GPIO_LCM_RST_PIN,GPIO_MODE_GPIO);	//add by sam 2013-12-05
	mt_set_gpio_dir(GPIO_LCM_RST_PIN,GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ONE);
	MDELAY(10); 
	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ZERO);
	MDELAY(10); 
	mt_set_gpio_out(GPIO_LCM_RST_PIN,GPIO_OUT_ONE);
	MDELAY(30); 

        data_array[0] = 0x00063902;                          
        data_array[1] = 0x52AA55F0; 
        data_array[2] = 0x00000108;
        dsi_set_cmdq(data_array, 3, 1);  

        data_array[0] = 0x00013700;// read id return two byte,version and id
        dsi_set_cmdq(data_array, 1, 1);
        MDELAY(10); 
                    
        read_reg_v2(0xda, buffer, 1);
        id1 = buffer[0]; //we only need ID
        printk("%s: LCM_HW_CURRENT_ID id1 try: %#x\n", __func__, id1);
        
        read_reg_v2(0xda, buffer, 1);
        id2 = buffer[0]; //we only need ID
        printk("%s: LCM_HW_CURRENT_ID: id2 try %#x\n", __func__, id2);
        
        read_reg_v2(0xda, buffer, 1);
        id3 = buffer[0]; //we only need ID
        printk("%s: LCM_HW_CURRENT_ID: id3 try %#x\n", __func__, id3);
        
        read_reg_v2(0xda, buffer, 1);
        id4 = buffer[0]; //we only need ID
        printk("%s: LCM_HW_CURRENT_ID: id4 try %#x\n", __func__, id4);
        
       
        if((id1==0x52)&(id2==0x52)&(id3==0x52)&(id4==0x52))
      // if(id1==0x52)
             	return 0;
        else
                return 1;   
        }
        else
        {
        	printk("%s: can not find the lcm driver of [try_nt35590_720p_JDI_5p0_cmd_lcm_drv] \n", __func__);
        	printk("%s: LCM_HW_CURRENT_ID: %#x\n", __func__, bits);
            	return 0;
	}
#endif

}


static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x36, buffer, 1);
	if(buffer[0]==0x90)
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();
	//lcm_resume();

	return TRUE;
}
/*
static unsigned int get_lcm_hw_id(void)
{
   	unsigned int dbit, bits;  
	printk("%s: \n", __func__);
		
	dbit = 1;
	bits = 0;       
	mt_set_gpio_mode(GPIO_LCM_IDO_PIN, GPIO_MODE_GPIO);
	mt_set_gpio_dir(GPIO_LCM_IDO_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_LCM_IDO_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
	//mt_set_gpio_mode(GPIO_LCM_ID1_PIN, GPIO_MODE_GPIO);
	//mt_set_gpio_dir(GPIO_LCM_ID1_PIN, GPIO_DIR_IN);  
	//mt_set_gpio_pull_enable(GPIO_LCM_ID1_PIN, GPIO_PULL_DISABLE); //DISABLE, ENABLE
	//dbit = mt_get_gpio_in(GPIO_LCM_ID1_PIN);
	//bits |= dbit;
	//bits <<= 1;
	dbit = mt_get_gpio_in(GPIO_LCM_IDO_PIN);
	//bits |= dbit;
	printk("[LCM]HW id %s: [bits = %d] \n", __func__, dbit);
//return LCM_HW_BYD_ID;
	if(LCM_HW_TRY_ID == dbit)
    {
       
        return LCM_HW_TRY_ID;
    }
    else
    {
        return LCM_HW_BYD_ID;
	}

}
*/
/*static unsigned int lcm_get_id(unsigned char* lcm_flag)
{
	unsigned char *str0 = "JDI";
//	unsigned int id = 0;
//	id = get_lcm_hw_id();
//	if (LCM_HW_TRY_ID == id)
//	    *str0 = "TRY";
//	else
//	    *str0 = "BYD";
	strcpy(lcm_flag, str0);	
	
   // return id;
	return LCM_HW_JDI_ID;
}*/
static unsigned int lcm_get_id(unsigned char* lcm_flag)
{
	//unsigned char *str0 = "TRY";
	//unsigned int id = 0;
	//id = get_lcm_hw_id();
	//if (LCM_HW_TRY_ID == id)
	//    *str0 = "TRY";
	//else
	//    *str0 = "BYD";
	//strcpy(lcm_flag, str0);	
	
   // return id;
	return LCM_HW_TRY_ID;
}

LCM_DRIVER try_nt35520_720p_jdi_5p0_cmd_lcm_drv = 
{
    .name			= "try_nt35520_720p_jdi_5p0_cmd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
//	.esd_check = lcm_esd_check,
//	.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .get_id		= lcm_get_id,
    };
