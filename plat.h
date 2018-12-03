/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#ifndef __PLAT_H
#define __PLAT_H

/* A platform currently supported by driver */
extern char *platform ;

#ifdef ENABLE_REGULATOR_POWER_ON
extern void regulator_power_reg(struct ilitek_platform_data *ipd);
#endif
extern int ilitek_gpio_to_irq(void);
extern int ilitek_platform_input_init(void);
extern void core_fr_input_set_param(struct input_dev *input_device);
extern int ilitek_platform_reg_suspend(void);
extern int ilitek_platform_tp_hw_reset(bool isEnable);
extern void ilitek_get_device_gpio(void);
extern int prob_compelet_status_set(void);

#endif /* __PLAT_H */
