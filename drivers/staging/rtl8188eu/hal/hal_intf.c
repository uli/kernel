/******************************************************************************
 *
 * Copyright(c) 2007 - 2012 Realtek Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 ******************************************************************************/

#define _HAL_INTF_C_
#include <osdep_service.h>
#include <drv_types.h>
#include <hal_intf.h>
#include <usb_hal.h>

uint	 rtw_hal_init(struct adapter *adapt)
{
	uint	status = _SUCCESS;

	adapt->hw_init_completed = false;

	status = rtl8188eu_hal_init(adapt);

	if (status == _SUCCESS) {
		adapt->hw_init_completed = true;

		if (adapt->registrypriv.notch_filter == 1)
			rtw_hal_notch_filter(adapt, 1);
	} else {
		adapt->hw_init_completed = false;
		DBG_88E("rtw_hal_init: hal__init fail\n");
	}

	RT_TRACE(_module_hal_init_c_, _drv_err_,
		 ("-rtl871x_hal_init:status=0x%x\n", status));

	return status;
}

uint rtw_hal_deinit(struct adapter *adapt)
{
	uint	status = _SUCCESS;

	status = rtl8188eu_hal_deinit(adapt);

	if (status == _SUCCESS)
		adapt->hw_init_completed = false;
	else
		DBG_88E("\n rtw_hal_deinit: hal_init fail\n");

	return status;
}

void rtw_hal_update_ra_mask(struct adapter *adapt, u32 mac_id, u8 rssi_level)
{
	struct mlme_priv *pmlmepriv = &(adapt->mlmepriv);

	if (check_fwstate(pmlmepriv, WIFI_AP_STATE) == true) {
#ifdef CONFIG_88EU_AP_MODE
		struct sta_info *psta = NULL;
		struct sta_priv *pstapriv = &adapt->stapriv;

		if ((mac_id-1) > 0)
			psta = pstapriv->sta_aid[(mac_id-1) - 1];
		if (psta)
			add_RATid(adapt, psta, 0);/* todo: based on rssi_level*/
#endif
	} else {
		UpdateHalRAMask8188EUsb(adapt, mac_id, rssi_level);
	}
}
