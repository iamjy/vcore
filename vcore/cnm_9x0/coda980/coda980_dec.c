/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>

#include <media/odin/vcodec/vcore/decoder.h>

#include "vcore/cnm_9x0/coda980/coda980.h"

#include "vcore/cnm_9x0/cnm/cnm_dec.h"

static void *coda980_cnm_dec_instance = NULL;

enum vcore_dec_ret _coda980_dec_open(void **vcore_id,
			enum vcore_dec_codec codec_type,
			unsigned int cpb_phy_addr, unsigned char *cpb_vir_ptr,
			unsigned int cpb_size,
			unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
			unsigned int workbuf_size,
			vcore_bool_t reordering,
			vcore_bool_t rendering_dpb,
			vcore_bool_t secure_buf,
			void *vdc_id,
			void (*vcore_dec_report)(void *vdc_id,
					struct vcore_dec_report *vcore_report))
{
	void *cnm_id = coda980_get_cnm_core_instance();

	return cnm_dec_open(vcore_id,
						cnm_id, coda980_cnm_dec_instance,
						codec_type,
						cpb_phy_addr, cpb_vir_ptr,
						cpb_size,
						workbuf_paddr, workbuf_vaddr,
						workbuf_size,
						reordering,
						rendering_dpb,
						secure_buf,
						vdc_id,
						vcore_dec_report);
}

void coda980_dec_init(struct vcore_dec_ops *ops)
{
	ops->open = _coda980_dec_open;
	ops->close = cnm_dec_close;
	ops->register_dpb = cnm_dec_register_dpb;
	ops->clear_dpb = cnm_dec_clear_dpb;
	ops->update_buffer = cnm_dec_update_buffer;
	ops->reset = cnm_dec_reset;
	ops->flush = cnm_dec_flush;

	if (coda980_cnm_dec_instance == NULL)
		coda980_cnm_dec_instance = cnm_dec_init();
}

void coda980_dec_report_reset(void)
{
	cnm_dec_report_reset(coda980_cnm_dec_instance);
}

void coda980_dec_broadcast(void *cnm_id, void *vcore_id)
{
	if (coda980_cnm_dec_instance)
		cnm_dec_broadcast(cnm_id, coda980_cnm_dec_instance, vcore_id);
}
