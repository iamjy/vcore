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

#include <media/odin/vcodec/vcore/encoder.h>

#include "vcore/cnm_9x0/coda980/coda980.h"

#include "vcore/cnm_9x0/cnm/cnm_enc.h"

static void *coda980_cnm_enc_instance = NULL;

static enum vcore_enc_ret _coda980_enc_open(void **vcore_id,
		struct vcore_enc_config *config,
		unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
		unsigned int workbuf_size,
		void *vec_id,
		void (*vcore_enc_report)(void *vec_id,
								struct vcore_enc_report *vcore_report))
{
	void *cnm_core_instance = coda980_get_cnm_core_instance();
	return cnm_enc_open(vcore_id,
			cnm_core_instance,
			coda980_cnm_enc_instance,
			config,
			workbuf_paddr, workbuf_vaddr,
			workbuf_size,
			vec_id, vcore_enc_report);
}

void coda980_enc_init(struct vcore_enc_ops *ops)
{
	ops->open = _coda980_enc_open;
	ops->close = cnm_enc_close;
	ops->reset = cnm_enc_reset;
	ops->register_dpb = cnm_enc_register_dpb;
	ops->update_buffer = cnm_enc_update_buffer;
	ops->update_epb_rdaddr = cnm_enc_update_epb_rdaddr;
	ops->set_config = cnm_enc_set_config;

	if (coda980_cnm_enc_instance == NULL)
		coda980_cnm_enc_instance = cnm_enc_init();
}

void coda980_enc_report_reset(void)
{
	cnm_enc_report_reset(coda980_cnm_enc_instance);
}

void coda980_enc_broadcast(void *cnm_id, void *vcore_id)
{
	if (coda980_cnm_enc_instance)
		cnm_enc_broadcast(cnm_id, coda980_cnm_enc_instance, vcore_id);
}
