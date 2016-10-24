/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 * Seokhoon.Kang <m4seokhoon.kang@lgepartner.com>
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

#include <media/odin/vcodec/vcore/ppu.h>

#include "vcore/cnm_9x0/boda950/boda950.h"

#include "vcore/cnm_9x0/cnm/cnm_ppu.h"

static void *boda950_cnm_ppu_instance = NULL;

enum vcore_ppu_ret _boda950_ppu_open(void **vcore_id,
				unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
				unsigned int workbuf_size,
				unsigned int width, unsigned int height,
				unsigned int fr_residual, unsigned int fr_divider,
				void *vppu_id,
				void (*vcore_ppu_report)(void *vppu_id,
									struct vcore_ppu_report *vcore_report))
{
	void *boda950_instance = boda950_get_cnm_core_instance();

	return cnm_ppu_open(vcore_id,
						boda950_instance, boda950_cnm_ppu_instance,
						workbuf_paddr, workbuf_vaddr, workbuf_size,
						width, height,
						fr_residual, fr_divider,
						vppu_id, vcore_ppu_report);
}

void boda950_ppu_init(struct vcore_ppu_ops *ops)
{
	ops->open = _boda950_ppu_open;
	ops->close = cnm_ppu_close;
	ops->reset = cnm_ppu_reset;
	ops->rotate = cnm_ppu_rotate;

	boda950_cnm_ppu_instance = cnm_ppu_init();
}

void boda950_ppu_report_reset(void)
{
	cnm_ppu_report_reset(boda950_cnm_ppu_instance);
}

void boda950_ppu_broadcast(void *cnm_id, void *vcore_id)
{
	if (boda950_cnm_ppu_instance)
		cnm_ppu_broadcast(cnm_id, boda950_cnm_ppu_instance, vcore_id);
}
