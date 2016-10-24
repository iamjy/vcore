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

#ifndef _CNM_PPU_H_
#define _CNM_PPU_H_

void *cnm_ppu_init(void);

enum vcore_ppu_ret cnm_ppu_open(void **vcore_id,
				void *cnm_core_instance, void *cnm_ppu_instance,
				unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
				unsigned int workbuf_size,
				unsigned int width, unsigned int height,
				unsigned int fr_residual, unsigned int fr_divider,
				void *vppu_id,
				void (*vcore_ppu_report)(void *vppu_id,
									struct vcore_ppu_report *vcore_report));
enum vcore_ppu_ret cnm_ppu_close(void *vcore_id);

enum vcore_ppu_ret cnm_ppu_rotate(void *vcore_id,
							unsigned int src_addr, unsigned int dst_addr,
							unsigned int width, unsigned int height,
							int angle, enum vcore_ppu_image_format foramt);

void cnm_ppu_reset(void *vcore_id);
void cnm_ppu_report_reset(void *_cnm_ppu_instance);
void cnm_ppu_broadcast(void *cnm_core_instance, void *_cnm_dec_instance, void *last_vcore_id);

#endif /* #ifndef _CNM_PPU_H_ */

