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

#ifndef _CNM_DEC_H_
#define _CNM_DEC_H_

void *cnm_dec_init(void);

enum vcore_dec_ret cnm_dec_open(void **vcore_id,
		void* cnm_core_instance,
		void* cnm_dec_instance,
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
								struct vcore_dec_report *vcore_report));

enum vcore_dec_ret cnm_dec_close(void *vcore_id);

enum vcore_dec_ret cnm_dec_register_dpb(void *vcore_id,
						struct vcore_dec_fb *fbinfo);
enum vcore_dec_ret cnm_dec_clear_dpb(void *vcore_id, unsigned int dpb_addr);
enum vcore_dec_ret cnm_dec_update_buffer(void *vcore_id,
						struct vcore_dec_au *au, vcore_bool_t *running);

enum vcore_dec_ret cnm_dec_flush(void *vcore_id, unsigned int rd_addr);

void cnm_dec_reset(void *vcore_id);
void cnm_dec_report_reset(void *_cnm_dec_instance);
void cnm_dec_broadcast(void *cnm_core_instance, void *_cnm_dec_instance, void *last_vcore_id);

#endif /* #ifndef _CNM_DEC_H_ */

