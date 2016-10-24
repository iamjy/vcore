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

#ifndef _CNM_ENC_H_
#define _CNM_ENC_H_

void *cnm_enc_init(void);

enum vcore_enc_ret cnm_enc_open(void **vcore_id, void *cnm_core_instance,
				void *cnm_enc_instance, struct vcore_enc_config *enc_config,
				unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
				unsigned int workbuf_size, void *vec_id,
				void (*vcore_enc_report)(void *vec_id,
					struct vcore_enc_report *vcore_report));
enum vcore_enc_ret cnm_enc_close(void *vcore_id);

enum vcore_enc_ret cnm_enc_register_dpb(void *vcore_id,
									struct vcore_enc_dpb *fbinfo);
enum vcore_enc_ret cnm_enc_update_buffer(void *vcore_id,
									struct vcore_enc_fb *fb);
enum vcore_enc_ret cnm_enc_update_epb_rdaddr(void *vcore_id,
									unsigned int rd_addr);
enum vcore_enc_ret cnm_enc_set_config(void *vcore_id,
									struct vcore_enc_running_config *config);

void cnm_enc_reset(void *vcore_id);
void cnm_enc_report_reset(void *_cnm_enc_instance);
void cnm_enc_broadcast(void *cnm_core_instance, void *_cnm_enc_instance, void *last_vcore_id);

#endif /* #ifndef _CNM_ENC_H_ */

