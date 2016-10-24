
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/slab.h>

#include <media/odin/vcodec/vcore/decoder.h>
#include <media/odin/vcodec/vlog.h>
#include "odin_mpng.h"
#include "odin_mpng_dec.h"
#include "mpng.h"

static void odin_mpng_dec_isr(void *vcore_id, unsigned long reason);

static spinlock_t mpng_lock;
static struct list_head mpng_channel_list;

struct png_dec_cpb
{
	unsigned int phy_addr;
	unsigned char *vir_ptr;
	unsigned int size;
};

struct mpng_channel
{
	void *vdc_id;
	struct list_head list;

	struct mpng_ctx_t png;
	struct png_dec_cpb cpb;
	struct vcore_dec_au au;

	/* CBs */
	void (*cb_vcore_dec_report)(void *vdc_id,
								struct vcore_dec_report *vcore_report);
};

static void __odin_png_dec_report_seq(struct mpng_channel *ch,
								struct mpng_decode_conf_t *decode_param)
{
	struct vcore_dec_report dec_report = {0};
	unsigned long flags;

	mpng_spin_lock(&mpng_lock, &flags);

	dec_report.hdr = VCORE_DEC_DONE;
	dec_report.info.done.vcore_complete = VCORE_TRUE;
	dec_report.info.done.need_more_au = VCORE_TRUE;
	dec_report.info.done.hdr = VCORE_DEC_REPORT_SEQ;

	dec_report.info.done.info.seq.success = VCORE_TRUE;
	dec_report.info.done.info.seq.pic_width = decode_param->img_hdr.width;
	dec_report.info.done.info.seq.pic_height = decode_param->img_hdr.height;
	dec_report.info.done.info.seq.buf_width = decode_param->img_hdr.width;
	dec_report.info.done.info.seq.buf_height = decode_param->img_hdr.height;
	dec_report.info.done.info.seq.ref_frame_cnt = 1;
#ifdef VCORE_WTL_ENABLE
	dec_report.info.done.info.seq.tiled_buf_size = 0;
#endif
/*	dec_report.info.done.info.seq.format = decode_param->img_hdr.color_type; */
	dec_report.info.done.info.seq.profile = decode_param->img_hdr.color_type;
	dec_report.info.done.info.seq.level = decode_param->img_hdr.bit_depth;
	memcpy(&dec_report.info.done.info.seq.meta, &ch->au.meta,
										sizeof(struct vcore_dec_au_meta));

	odin_mpng_ch_unlock(ch);
	mpng_spin_unlock(&mpng_lock, &flags);
	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
}

enum vcore_dec_ret _odin_mpng_dec_open(void **vcore_id,
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
	unsigned long flags;
	int lock;
	struct mpng_channel* ch;

	if (codec_type != VCORE_DEC_PNG)
		return VCORE_DEC_FAIL;

	*vcore_id = NULL;

	ch = (struct mpng_channel *)kzalloc(sizeof(struct mpng_channel), GFP_ATOMIC);
	if (!ch) {
		vlog_error("mpng kzalloc failed \n");
		return VCORE_DEC_FAIL;
	}

	mpng_spin_lock(&mpng_lock, &flags);
	lock = odin_mpng_ch_is_locked(NULL);
	if (lock != 0) {
		mpng_spin_unlock(&mpng_lock, &flags);
		kfree(ch);

		return VCORE_DEC_RETRY;
	}

	memset (&ch->png, 0x0, sizeof(struct mpng_ctx_t));

	ch->png.conf.bb_iova = cpb_phy_addr;
	ch->png.conf.bb_vir_ptr = cpb_vir_ptr;
	ch->png.conf.bb_length = cpb_size;

	ch->vdc_id = vdc_id;
	ch->cb_vcore_dec_report = vcore_dec_report;
	ch->cpb.phy_addr = cpb_phy_addr;
	ch->cpb.vir_ptr = cpb_vir_ptr;
	ch->cpb.size = cpb_size;

	list_add_tail(&ch->list, &mpng_channel_list);

	mpng_spin_unlock(&mpng_lock, &flags);

	*vcore_id = (void *)ch;

	return VCORE_DEC_SUCCESS;
}

enum vcore_dec_ret _odin_mpng_dec_close(void *vcore_id)
{
	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	unsigned long flags;

	mpng_spin_lock(&mpng_lock, &flags);
	list_del(&ch->list);
	mpng_spin_unlock(&mpng_lock, &flags);

	kfree(ch);

	return VCORE_DEC_SUCCESS;
}

enum vcore_dec_ret __odin_mpng_dec_clear_dpb(void *vcore_id, unsigned int dpb_addr)
{
/*	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	unsigned int clear_dpb_reg;
*/

	vlog_info("dpb_addr 0x%x \n", dpb_addr);

	return VCORE_DEC_SUCCESS;
}

enum vcore_dec_ret _odin_mpng_dec_register_dpb(void *vcore_id,
											struct vcore_dec_fb *fbinfo)
{
	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	unsigned long flags;

	mpng_spin_lock(&mpng_lock, &flags);

	ch->png.conf.fb_iova = fbinfo->linear_dpb_addr[(fbinfo->num - 1)];

	mpng_spin_unlock(&mpng_lock, &flags);

	return VCORE_DEC_SUCCESS;
}

static enum vcore_dec_ret _odin_mpng_dec_update_buffer(void *vcore_id,
								struct vcore_dec_au *au, vcore_bool_t *running)
{
	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	struct stream_t img_stream;
	struct stream_t bit_stream;
	unsigned long flags;
	int ret;
	int lock;

	char* chunk_data;
	int remain = 0, chunk_length = 0;
	unsigned int chunk_crc = 0;
	mpng_chunk_type_e chunk_type = PNG_CHUNK_UNKNOWN;

	mpng_spin_lock(&mpng_lock, &flags);
	odin_mpng_clock_on();
	lock = odin_mpng_ch_is_locked(ch);
	if (lock != 0) {
		mpng_spin_unlock(&mpng_lock, &flags);
		return VCORE_DEC_RETRY;
	}

	odin_mpng_ch_lock(ch, odin_mpng_dec_isr);

	if (au->buf.size > 0) {
		memcpy(&ch->au, au, sizeof(struct vcore_dec_au));
		mpng_mem_stream_init(&img_stream,
								au->buf.start_vir_ptr, (int)au->buf.size);
		if (mpng_signature_check(&img_stream) < 0) {
			vlog_error("PNG Signature check failed \n");
			return VCORE_DEC_FAIL;
		}

		mpng_mem_stream_init(&bit_stream,
								au->buf.start_vir_ptr, (int)au->buf.size);
	}

	switch (au->buf.au_type) {
	case VCORE_DEC_AU_SEQUENCE :
	{
		while (chunk_type != PNG_CHUNK_IEND) {
			remain = mpng_mem_stream_get_remained_length(&img_stream);
			chunk_length = (int)mpng_mem_stream_read_4byte(&img_stream);
			if ((chunk_length < 0) || ((remain - 12) < chunk_length)) {
			/* broken PNG image */
				vlog_error("Broken image remain : %d chunk : %d \n",
										remain, chunk_length);
				chunk_type = PNG_CHUNK_UNKNOWN;
				chunk_data = NULL;
				chunk_crc = 0x0;
			} else {
				chunk_type = mpng_chunk_type_get(
								mpng_mem_stream_read_string(&img_stream, 4));
				chunk_data = (char *)mpng_mem_stream_read_string(
								&img_stream, chunk_length);
				chunk_crc = mpng_mem_stream_read_4byte(&img_stream);
			}

			if (chunk_type == PNG_CHUNK_UNKNOWN)
				continue;

			switch (chunk_type) {
			case PNG_CHUNK_IHDR :
				vlog_print(VLOG_VCORE_PNG, "IDHR ->> \n");
				ret = mpng_do_ihdr(chunk_data, chunk_length,
												&ch->png.conf, &bit_stream);
				break;

			case PNG_CHUNK_IDAT :
				/* 	vlog_print(VLOG_VCORE_PNG, "IDAT ->> \n"); */
				ret = mpng_do_idat(chunk_data, chunk_length,
												&ch->png.conf, &bit_stream);
				break;

			case PNG_CHUNK_IEND :
				vlog_print(VLOG_VCORE_PNG, "IEND ->> \n");
				mpng_do_iend(chunk_data, chunk_length,
												&ch->png.conf, &bit_stream);
				break;

			default :
				vlog_error("Broken image remain ~~~~!!!! \n");
				goto update_buffer_err;
				break;
			}
		}
		mpng_spin_unlock(&mpng_lock, &flags);
		__odin_png_dec_report_seq(ch, &ch->png.conf);
	}
		break;
	case VCORE_DEC_AU_PICTURE :
	{
		vlog_info("mpng_decode \n");
		mpng_decode(&ch->png.conf, ch->png.conf.bb_length);
		mpng_spin_unlock(&mpng_lock, &flags);
	}
		break;
	default :
		vlog_error("au type %d\n", au->buf.au_type);
		mpng_spin_unlock(&mpng_lock, &flags);
		break;
	}

	au->meta.rotation_angle = VCORE_DEC_ROTATE_0;

	return VCORE_DEC_SUCCESS;

update_buffer_err:
	odin_mpng_ch_unlock(ch);
	mpng_spin_unlock(&mpng_lock, &flags);
	return VCORE_DEC_FAIL;

}

void _odin_mpng_dec_reset(void *vcore_id)
{
/* 	struct mpng_channel *ch = (struct mpng_channel *)vcore_id; */

	vlog_error("_lge_mpng_dec_reset \n");
}

enum vcore_dec_ret _odin_mpng_dec_flush(void *vcore_id, unsigned int rd_addr)
{
	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	unsigned long flags;
	struct vcore_dec_report dec_report = {0};

	mpng_spin_lock(&mpng_lock, &flags);
	odin_mpng_ch_unlock(ch);

	dec_report.hdr = VCORE_DEC_DONE;
	dec_report.info.done.vcore_complete = VCORE_TRUE;
	dec_report.info.done.need_more_au = VCORE_TRUE;
	dec_report.info.done.hdr = VCORE_DEC_REPORT_FLUSH_DONE;
	mpng_spin_unlock(&mpng_lock, &flags);

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);

	return VCORE_DEC_SUCCESS;
}



void odin_mpng_dec_init(struct vcore_dec_ops *ops)
{
	ops->open = _odin_mpng_dec_open;
	ops->close = _odin_mpng_dec_close;
	ops->register_dpb = _odin_mpng_dec_register_dpb;
	ops->clear_dpb = __odin_mpng_dec_clear_dpb;
	ops->update_buffer = _odin_mpng_dec_update_buffer;
	ops->reset = _odin_mpng_dec_reset;
	ops->flush = _odin_mpng_dec_flush;

	mpng_spin_lock_init(&mpng_lock);
	INIT_LIST_HEAD(&mpng_channel_list);
}

static vcore_bool_t _odin_mpng_dec_report_pic(struct mpng_channel *ch,
										struct vcore_dec_report *dec_report)
{
	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_TRUE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_PIC;

	dec_report->info.done.info.pic.success = VCORE_TRUE;
	dec_report->info.done.info.pic.pic_width = ch->png.conf.img_hdr.width;
	dec_report->info.done.info.pic.pic_height = ch->png.conf.img_hdr.height;
	dec_report->info.done.info.pic.dpb_addr =
										(unsigned int)ch->png.conf.fb_iova;
	memcpy(&dec_report->info.done.info.pic.meta,
							&ch->au.meta, sizeof(struct vcore_dec_au_meta));

	odin_mpng_ch_unlock(ch);

	return VCORE_TRUE;
}

static void odin_mpng_dec_isr(void *vcore_id, unsigned long reason)
{
	struct mpng_channel *ch = (struct mpng_channel *)vcore_id;
	struct vcore_dec_report dec_report = {0};
	unsigned long flags;
	vcore_bool_t work_done = VCORE_FALSE;

	vlog_info("\n");

	if (vcore_id == NULL) {
		vlog_error("no locked channel, reason: %x\n", (unsigned int)reason);
		return;
	}

	mpng_spin_lock(&mpng_lock, &flags);

	if (reason & INT_INFLATE_DONE_EN) {
		work_done = _odin_mpng_dec_report_pic(ch, &dec_report);
	}
	else {
		vlog_error("no handle of unknown\n");
		mpng_irq_clear();
	}

	mpng_spin_unlock(&mpng_lock, &flags);
	if (work_done == VCORE_TRUE)
		odin_mpng_clock_off();

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
}


