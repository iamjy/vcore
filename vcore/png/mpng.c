

#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <media/odin/vcodec/vlog.h>

#include "mpng.h"

static unsigned long s_mpng_reg_base;

#define MPNG_TIMEOUT (msecs_to_jiffies(3800))

int mpng_init(unsigned long reg_base)
{
	s_mpng_reg_base = (unsigned long)ioremap(reg_base, (4 * SZ_4K));

	return 0;
}

int mpng_release(void)
{
	iounmap((void *)s_mpng_reg_base);
	s_mpng_reg_base = 0;

	return 0;
}

int mpng_reset(void)
{
/*	mpng_write_register(unsigned long addr,unsigned int data); */

	return 0;
}

int mpng_open(struct mpng_cap_t *cap)
{
	cap->width_min = 1;
	cap->height_min = 1;
	cap->width_max = 2048;
	cap->height_max = 2048;

	cap->interlacing = PNG_CAP_INTERLACING_DISABLED;
	cap->color_type = PNG_CAP_COLOR_TYPE(PNG_FMT_GRAYSCALE) |
					PNG_CAP_COLOR_TYPE(PNG_FMT_TRUECOLOR)  |
					PNG_CAP_COLOR_TYPE(PNG_FMT_INDEXED)  |
					PNG_CAP_COLOR_TYPE(PNG_FMT_GRAYSCALE_ALPHA)  |
					PNG_CAP_COLOR_TYPE(PNG_FMT_TRUECOLOR_ALPHA);

	return 0;
}

void mpng_start(void)
{
	mpng_write_register(PNG_CMD, START_CMD);
}

static void mpng_irq_enable(int mask)
{
	unsigned int reg = 0;

	reg = mpng_read_register(PNG_INT_CONTROL);
	reg |= mask;
	mpng_write_register(PNG_INT_CONTROL, reg);

	return ;
}

static void mpng_irq_disable(int mask)
{
	unsigned int reg = 0;

	reg = mpng_read_register(PNG_INT_CONTROL);
	reg &= (~mask);
	mpng_write_register(PNG_INT_CONTROL, reg);

	return ;
}

void mpng_irq_clear(void)
{
	mpng_write_register(PNG_INT_CLEAR, 0xffffffff);

	return ;
}

int mpng_decode(struct mpng_decode_conf_t *param, int stream_length)
{
	struct ihdr_t *img_hdr;

	img_hdr = &param->img_hdr;

	vlog_info(" start \n");

	mpng_irq_disable(INT_PNG_ALL_MASK);
	mpng_irq_enable(INT_PNG_ALL_EN);

	/* resolution setting */
	mpng_write_register(PNG_IMAGE_INFO1,
			(img_hdr->width << 16) | (img_hdr->height));

	/* color type & depth setting */
	mpng_write_register(PNG_IMAGE_INFO2,
			((img_hdr->color_type & 0x07) << 8) | (img_hdr->bit_depth & 0x1F));

	/* input/output iova address setting */
	mpng_write_register(PNG_READ_PTR_ADDR, param->bb_iova);
	mpng_write_register(PNG_READ_BYTE_SIZE, stream_length);
	mpng_write_register(PNG_WRITE_PTR_ADDR, param->fb_iova);

	mpng_start();

	return 0;
}


void mpng_mem_stream_rewind(struct stream_t *stream)
{
	stream->current_char = stream->data;
}

void mpng_mem_stream_init(struct stream_t *stream,
						unsigned char *data,
						int length)
{
	memset(stream, 0x0, sizeof(struct stream_t));

	stream->data = data;
	stream->length = length;

	mpng_mem_stream_rewind(stream);
}

mpng_chunk_type_e mpng_chunk_type_get(unsigned char *type)
{
	static const unsigned char IHDR[4] = { 'I',  'H',  'D',  'R' };
	static const unsigned char IDAT[4] = { 'I',  'D',  'A',  'T' };
	static const unsigned char IEND[4] = { 'I',  'E',  'N',  'D' };

	if (!memcmp(type, IDAT, 4))
		return PNG_CHUNK_IDAT;

	if (!memcmp(type, IHDR, 4))
		return PNG_CHUNK_IHDR;

	if (!memcmp(type, IEND, 4))
		return PNG_CHUNK_IEND;

	return PNG_CHUNK_UNKNOWN;
}

unsigned char *mpng_mem_stream_get_end(struct stream_t *stream)
{
	return (stream->data + stream->length);
}

unsigned char *mpng_mem_stream_get_current(struct stream_t *stream)
{
	return (stream->current_char);
}


int mpng_mem_stream_get_remained_length(struct stream_t *stream)
{
	return (int)((unsigned int)mpng_mem_stream_get_end(stream)
					- (unsigned int)mpng_mem_stream_get_current(stream));
}


unsigned char *mpng_mem_stream_read_string(struct stream_t *stream,
										int num_bytes)
{
	unsigned char *str = stream->current_char;
	stream->current_char += num_bytes;

	return str;
}

int mpng_mem_stream_write_string(struct stream_t *stream,
								unsigned char *data,
								int length)
{
	unsigned int avail_buf_sz = (unsigned int)mpng_mem_stream_get_end(stream)
									- (unsigned int)stream->current_char;

	if (length > avail_buf_sz)
		return 0;

	memcpy(stream->current_char, data, length);
	stream->current_char += length;

	return length;
}

unsigned int mpng_mem_stream_read_4byte(struct stream_t *stream)
{
	unsigned int a = *(stream->current_char++);
	unsigned int b = *(stream->current_char++);
	unsigned int c = *(stream->current_char++);
	unsigned int d = *(stream->current_char++);

	return (a << 24) | (b << 16) | (c << 8) | d;
}

unsigned char mpng_mem_stream_read_1byte(struct stream_t *stream)
{
	return *(stream->current_char++);
}

unsigned char *mpng_mem_stream_get_base(struct stream_t *stream)
{
	return (stream->data);
}

unsigned char *mpng_mem_steam_get_current(struct stream_t *stream)
{
	return (stream->current_char);
}

unsigned int mpng_mem_stream_get_offset(struct stream_t *stream)
{
	return ((unsigned int)stream->current_char - (unsigned int)stream->data);
}

int mpng_signature_check(struct stream_t *stream)
{
/*	const unsigned char signature[8] = {137, 80, 78, 71, 13, 10, 26, 10}; */
	const unsigned char signature[8] =
						{0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A};

	if (memcmp(mpng_mem_stream_read_string(stream, sizeof(signature)),
									signature, sizeof(signature)) != 0)
	{
		return -1;
	}
	return 0;
}

int mpng_do_ihdr(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream)
{
	struct stream_t hdr_stream;
	mpng_mem_stream_init(&hdr_stream, data, length);

	decode_param->img_hdr.width = mpng_mem_stream_read_4byte(&hdr_stream);
	decode_param->img_hdr.height = mpng_mem_stream_read_4byte(&hdr_stream);
	decode_param->img_hdr.bit_depth = mpng_mem_stream_read_1byte(&hdr_stream);
	decode_param->img_hdr.color_type = mpng_mem_stream_read_1byte(&hdr_stream);
	decode_param->img_hdr.compression_method =
								mpng_mem_stream_read_1byte(&hdr_stream);
	decode_param->img_hdr.filter_method =
								mpng_mem_stream_read_1byte(&hdr_stream);
	decode_param->img_hdr.interlace_method =
								mpng_mem_stream_read_1byte(&hdr_stream);

/*	vlog_info("hdr width:%d, height:%d \n", decode_param->img_hdr.width,
											decode_param->img_hdr.height);
	vlog_info("hdr color type:%x, bit_depth:%x \n",
											decode_param->img_hdr.color_type,
											decode_param->img_hdr.bit_depth);
	vlog_info("hdr compression_method:%x, filter_method:%x \n",
									decode_param->img_hdr.compression_method,
									decode_param->img_hdr.filter_method);
	vlog_info("hdr interlace_method:%x \n",
									decode_param->img_hdr.interlace_method);
*/

	return 0;
}

int mpng_do_idat(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream)
{
	if (mpng_mem_stream_get_base(bitstream) ==
			mpng_mem_steam_get_current(bitstream))
	{	/* first idat.	skip first two byte */
		data += 2;
		length -= 2;
	}

	return mpng_mem_stream_write_string(bitstream,
									(unsigned char*)data, length);
}

int mpng_do_iend(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream)
{
	decode_param->bb_length = mpng_mem_stream_get_offset(bitstream);

	/* the last 4 byte of zlib stream should be removed */
	decode_param->bb_length -= 4;

	return 0;
}


void mpng_write_register(unsigned long addr, unsigned int data)
{
	unsigned long *reg_addr = (unsigned long *)(addr +
											(unsigned long)s_mpng_reg_base);
	*(volatile unsigned long *)reg_addr = data;
}

unsigned long mpng_read_register(unsigned long addr)
{
	unsigned long *reg_addr = (unsigned long *)(addr +
											(unsigned long)s_mpng_reg_base);
	return *(volatile unsigned long *)reg_addr;
}

void mpng_spin_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}

void mpng_spin_lock(spinlock_t *lock, unsigned long *flags)
{
	spin_lock_irqsave(lock, *flags);
}

void mpng_spin_unlock(spinlock_t *lock, unsigned long *flags)
{
	spin_unlock_irqrestore(lock, *flags);
}

