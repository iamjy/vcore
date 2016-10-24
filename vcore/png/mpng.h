
#ifndef _MPNG_LINUX_H_
#define _MPNG_LINUX_H_

/* PNG register's offsets */
#define PNG_CMD					0x0000
#define PNG_READ_PTR_ADDR		0x0004
#define PNG_READ_BYTE_SIZE		0x0008
#define PNG_WRITE_PTR_ADDR		0x000C

#define PNG_IMAGE_INFO1			0x0010
#define PNG_IMAGE_INFO2			0x0014

#define PNG_INT_CONTROL			0x0018
#define PNG_INT_STATUS			0x001C
#define PNG_INT_CLEAR			0x0020

#define PNG_READ_GMAU_CONF		0x0024
#define PNG_WRITE_GMAU_CONF		0x0028
#define PNG_INFLATE_CONF		0x002C

#define PNG_FIFO_STATUS			0x0030
#define PNG_INFLATE_STATUS		0x0034

#define PNG_READ_CUR_ADDR		0x0038
#define PNG_FTR_CUR_ADDR		0x003C
#define PNG_FTR_WRTR_BND		0x0040

#define PNG_WRITE_CUR_ADDR		0x0044
#define PNG_FTR_SL_CNT_MSB		0x0048
#define PNG_FTR_SL_CNT_LSB		0x004C
#define PNG_FTR_IN_CNT_MSB		0x0050
#define PNG_FTR_IN_CNT_LSB		0x0054
#define PNG_FTR_TYPE			0x0058
#define PNG_FTR_STATUS			0x005C

#define PNG_TIME_OUT_MASK		0x0060
#define PNG_TIME_CUT_CNT		0x0064


/* png register's bit-fields */

/* STATUS */
#define STAT_BUSY		(0x2)
#define STAT_DONE		(0x1)


/* START CONTROL */
#define START_CMD		(0x1)
#define MID_START_CMD	(0x8)

/* INT CONTROL */
#define INT_TIME_OUT_EN  	(0x1 << 12)
#define INT_READ_DONE_EN  	(0x1 << 11)
#define INT_FTYPE_ERR_EN 	(0x1 << 10)
#define INT_ROW_ERR_EN  	(0x1 << 9)
#define INT_LEN_ERR_EN  	(0x1 << 8)
#define INT_BTYPE_ERR_EN  	(0x1 << 7)
#define INT_CODE_ERR_EN  	(0x1 << 6)
#define INT_WIN_ERR_EN  	(0x1 << 5)
#define INT_TGEN_ERR_EN  	(0x1 << 4)
#define INT_TLEN_ERR_EN  	(0x1 << 3)
#define INT_TCOPT_ERR_EN  	(0x1 << 2)
#define INT_INFLATE_DONE_EN	(0x2)
#define INT_PNG_DONE_EN  	(0x1)
#define INT_PNG_ALL_EN		(0x17ff)
#define INT_PNG_ALL_MASK	(0)

/* INT STATUS */
 #define INT_TIME_OUT_STATUS  	(0x1 << 12)
 #define INT_READ_DONE_STATUS  	(0x1 << 11)
 #define INT_FTYPE_ERR_STATUS 	(0x1 << 10)
 #define INT_ROW_ERR_STATUS  	(0x1 << 9)
 #define INT_LEN_ERR_STATUS  	(0x1 << 8)
 #define INT_BTYPE_ERR_STATUS  	(0x1 << 7)
 #define INT_CODE_ERR_STATUS  	(0x1 << 6)
 #define INT_WIN_ERR_STATUS  	(0x1 << 5)
 #define INT_TGEN_ERR_STATUS  	(0x1 << 4)
 #define INT_TLEN_ERR_STATUS  	(0x1 << 3)
 #define INT_TCOPT_ERR_STATUS  	(0x1 << 2)
 #define INT_INFLATE_DONE_STATUS	(0x2)
 #define INT_PNG_DONE_STATUS  	(0x1)

/* INT CLEAR */
 #define INT_TIME_OUT_CLEAR  	(0x1 << 12)
 #define INT_READ_DONE_CLEAR  	(0x1 << 11)
 #define INT_FTYPE_ERR_CLEAR 	(0x1 << 10)
 #define INT_ROW_ERR_CLEAR  	(0x1 << 9)
 #define INT_LEN_ERR_CLEAR  	(0x1 << 8)
 #define INT_BTYPE_ERR_CLEAR  	(0x1 << 7)
 #define INT_CODE_ERR_CLEAR  	(0x1 << 6)
 #define INT_WIN_ERR_CLEAR  	(0x1 << 5)
 #define INT_TGEN_ERR_CLEAR  	(0x1 << 4)
 #define INT_TLEN_ERR_CLEAR  	(0x1 << 3)
 #define INT_TCOPT_ERR_CLEAR  	(0x1 << 2)
 #define INT_INFLATE_DONE_CLEAR	(0x2)
 #define INT_PNG_DONE_CLEAR  	(0x1)

/* READ GMAU CONFIG*/
#define READ_CMD_RTXID			(0xf << 24)
#define READ_CMD_RDLY			(0xffff << 8)
#define READ_CMD_RPRI1			(0xf << 4)
#define READ_CMD_RPRI2			(0xf)

/* WRITE GMAU CONFIG*/
#define WRITE_CMD_RTXID			(0xf << 24)
#define WRITE_CMD_RDLY			(0xffff << 8)
#define WRITE_CMD_RPRI1			(0xf << 4)
#define WRITE_CMD_RPRI2			(0xf)

enum mpng_is_status_e {
	PNG_IDLE=0,
	PNG_READY,
	PNG_DECODING,
	PNG_DONE,
	PNG_ERROR,
	PNG_END,
};
enum mpng_error_status_e {
	PNG_ERROR_INIT=10,
	PNG_ERROR_MASK_SIZE,
	PNG_ERROR_BUSY,
	PNG_ERROR_IRQ,
};

typedef enum {
	PNG_CAP_INTERLACING_DISABLED = 0,
	PNG_CAP_INTERLACING_ENABLED = 1,
} mpng_interlacing_e;

enum mpng_fmt_e {
	PNG_FMT_GRAYSCALE = 0,
	PNG_FMT_INDEXED_GRAYSCALE = 1,
	PNG_FMT_TRUECOLOR = 2,
	PNG_FMT_INDEXED = 3,
	PNG_FMT_GRAYSCALE_ALPHA = 4,
	PNG_FMT_INDEXED_GRAYSCALE_ALPHA = 5,
	PNG_FMT_TRUECOLOR_ALPHA = 6,
	PNG_FMT_INDEXED_ALPHA = 7,
	PNG_FMT_MAX = 8,
};

typedef enum {
	PNG_CHUNK_IHDR  = 0,
	PNG_CHUNK_IDAT  = 1,
	PNG_CHUNK_IEND  = 2,
	PNG_CHUNK_UNKNOWN,
} mpng_chunk_type_e;


#define PNG_CAP_COLOR_TYPE(x)	(1<<x)


/***************************************************************************/
/* Internal header files */
/***************************************************************************/
#include <linux/types.h>
/***************************************************************************/
/* Local Literals & definitions */
/***************************************************************************/

struct ihdr_t {
	unsigned int width;
	unsigned int height;
	unsigned char bit_depth;
	unsigned char color_type;
	unsigned char compression_method;
	unsigned char filter_method;
	unsigned char interlace_method;
} ;

struct mpng_decode_conf_t {
	struct ihdr_t img_hdr;
	unsigned long bb_iova;
			/* bitstream buffer address. must be physical address */
	unsigned char *bb_vir_ptr;
	unsigned int bb_length;

	unsigned long fb_iova;
			/* frame buffer address. must be physical address */
	unsigned int fb_length;
} ;

struct mpng_ctx_t {
 	struct mpng_decode_conf_t conf;

	unsigned int status;
	unsigned int reserved;
} ;

struct stream_t {
	unsigned char *data;
	int length;
	unsigned char *current_char;
} ;

/**
 *	the device capability
 */
struct mpng_cap_t {
	mpng_interlacing_e interlacing;

	unsigned int color_type;

	unsigned int width_min;
	unsigned int height_min;
	unsigned int width_max;
	unsigned int height_max;
};

#if defined (__cplusplus)
extern "C" {
#endif

int mpng_init(unsigned long reg_base);
int mpng_release(void);

void mpng_irq_clear(void);
int mpng_decode(struct mpng_decode_conf_t *param, int stream_length);

mpng_chunk_type_e mpng_chunk_type_get(unsigned char *type);
int mpng_mem_stream_get_remained_length(struct stream_t *stream);
unsigned char *mpng_mem_stream_read_string(struct stream_t *stream,
									int num_bytes);
unsigned int mpng_mem_stream_read_4byte(struct stream_t *stream);
void mpng_mem_stream_init(struct stream_t *stream,
						unsigned char *data,
						int length);
int mpng_signature_check(struct stream_t *stream);

int mpng_do_ihdr(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream);
int mpng_do_idat(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream);
int mpng_do_iend(char *data,
				int length,
				struct mpng_decode_conf_t *decode_param,
				struct stream_t *bitstream);


void mpng_write_register(unsigned long addr, unsigned int data);
unsigned long mpng_read_register(unsigned long addr);

void mpng_spin_lock_init(spinlock_t *lock);
void mpng_spin_lock(spinlock_t *lock, unsigned long *flags);
void mpng_spin_unlock(spinlock_t *lock, unsigned long *flags);

#if defined (__cplusplus)
}
#endif

#endif
