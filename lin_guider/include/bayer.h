/*
 * Created by Rumen G. Bogdanovski
 * based on some definitions in v4l2
 */
#ifndef _BAYER_H
#define _BAYER_H

#ifndef v4l2_fourcc
#include <inttypes.h>
#define v4l2_fourcc(a, b, c, d)\
        ((uint32_t)(a) | ((uint32_t)(b) << 8) | ((uint32_t)(c) << 16) | ((uint32_t)(d) << 24))
#endif

#ifndef PIX_FMT_SPCA501
#define PIX_FMT_SPCA501 v4l2_fourcc('S','5','0','1') /* YUYV per line */
#endif

#ifndef PIX_FMT_SPCA505
#define PIX_FMT_SPCA505 v4l2_fourcc('S','5','0','5') /* YYUV per line */
#endif

#ifndef PIX_FMT_SPCA508
#define PIX_FMT_SPCA508 v4l2_fourcc('S','5','0','8') /* YUVY per line */
#endif

#ifndef PIX_FMT_SPCA561
#define PIX_FMT_SPCA561 v4l2_fourcc('S','5','6','1')
#endif

#ifndef PIX_FMT_PAC207
#define PIX_FMT_PAC207 v4l2_fourcc('P','2','0','7')
#endif

#ifndef PIX_FMT_MR97310A
#define PIX_FMT_MR97310A v4l2_fourcc('M','3','1','0')
#endif

#ifndef PIX_FMT_SN9C2028
#define PIX_FMT_SN9C2028 v4l2_fourcc('S', 'O', 'N', 'X')
#endif

#ifndef PIX_FMT_SQ905C
#define PIX_FMT_SQ905C v4l2_fourcc('9', '0', '5', 'C')
#endif

#ifndef PIX_FMT_PJPG
#define PIX_FMT_PJPG v4l2_fourcc('P', 'J', 'P', 'G')
#endif

#ifndef PIX_FMT_SGBRG8
#define PIX_FMT_SGBRG8 v4l2_fourcc('G','B','R','G')
#endif

#ifndef PIX_FMT_SGRBG8
#define PIX_FMT_SGRBG8 v4l2_fourcc('G','R','B','G')
#endif

#ifndef PIX_FMT_SRGGB8
#define PIX_FMT_SRGGB8 v4l2_fourcc('R','G','G','B')
#endif

#ifndef PIX_FMT_SBGGR8
#define PIX_FMT_SBGGR8 v4l2_fourcc('B','G','G','R')
#endif

#ifndef PIX_FMT_YVYU
#define PIX_FMT_YVYU v4l2_fourcc('Y', 'V', 'Y', 'U')
#endif

#ifndef PIX_FMT_HM12
#define PIX_FMT_HM12 v4l2_fourcc('H', 'M', '1', '2')
#endif

#ifndef PIX_FMT_SN9C20X_I420
#define PIX_FMT_SN9C20X_I420  v4l2_fourcc('S', '9', '2', '0')
#endif

#ifndef PIX_FMT_OV511
#define PIX_FMT_OV511 v4l2_fourcc('O', '5', '1', '1')
#endif

#ifndef PIX_FMT_OV518
#define PIX_FMT_OV518 v4l2_fourcc('O', '5', '1', '8') /* ov518 JPEG */
#endif

#ifdef __cplusplus
extern "C" {
#endif

void bayer_to_rgb24(const unsigned char *bayer,
  unsigned char *rgb, int width, int height, unsigned int pixfmt);

void bayer_to_bgr24(const unsigned char *bayer,
  unsigned char *rgb, int width, int height, unsigned int pixfmt);

void bayer_to_rgb48(const uint16_t *bayer,
  uint16_t *rgb, int width, int height, unsigned int pixfmt);

void bayer_to_bgr48(const uint16_t *bayer,
  uint16_t *rgb, int width, int height, unsigned int pixfmt);

#ifdef __cplusplus
}
#endif

#endif
