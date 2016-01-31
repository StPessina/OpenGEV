#ifndef PIXELFORMAT_H
#define PIXELFORMAT_H

//===================================================
// PIXEL FORMATS
//===================================================
// Indicate if pixel is monochrome or RGB
#define GVSP_PIX_MONO 0x01000000
#define GVSP_PIX_RGB 0x02000000 // deprecated in version 1.1
#define GVSP_PIX_COLOR 0x02000000
#define GVSP_PIX_CUSTOM 0x80000000
#define GVSP_PIX_COLOR_MASK 0xFF000000
// Indicate effective number of bits occupied by the pixel (including padding).
// This can be used to compute amount of memory required to store an image.
#define GVSP_PIX_OCCUPY1BIT 0x00010000
#define GVSP_PIX_OCCUPY2BIT 0x00020000
#define GVSP_PIX_OCCUPY4BIT 0x00040000
#define GVSP_PIX_OCCUPY8BIT 0x00080000
#define GVSP_PIX_OCCUPY12BIT 0x000C0000
#define GVSP_PIX_OCCUPY16BIT 0x00100000
#define GVSP_PIX_OCCUPY24BIT 0x00180000
#define GVSP_PIX_OCCUPY32BIT 0x00200000
#define GVSP_PIX_OCCUPY36BIT 0x00240000
#define GVSP_PIX_OCCUPY48BIT 0x00300000

#define GVSP_PIX_EFFECTIVE_PIXEL_SIZE_MASK 0x00FF0000
#define GVSP_PIX_EFFECTIVE_PIXEL_SIZE_SHIFT 16
// Pixel ID: lower 16-bit of the pixel formats
#define GVSP_PIX_ID_MASK 0x0000FFFF
#define GVSP_PIX_COUNT 0x46
// next Pixel ID available

//Custom pixel number
#define GVSP_PIX_OCCUPY40BIT 0x00280000
#define GVSP_PIX_OCCUPY56BIT 0x00380000

//===================================================
// MONO FORMAT
//===================================================

#define GVSP_PIX_MONO16 (GVSP_PIX_MONO | GVSP_PIX_OCCUPY16BIT | 0x0007)

//===================================================
// BAYER FORMAT
//===================================================

//===================================================
// RGB FORMAT
//===================================================

#define GVSP_PIX_RGB8 (GVSP_PIX_COLOR | GVSP_PIX_OCCUPY24BIT | 0x0014)
#define GVSP_PIX_BGR8 (GVSP_PIX_COLOR | GVSP_PIX_OCCUPY24BIT | 0x0015)

#define GVSP_PIX_BGRA8 (GVSP_PIX_COLOR | GVSP_PIX_OCCUPY32BIT | 0x0017)

#define GVSP_PIX_RGB16 (GVSP_PIX_COLOR | GVSP_PIX_OCCUPY48BIT | 0x0033)

//===================================================
// YUV and YCbCr FORMAT
//===================================================

//===================================================
// RGB PLANAR FORMAT
//===================================================

//===================================================
// CUSTOM DEPTH FORMAT
//===================================================

#define GVSP_PIX_MONO32 (GVSP_PIX_CUSTOM | GVSP_PIX_OCCUPY32BIT | 0x0010)

//===================================================
// CUSTOM DEPTH + RGB FORMAT
//===================================================

#define GVSP_PIX_MONO16_RGB8 (GVSP_PIX_CUSTOM | GVSP_PIX_OCCUPY40BIT | 0x0001)
#define GVSP_PIX_MONO32_RGB8 (GVSP_PIX_CUSTOM | GVSP_PIX_OCCUPY56BIT | 0x0002)


#endif // PIXELFORMAT_H
