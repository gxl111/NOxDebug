#ifndef __FONT_H
#define __FONT_H
#include "stdint.h"
#include "string.h"
typedef struct ASCIIFont {
    uint8_t h;
    uint8_t w;
    uint8_t *chars;
} ASCIIFont;

extern const ASCIIFont afont8x6;
extern const ASCIIFont afont12x6;
extern const ASCIIFont afont16x8;
extern const ASCIIFont afont24x12;

/**
 * @brief Font descriptor
 * @note  First 4 bytes of each glyph entry = UTF-8 code; remainder = bitmap
 * @note  Glyph data can be generated with Baud-Dance LED font tool (https://led.baud-dance.com)
 */
typedef struct Font {
    uint8_t h;              /* Glyph height */
    uint8_t w;              /* Glyph width */
    const uint8_t *chars;   /* Font table: UTF-8 prefix + bitmap per glyph */
    uint8_t len;            /* Number of glyphs; use uint16_t if >256 */
    const ASCIIFont *ascii; /* Fallback ASCII font when glyph not in table */
} Font;

extern const Font font16x16;

/**
 * @brief Image descriptor
 * @note  Image data from Baud-Dance LED tool (https://led.baud-dance.com)
 */
typedef struct Image {
    uint8_t w;           /* Width */
    uint8_t h;           /* Height */
    const uint8_t *data; /* Bitmap */
} Image;

extern const Image bilibiliImg;

#endif /* __FONT_H */
