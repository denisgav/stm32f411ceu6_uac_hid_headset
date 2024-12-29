#ifndef VOLUME_CTRL__H
#define VOLUME_CTRL__H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// todo this seemed like good guess, but is not correct
static const uint16_t db_to_vol[91] = {
        0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0001, 0x0002, 0x0002,
        0x0002, 0x0002, 0x0003, 0x0003, 0x0004, 0x0004, 0x0005, 0x0005,
        0x0006, 0x0007, 0x0008, 0x0009, 0x000a, 0x000b, 0x000d, 0x000e,
        0x0010, 0x0012, 0x0014, 0x0017, 0x001a, 0x001d, 0x0020, 0x0024,
        0x0029, 0x002e, 0x0033, 0x003a, 0x0041, 0x0049, 0x0052, 0x005c,
        0x0067, 0x0074, 0x0082, 0x0092, 0x00a4, 0x00b8, 0x00ce, 0x00e7,
        0x0104, 0x0124, 0x0147, 0x016f, 0x019c, 0x01ce, 0x0207, 0x0246,
        0x028d, 0x02dd, 0x0337, 0x039b, 0x040c, 0x048a, 0x0518, 0x05b7,
        0x066a, 0x0732, 0x0813, 0x090f, 0x0a2a, 0x0b68, 0x0ccc, 0x0e5c,
        0x101d, 0x1214, 0x1449, 0x16c3, 0x198a, 0x1ca7, 0x2026, 0x2413,
        0x287a, 0x2d6a, 0x32f5, 0x392c, 0x4026, 0x47fa, 0x50c3, 0x5a9d,
        0x65ac, 0x7214, 0x7fff
};

// actually windows doesn't seem to like this in the middle, so set top range to 0db
#define COUTNOF_DB_TO_VOL 91
#define CENTER_VOLUME_INDEX 91
#define ENC_NUM_OF_FP_BITS 8

#define ENCODE_DB(x) ((uint16_t)(int16_t)((x)<<ENC_NUM_OF_FP_BITS))

#define DEFAULT_VOLUME        (0)

#define MIN_VOLUME_ENC        ENCODE_DB(-CENTER_VOLUME_INDEX)
#define MAX_VOLUME_ENC        ENCODE_DB(COUTNOF_DB_TO_VOL-CENTER_VOLUME_INDEX)
#define VOLUME_RESOLUTION_ENC ENCODE_DB(1)

#define MIN_VOLUME        (-CENTER_VOLUME_INDEX)
#define MAX_VOLUME        (COUTNOF_DB_TO_VOL-CENTER_VOLUME_INDEX)
#define VOLUME_RESOLUTION (1)

uint16_t vol_to_db_convert(bool channel_mute, uint16_t channel_volume);
uint16_t vol_to_db_convert_enc(bool channel_mute, uint16_t channel_volume);

#ifdef __cplusplus
}
#endif


#endif //VOLUME_CTRL__H
