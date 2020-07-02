
/*
 * board specific functions for the nucleo144_f767zi board
 *
 */

#ifndef BOARDS_NUCLEO144_F767ZI_BARO_H
#define BOARDS_NUCLEO144_F767ZI_BARO_H

// only for printing the baro type during compilation
#ifndef BARO_BOARD
#define BARO_BOARD BARO_BMP3_I2C
#endif

extern void baro_event(void);
#define BaroEvent baro_event

#endif /* BOARDS_NUCLEO144_F767ZI_BARO_H */

