#ifndef XOSHIRO128PLUSPLUS_H
#define XOSHIRO128PLUSPLUS_H

uint32_t xoshiro128plusplus_next(void);
uint8_t xoshiro128plusplus_init_seed(uint32_t init_tab[4]);

#endif