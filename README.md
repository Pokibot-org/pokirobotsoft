# POKIBOT ROBOT SOFTWARE

Cette repo contient le code des pokibots.
On utilise ici Zehpyr OS et platformio.


## Project organization
- Lib / HAL in lib/
- Tasks / routines / threads in src/, may be in a subfolder
    - Associated header in include/
    - Need to change CMakeList for those, add the path of both src and include folders

## CAMSENSE
On stm32f407:
pb6/pa9 : usart1_tx
pb7/pa10 : usart1_rx
default on pb6 pb7

## Materiel

Acheter un équivalent au stm32H743ZI
