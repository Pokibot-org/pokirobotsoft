.. _weact_stm32h743vi_board:

WEACT_STM32H743VI
################

<https://github.com/WeActTC/MiniSTM32H7xx>

Supported Features
==================

The Zephyr nucleo_h743zi board configuration supports the following hardware
features:

+-----------+------------+-------------------------------------+
| Interface | Controller | Driver/Component                    |
+===========+============+=====================================+
| NVIC      | on-chip    | nested vector interrupt controller  |
+-----------+------------+-------------------------------------+
| UART      | on-chip    | serial port                         |
+-----------+------------+-------------------------------------+
| PINMUX    | on-chip    | pinmux                              |
+-----------+------------+-------------------------------------+
| GPIO      | on-chip    | gpio                                |
+-----------+------------+-------------------------------------+
| RTC       | on-chip    | counter                             |
+-----------+------------+-------------------------------------+
| I2C       | on-chip    | i2c                                 |
+-----------+------------+-------------------------------------+
| PWM       | on-chip    | pwm                                 |
+-----------+------------+-------------------------------------+
| ADC       | on-chip    | adc                                 |
+-----------+------------+-------------------------------------+
| RNG       | on-chip    | True Random number generator        |
+-----------+------------+-------------------------------------+
| ETHERNET  | on-chip    | ethernet                            |
+-----------+------------+-------------------------------------+
| SPI       | on-chip    | spi                                 |
+-----------+------------+-------------------------------------+

Other hardware features are not yet supported on this Zephyr port.

The default configuration can be found in the defconfig file:
``boards/arm/nucleo_h743zi/nucleo_h743zi_defconfig``

For mode details please refer to `STM32 Nucleo-144 board User Manual`_.

Default Zephyr Peripheral Mapping:
----------------------------------

The Nucleo H743ZI board features a ST Zio connector (extended Arduino Uno V3)
and a ST morpho connector. Board is configured as follows:

- UART_3 TX/RX : PD8/PD9 (ST-Link Virtual Port Com)
- USER_PB : PC13
- LD1 : PB0
- LD2 : PB7
- LD3 : PB14
- I2C : PB8, PB9
- ADC1_INP15 : PA3
- ETH : PA1, PA2, PA7, PB13, PC1, PC4, PC5, PG11, PG13
- SPI1 SCK/MISO/MOSI : PA5/PA6/PB5 (Arduino SPI)

System Clock
------------

Nucleo H743ZI System Clock could be driven by an internal or external
oscillator, as well as the main PLL clock. By default, the System clock is
driven by the PLL clock at 96MHz, driven by an 8MHz high-speed external clock.

Serial Port
-----------

Nucleo H743ZI board has 4 UARTs and 4 USARTs. The Zephyr console output is
assigned to UART3. Default settings are 115200 8N1.

Programming and Debugging
*************************

Applications for the ``nucleo_h743zi`` board configuration can be built and
flashed in the usual way (see :ref:`build_an_application` and
:ref:`application_run` for more details).

.. note::

   If using OpenOCD you will need a recent development version as the last
   official release does not support H7 series yet. You can also choose the
   ``stm32cubeprogrammer`` runner.

Flashing
========

Nucleo H743ZI board includes an ST-LINK/V2-1 embedded debug tool interface.

Flashing an application to Nucleo H743ZI
----------------------------------------

Here is an example for the :ref:`hello_world` application.

Run a serial host program to connect with your Nucleo board.

.. code-block:: console

   $ minicom -b 115200 -D /dev/ttyACM0

Build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nucleo_h743zi
   :goals: build flash

You should see the following message on the console:

.. code-block:: console

   $ Hello World! nucleo_h743zi

Debugging
=========

You can debug an application in the usual way.  Here is an example for the
:ref:`hello_world` application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: nucleo_h743zi
   :maybe-skip-config:
   :goals: debug

.. _Nucleo H743ZI website:
   https://www.st.com/en/evaluation-tools/nucleo-h743zi.html

.. _STM32 Nucleo-144 board User Manual:
   http://www.st.com/resource/en/user_manual/dm00244518.pdf

.. _STM32H743ZI on www.st.com:
   https://www.st.com/content/st_com/en/products/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus/stm32-high-performance-mcus/stm32h7-series/stm32h743-753/stm32h743zi.html

.. _STM32H743 reference manual:
   https://www.st.com/resource/en/reference_manual/dm00314099.pdf
