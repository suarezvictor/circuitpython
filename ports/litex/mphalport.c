/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Glenn Ruben Bakke
 * Copyright (c) 2018 Artur Pacholec
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <string.h>

#include "uart.h"
//#include "lib/tinyusb/src/device/usbd.h"
//#include "lib/tinyusb/src/host/usbh.h"
#include "tusb.h"

#include "py/mphal.h"
#include "py/mpstate.h"
#include "py/gc.h"

#include "generated/csr.h"
#include "generated/soc.h"

#include "irq.h"

void mp_hal_delay_us(mp_uint_t delay) {
    mp_hal_delay_ms(delay / 1000);
}

extern void SysTick_Handler(void);

__attribute__((section(".ramtext")))
void isr(void) {
    uint8_t irqs = irq_pending() & irq_getmask();

#ifdef CFG_TUSB_MCU
    if (irqs & (1 << USB_INTERRUPT))
    {
        tud_int_handler(0);
        //uart_write('i');
    }
#endif

#ifdef USB_OHCI_CTRL_INTERRUPT //CFG_TUSB_RHPORT0_MODE & OPT_MODE_HOST
  if (irqs & (1 << USB_OHCI_CTRL_INTERRUPT))
    tuh_int_handler(0); //FIXME: tuh_isr is defined as hcd_isr (tuh_int_handler in newer versions of TinyUSB)
#endif


    if (irqs & (1 << TIMER0_INTERRUPT))
        SysTick_Handler();
}

mp_uint_t cpu_get_regs_and_sp(mp_uint_t *regs) {
    unsigned long __tmp;
    asm volatile ("mv %0, x2" :"=r"(__tmp));
    return __tmp;
}


void hcd_int_enable(uint8_t rhport)
{
    assert(rhport == 0);
    irq_setmask(irq_getmask() | (1 << USB_OHCI_CTRL_INTERRUPT));
}

void hcd_int_disable(uint8_t rhport)
{
    assert(rhport == 0);
    irq_setmask(irq_getmask() & ~(1 << USB_OHCI_CTRL_INTERRUPT));
}


