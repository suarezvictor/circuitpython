/**************************************************************************/
/*!
    @file     tusb_config.h
    @author   hathach (tinyusb.org)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2013, hathach (tinyusb.org)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    INCLUDING NEGLIGENCE OR OTHERWISE ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
/**************************************************************************/

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#include "genhdr/autogen_usb_descriptor.h"
#include <generated/csr.h>

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// COMMON CONFIGURATION
//--------------------------------------------------------------------+
#define CFG_TUSB_RHPORT0_MODE       (OPT_MODE_DEVICE | OPT_MODE_HOST)

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG              0
#endif

/*------------- RTOS -------------*/
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS                 OPT_OS_NONE
#endif
//#define CFG_TUD_TASK_QUEUE_SZ     16
//#define CFG_TUD_TASK_PRIO         0
//#define CFG_TUD_TASK_STACK_SZ     150

//--------------------------------------------------------------------+
// DEVICE CONFIGURATION
//--------------------------------------------------------------------+

#define CFG_TUD_ENDOINT0_SIZE       64

/*------------- Descriptors -------------*/
/* Enable auto generated descriptor, tinyusb will try its best to create
 * descriptor ( device, configuration, hid ) that matches enabled CFG_* in this file
 *
 * Note: All CFG_TUD_DESC_* are relevant only if CFG_TUD_DESC_AUTO is enabled
 */
#define CFG_TUD_DESC_AUTO           0

//------------- CLASS -------------//
#define CFG_TUD_CDC                 1
#define CFG_TUD_MSC                 1
#define CFG_TUD_HID                 CIRCUITPY_USB_HID
#define CFG_TUD_MIDI                CIRCUITPY_USB_MIDI
#define CFG_TUD_CUSTOM_CLASS        0

/*------------------------------------------------------------------*/
/* CLASS DRIVER
 *------------------------------------------------------------------*/

/* TX is sent automatically on every Start of Frame event ~ 1ms.
 * If not enabled, application must call tud_cdc_flush() periodically
 * Note: Enabled this could overflow device task, if it does, define
 * CFG_TUD_TASK_QUEUE_SZ with large value
 */
#define CFG_TUD_CDC_FLUSH_ON_SOF    0


/*------------- MSC -------------*/
// Number of supported Logical Unit Number (At least 1)
#define CFG_TUD_MSC_MAXLUN          1

// Number of Blocks
#define CFG_TUD_MSC_BLOCK_NUM       (256*1024)/512



// Product revision string included in Inquiry response, max 4 bytes
#define CFG_TUD_MSC_PRODUCT_REV     "1.0"


//--------------------------------------------------------------------+
// USB RAM PLACEMENT
//--------------------------------------------------------------------+
#define CFG_TUSB_ATTR_USBRAM
#define CFG_TUSB_MEM_ALIGN          __attribute__ ((aligned(4)))


//--------------------------------------------------------------------
// HOST CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUSB_HOST_DEVICE_MAX    4

// Size of buffer to hold descriptors and other data used for enumeration
#define CFG_TUH_ENUMERATION_BUFSIZE 256

#define CFG_TUH_HUB                 1 //it seems must be set
#define CFG_TUH_CDC                 1
#define CFG_TUH_MSC                 1
#define CFG_TUH_VENDOR              0

// max device support (excluding hub device)
#define CFG_TUH_DEVICE_MAX          (CFG_TUH_HUB ? 4 : 1) // hub typically has 4 ports


//------------- HID -------------//
#define CFG_TUH_HID                  4 // typical keyboard + mouse device can have 3-4 HID interfaces
#define CFG_TUH_HID_EPIN_BUFSIZE    64
#define CFG_TUH_HID_EPOUT_BUFSIZE   64

#define CFG_TUH_HID_KEYBOARD 1
#define CFG_TUH_HID_MOUSE 1
#define CFG_TUSB_HOST_HID_GENERIC 1

#define LPC_USB_BASE USB_OHCI_BASE //from soc.h, same as usb_ohci_base_read()
#define TUP_USBIP_OHCI //needed for ohci.c
 

typedef void *pipe_handle_t;

#ifdef __cplusplus
 }
#endif

#endif /* _TUSB_CONFIG_H_ */
