/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016-2017 Scott Shawcroft for Adafruit Industries
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

#include <generated/soc.h>
#include <tusb.h>
//#include <lib/tinyusb/src/host/ohci/ohci.h>
#include <portable/ohci/ohci.h>
#include <uart.c> //FIXME: move to makefile
/////////////////////////////

#include <stdint.h>
#include <string.h>

#include "extmod/vfs.h"
#include "extmod/vfs_fat.h"

#include "genhdr/mpversion.h"
#include "py/nlr.h"
#include "py/compile.h"
#include "py/frozenmod.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/stackctrl.h"

#include "lib/mp-readline/readline.h"
#include "lib/utils/pyexec.h"

#include "background.h"
#include "mpconfigboard.h"
#include "supervisor/cpu.h"
#include "supervisor/memory.h"
#include "supervisor/port.h"
#include "supervisor/filesystem.h"
#include "supervisor/shared/autoreload.h"
#include "supervisor/shared/translate.h"
#include "supervisor/shared/rgb_led_status.h"
#include "supervisor/shared/safe_mode.h"
#include "supervisor/shared/status_leds.h"
#include "supervisor/shared/stack.h"
#include "supervisor/serial.h"

#include "boards/board.h"

#if CIRCUITPY_DISPLAYIO
#include "shared-module/displayio/__init__.h"
#endif

#if CIRCUITPY_NETWORK
#include "shared-module/network/__init__.h"
#endif

#if CIRCUITPY_BOARD
#include "shared-module/board/__init__.h"
#endif

#if CIRCUITPY_BLEIO
#include "shared-bindings/_bleio/__init__.h"
#include "supervisor/shared/bluetooth.h"
#endif

void do_str(const char *src, mp_parse_input_kind_t input_kind) {
    mp_lexer_t *lex = mp_lexer_new_from_str_len(MP_QSTR__lt_stdin_gt_, src, strlen(src), 0);
    if (lex == NULL) {
        //printf("MemoryError: lexer could not allocate memory\n");
        return;
    }

    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        qstr source_name = lex->source_name;
        mp_parse_tree_t parse_tree = mp_parse(lex, input_kind);
        mp_obj_t module_fun = mp_compile(&parse_tree, source_name, MP_EMIT_OPT_NONE, true);
        mp_call_function_0(module_fun);
        nlr_pop();
    } else {
        // uncaught exception
        mp_obj_print_exception(&mp_plat_print, (mp_obj_t)nlr.ret_val);
    }
}

void start_mp(supervisor_allocation* heap) {
    reset_status_led();
    autoreload_stop();

    background_tasks_reset();

    // Stack limit should be less than real stack size, so we have a chance
    // to recover from limit hit.  (Limit is measured in bytes.)
    mp_stack_ctrl_init();

    if (stack_alloc != NULL) {
        mp_stack_set_limit(stack_alloc->length - 1024);
    }


#if MICROPY_MAX_STACK_USAGE
    // _ezero (same as _ebss) is an int, so start 4 bytes above it.
    if (stack_alloc != NULL) {
        mp_stack_set_bottom(stack_alloc->ptr);
        mp_stack_fill_with_sentinel();
    }
#endif

    // Sync the file systems in case any used RAM from the GC to cache. As soon
    // as we re-init the GC all bets are off on the cache.
    filesystem_flush();

    // Clear the readline history. It references the heap we're about to destroy.
    readline_init0();

    #if MICROPY_ENABLE_GC
    gc_init(heap->ptr, heap->ptr + heap->length / 4);
    #endif
    mp_init();
    mp_obj_list_init(mp_sys_path, 0);
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR_)); // current dir (or base dir of the script)
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR__slash_));
    // Frozen modules are in their own pseudo-dir, e.g., ".frozen".
    // Prioritize .frozen over /lib.
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_FROZEN_FAKE_DIR_QSTR));
    mp_obj_list_append(mp_sys_path, MP_OBJ_NEW_QSTR(MP_QSTR__slash_lib));

    mp_obj_list_init(mp_sys_argv, 0);

    #if CIRCUITPY_NETWORK
    network_module_init();
    #endif
}

void stop_mp(void) {
    #if CIRCUITPY_NETWORK
    network_module_deinit();
    #endif

    #if MICROPY_VFS
    mp_vfs_mount_t *vfs = MP_STATE_VM(vfs_mount_table);

    // Unmount all heap allocated vfs mounts.
    while (gc_nbytes(vfs) > 0) {
        vfs = vfs->next;
    }
    MP_STATE_VM(vfs_mount_table) = vfs;
    MP_STATE_VM(vfs_cur) = vfs;
    #endif

    gc_deinit();
}

#define STRING_LIST(...) {__VA_ARGS__, ""}

// Look for the first file that exists in the list of filenames, using mp_import_stat().
// Return its index. If no file found, return -1.
const char* first_existing_file_in_list(const char ** filenames) {
    for (int i = 0; filenames[i] != (char*)""; i++) {
        mp_import_stat_t stat = mp_import_stat(filenames[i]);
        if (stat == MP_IMPORT_STAT_FILE) {
            return filenames[i];
        }
    }
    return NULL;
}

bool maybe_run_list(const char ** filenames, pyexec_result_t* exec_result) {
    const char* filename = first_existing_file_in_list(filenames);
    if (filename == NULL) {
        return false;
    }
    mp_hal_stdout_tx_str(filename);
    const compressed_string_t* compressed = translate(" output:\n");
    char decompressed[decompress_length(compressed)];
    decompress(compressed, decompressed);
    mp_hal_stdout_tx_str(decompressed);
    pyexec_file(filename, exec_result);
    return true;
}

void cleanup_after_vm(supervisor_allocation* heap) {
    // Turn off the display and flush the fileystem before the heap disappears.
    #if CIRCUITPY_DISPLAYIO
    reset_displays();
    #endif
    filesystem_flush();
    stop_mp();
    free_memory(heap);
    supervisor_move_memory();

    reset_port();
    #if CIRCUITPY_BOARD
    reset_board_busses();
    #endif
    reset_board();
    reset_status_led();
}

bool run_code_py(safe_mode_t safe_mode) {
    bool serial_connected_at_start = serial_connected();
    #if CIRCUITPY_AUTORELOAD_DELAY_MS > 0
    if (serial_connected_at_start) {
        serial_write("\n");
        if (autoreload_is_enabled()) {
            serial_write_compressed(translate("Auto-reload is on. Simply save files over USB to run them or enter REPL to disable.\n"));
        } else if (safe_mode != NO_SAFE_MODE) {
            serial_write_compressed(translate("Running in safe mode! Auto-reload is off.\n"));
        } else if (!autoreload_is_enabled()) {
            serial_write_compressed(translate("Auto-reload is off.\n"));
        }
    }
    #endif

    pyexec_result_t result;

    result.return_code = 0;
    result.exception_type = NULL;
    result.exception_line = 0;

    bool found_main = false;

    if (safe_mode != NO_SAFE_MODE) {
        serial_write_compressed(translate("Running in safe mode! Not running saved code.\n"));
    } else {
        new_status_color(MAIN_RUNNING);

        static const char *supported_filenames[] = STRING_LIST("code.txt", "code.py", "main.py", "main.txt");
        static const char *double_extension_filenames[] = STRING_LIST("code.txt.py", "code.py.txt", "code.txt.txt","code.py.py",
                                                    "main.txt.py", "main.py.txt", "main.txt.txt","main.py.py");

        stack_resize();
        filesystem_flush();
        supervisor_allocation* heap = allocate_remaining_memory();
        start_mp(heap);
        found_main = maybe_run_list(supported_filenames, &result);
        if (!found_main){
            found_main = maybe_run_list(double_extension_filenames, &result);
            if (found_main) {
                serial_write_compressed(translate("WARNING: Your code filename has two extensions\n"));
            }
        }
        cleanup_after_vm(heap);

        if (result.return_code & PYEXEC_FORCED_EXIT) {
            return reload_requested;
        }
    }

    // Wait for connection or character.
    if (!serial_connected_at_start) {
        serial_write_compressed(translate("\nCode done running. Waiting for reload.\n"));
    }

    bool serial_connected_before_animation = false;
    #if CIRCUITPY_DISPLAYIO
    bool refreshed_epaper_display = false;
    #endif
    rgb_status_animation_t animation;
    prep_rgb_status_animation(&result, found_main, safe_mode, &animation);
    while (true) {
        RUN_BACKGROUND_TASKS;
        if (reload_requested) {
            reload_requested = false;
            return true;
        }

        if (serial_connected() && serial_bytes_available()) {
            // Skip REPL if reload was requested.
            return (serial_read() == CHAR_CTRL_D);
        }

        if (!serial_connected_before_animation && serial_connected()) {
            if (serial_connected_at_start) {
                serial_write("\n\n");
            }

            if (!serial_connected_at_start) {
                if (autoreload_is_enabled()) {
                    serial_write_compressed(translate("Auto-reload is on. Simply save files over USB to run them or enter REPL to disable.\n"));
                } else {
                    serial_write_compressed(translate("Auto-reload is off.\n"));
                }
            }
            print_safe_mode_message(safe_mode);
            serial_write("\n");
            serial_write_compressed(translate("Press any key to enter the REPL. Use CTRL-D to reload."));
        }
        if (serial_connected_before_animation && !serial_connected()) {
            serial_connected_at_start = false;
        }
        serial_connected_before_animation = serial_connected();

        // Refresh the ePaper display if we have one. That way it'll show an error message.
        #if CIRCUITPY_DISPLAYIO
        if (!refreshed_epaper_display) {
            refreshed_epaper_display = maybe_refresh_epaperdisplay();
        }
        #endif

        tick_rgb_status_animation(&animation);
    }
}

void __attribute__ ((noinline)) run_boot_py(safe_mode_t safe_mode) {
    // If not in safe mode, run boot before initing USB and capture output in a
    // file.
    if (filesystem_present() && safe_mode == NO_SAFE_MODE && MP_STATE_VM(vfs_mount_table) != NULL) {
        static const char *boot_py_filenames[] = STRING_LIST("settings.txt", "settings.py", "boot.py", "boot.txt");

        new_status_color(BOOT_RUNNING);

        #ifdef CIRCUITPY_BOOT_OUTPUT_FILE
        FIL file_pointer;
        boot_output_file = &file_pointer;

        // Get the base filesystem.
        FATFS *fs = &((fs_user_mount_t *) MP_STATE_VM(vfs_mount_table)->obj)->fatfs;

        bool have_boot_py = first_existing_file_in_list(boot_py_filenames) != NULL;

        bool skip_boot_output = false;

        // If there's no boot.py file that might write some changing output,
        // read the existing copy of CIRCUITPY_BOOT_OUTPUT_FILE and see if its contents
        // match the version info we would print anyway. If so, skip writing CIRCUITPY_BOOT_OUTPUT_FILE.
        // This saves wear and tear on the flash and also prevents filesystem damage if power is lost
        // during the write, which may happen due to bobbling the power connector or weak power.

        static const size_t NUM_CHARS_TO_COMPARE = 160;
        if (!have_boot_py && f_open(fs, boot_output_file, CIRCUITPY_BOOT_OUTPUT_FILE, FA_READ) == FR_OK) {

            char file_contents[NUM_CHARS_TO_COMPARE];
            UINT chars_read = 0;
            f_read(boot_output_file, file_contents, NUM_CHARS_TO_COMPARE, &chars_read);
            f_close(boot_output_file);
            skip_boot_output =
                // + 2 accounts for  \r\n.
                chars_read == strlen(MICROPY_FULL_VERSION_INFO) + 2 &&
                strncmp(file_contents, MICROPY_FULL_VERSION_INFO, strlen(MICROPY_FULL_VERSION_INFO)) == 0;
        }

        if (!skip_boot_output) {
            // Wait 1.5 seconds before opening CIRCUITPY_BOOT_OUTPUT_FILE for write,
            // in case power is momentary or will fail shortly due to, say a low, battery.
            mp_hal_delay_ms(1500);

            // USB isn't up, so we can write the file.
            filesystem_set_internal_writable_by_usb(false);
            f_open(fs, boot_output_file, CIRCUITPY_BOOT_OUTPUT_FILE, FA_WRITE | FA_CREATE_ALWAYS);

            // Switch the filesystem back to non-writable by Python now instead of later,
            // since boot.py might change it back to writable.
            filesystem_set_internal_writable_by_usb(true);

            // Write version info to boot_out.txt.
            mp_hal_stdout_tx_str(MICROPY_FULL_VERSION_INFO);
            mp_hal_stdout_tx_str("\r\n");
        }
        #endif

        // TODO(tannewt): Allocate temporary space to hold custom usb descriptors.
        filesystem_flush();
        supervisor_allocation* heap = allocate_remaining_memory();
        start_mp(heap);

        // TODO(tannewt): Re-add support for flashing boot error output.
        bool found_boot = maybe_run_list(boot_py_filenames, NULL);
        (void) found_boot;

        #ifdef CIRCUITPY_BOOT_OUTPUT_FILE
        if (!skip_boot_output) {
            f_close(boot_output_file);
            filesystem_flush();
        }
        boot_output_file = NULL;
        #endif

        cleanup_after_vm(heap);
    }
}

int run_repl(void) {
    int exit_code = PYEXEC_FORCED_EXIT;
    stack_resize();
    filesystem_flush();
    supervisor_allocation* heap = allocate_remaining_memory();
    start_mp(heap);
    autoreload_suspend();
    new_status_color(REPL_RUNNING);
    if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
        exit_code = pyexec_raw_repl();
    } else {
        exit_code = pyexec_friendly_repl();
    }
    cleanup_after_vm(heap);
    autoreload_resume();
    return exit_code;
}

void uart_send(uint8_t ch)
{
      //uart_rxtx_write(ch);
      //while (uart_txfull_read());
      //uart_write(ch);
}
void dump_hex(uint8_t *p, size_t len)
{
      uart_send(':');
      for(size_t r = 0; r < len; ++r)
      {
        uart_send("0123456789ABCDEF"[p[r]/16]);
        uart_send("0123456789ABCDEF"[p[r]%16]);
        while (uart_txfull_read());
        if((r & 0x7) == 0x7)
        {
          uart_send(' ');
        }
      }
      uart_send('\n');
}

uint8_t usb_buf[256] TU_ATTR_ALIGNED(4);

tusb_desc_device_t desc_device;

bool print_device_descriptor(uint8_t daddr, tusb_control_request_t const * request, xfer_result_t result)
{
  (void) request;

  if ( XFER_RESULT_SUCCESS != result )
  {
    printf("Failed to get device descriptor\r\n");
    return false;
  }

  printf("Rhport %u Device %u: ID %04x:%04x\r\n", 0, daddr, desc_device.idVendor, desc_device.idProduct);
  printf("Device Descriptor:\r\n");
  printf("  bLength             %u\r\n", desc_device.bLength);
  printf("  bDescriptorType     %u\r\n", desc_device.bDescriptorType);
  printf("  bcdUSB              %04x\r\n", desc_device.bcdUSB);

  printf("  bDeviceClass        %u\r\n", desc_device.bDeviceClass);
  printf("  bDeviceSubClass     %u\r\n", desc_device.bDeviceSubClass);
  printf("  bDeviceProtocol     %u\r\n", desc_device.bDeviceProtocol);
  printf("  bMaxPacketSize0     %u\r\n", desc_device.bMaxPacketSize0);

  printf("  idVendor            0x%04x\r\n", desc_device.idVendor);
  printf("  idProduct           0x%04x\r\n", desc_device.idProduct);
  printf("  bcdDevice           %04x\r\n", desc_device.bcdDevice);

  printf("  iManufacturer       %u\r\n", desc_device.iManufacturer);
  printf("  iProduct            %u\r\n", desc_device.iProduct);
  printf("  iSerialNumber       %u\r\n", desc_device.iSerialNumber);

  printf("  bNumConfigurations  %u\r\n", desc_device.bNumConfigurations);

  return true;
}

int __attribute__((used)) main(void) {
    memory_init();

    // initialise the cpu and peripherals
    safe_mode_t safe_mode = port_init();

    // Turn on LEDs
    init_status_leds();
    rgb_led_status_init();

    // Wait briefly to give a reset window where we'll enter safe mode after the reset.
    if (safe_mode == NO_SAFE_MODE) {
        safe_mode = wait_for_safe_mode_reset();
    }

    stack_init();

#if 1    
    //(void) safe_mode;
    
    board_init();
    uart_init(); //FIXME: needed?
      //static uint32_t xxx = 0x01020304;
    //uart_write('0'+*(uint8_t*)&xxx); //4: little endian
    //new_status_color(0x00FF00);

#define toupper(c) (islower(c) ? (ch) - 'a' + 'A' : (ch))        
#define islower(c) ((c) >= 'a' && (c) <= 'z')    
    tusb_init();

    volatile ohci_registers_t *OHCI_REG = (volatile ohci_registers_t *) USB_OHCI_BASE;
    dump_hex((uint8_t *) OHCI_REG, sizeof(ohci_registers_t));
    dump_hex((uint8_t *) OHCI_REG, sizeof(ohci_registers_t));
    ohci_registers_t init_regs = *OHCI_REG;
    for(;;)
    {
      const int intf = 0; //use 1st interface
      if (tud_cdc_n_available(intf) )
      {

        uint8_t buf[64];
        uint32_t count = tud_cdc_n_read(intf, buf, sizeof(buf));
        for(uint32_t x=0; x < count; ++x)
        {
          uint8_t ch = buf[x];
          tud_cdc_n_write_char(intf, toupper(ch)); //echoes back to cdc, uppercase
          uart_write(ch); //sends to uart unchanged
          tud_hid_mouse_report(2, 0x00, 15, -15, 0, 0); //move (15,-15), 2=REPORT_ID_MOUSE
        }
        tud_cdc_n_write_flush(intf);
      }
      
      tud_task(); //TinyUSB works with interrupts just for queuing packets
      tuh_task();
      
      if(memcmp((void *) &init_regs.interrupt_status, (void *) &OHCI_REG->interrupt_status, sizeof(OHCI_REG->interrupt_status)) != 0)
      {
        dump_hex((uint8_t *) OHCI_REG, sizeof(ohci_registers_t));
        memcpy((void *) &init_regs, (void *) OHCI_REG, sizeof(*OHCI_REG)); 
      }
      (void) init_regs;
   }
    
#else
    // Create a new filesystem only if we're not in a safe mode.
    // A power brownout here could make it appear as if there's
    // no SPI flash filesystem, and we might erase the existing one.
    filesystem_init(safe_mode == NO_SAFE_MODE, false);

    // displays init after filesystem, since they could share the flash SPI
    board_init();

    // Start the debug serial
    serial_early_init();

    // Reset everything and prep MicroPython to run boot.py.
    reset_port();
    reset_board();

    // Turn on autoreload by default but before boot.py in case it wants to change it.
    autoreload_enable();

    // By default our internal flash is readonly to local python code and
    // writable over USB. Set it here so that boot.py can change it.
    filesystem_set_internal_concurrent_write_protection(true);
    filesystem_set_internal_writable_by_usb(true);

    run_boot_py(safe_mode);

    // Start serial and HID after giving boot.py a chance to tweak behavior.
    serial_init();

    #if CIRCUITPY_BLEIO
    supervisor_start_bluetooth();
    #endif

    // Boot script is finished, so now go into REPL/main mode.
    int exit_code = PYEXEC_FORCED_EXIT;
    bool skip_repl = true;
    bool first_run = true;
    for (;;) {
        if (!skip_repl) {
            exit_code = run_repl();
        }
        if (exit_code == PYEXEC_FORCED_EXIT) {
            if (!first_run) {
                serial_write_compressed(translate("soft reboot\n"));
            }
            first_run = false;
            skip_repl = run_code_py(safe_mode);
        } else if (exit_code != 0) {
            break;
        }
    }
    mp_deinit();
#endif    
    return 0;
}

void gc_collect(void) {
    gc_collect_start();

    mp_uint_t regs[10];
    mp_uint_t sp = cpu_get_regs_and_sp(regs);

    // This collects root pointers from the VFS mount table. Some of them may
    // have lost their references in the VM even though they are mounted.
    gc_collect_root((void**)&MP_STATE_VM(vfs_mount_table), sizeof(mp_vfs_mount_t) / sizeof(mp_uint_t));

    #if CIRCUITPY_DISPLAYIO
    displayio_gc_collect();
    #endif

    #if CIRCUITPY_BLEIO
    common_hal_bleio_gc_collect();
    #endif

    // This naively collects all object references from an approximate stack
    // range.
    gc_collect_root((void**)sp, ((uint32_t)port_stack_get_top() - sp) / sizeof(uint32_t));
    gc_collect_end();
}

void NORETURN nlr_jump_fail(void *val) {
    reset_into_safe_mode(MICROPY_NLR_JUMP_FAIL);
    while (true) {}
}

void NORETURN __fatal_error(const char *msg) {
    reset_into_safe_mode(MICROPY_FATAL_ERROR);
    while (true) {}
}

#ifndef NDEBUG
void MP_WEAK __assert_func(const char *file, int line, const char *func, const char *expr) {
    mp_printf(&mp_plat_print, "Assertion '%s' failed, at file %s:%d\n", expr, file, line);
    __fatal_error("Assertion failed");
}
#endif

////////////////////////////////

void tuh_mount_cb(uint8_t dev_addr)
{
  // application set-up
  //printf("A device with address %d is mounted\r\n", dev_addr);
  uart_write('+');
}

void tuh_umount_cb(uint8_t dev_addr)
{
  // application tear-down
  //printf("A device with address %d is unmounted \r\n", dev_addr);
  uart_write('-');
}

void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
  (void)desc_report;
  (void)desc_len;
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);
  uart_write('H');
  uart_write('M');
  //printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);
  //printf("VID = %04x, PID = %04x\r\n", vid, pid);
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  //printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  uart_write('H');
  uart_write('U');
}


// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  uart_write('H');
  uart_write('R');

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, instance) )
  {
    //printf("Error: cannot request to receive report\r\n");
  }
}

//--------------------------------------------------------------------+
// USB CDC
//--------------------------------------------------------------------+
#if CFG_TUH_CDC
CFG_TUSB_MEM_SECTION static char serial_in_buffer[64] = { 0 };

// invoked ISR context
void tuh_cdc_xfer_isr(uint8_t dev_addr, xfer_result_t event, cdc_pipeid_t pipe_id, uint32_t xferred_bytes)
{
  (void) event;
  (void) pipe_id;
  (void) xferred_bytes;

  //printf(serial_in_buffer);
  tu_memclr(serial_in_buffer, sizeof(serial_in_buffer));

  tuh_cdc_receive(dev_addr, serial_in_buffer, sizeof(serial_in_buffer), true); // waiting for next data
  
  uart_write('C');
  uart_write('x');
}

#endif

