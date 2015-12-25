/* 
 * Author: Marcus Junered (marcus.junered@gmail.com)
 *
 * GN3S - GNSS IF Streamer for Windows 
 * Copyright (C) 2008 Marcus Junered 
 * 
 * This program is free software; you can redistribute it and/or modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation; either version 2 of the License, or 
 * (at your option) any later version. 
 * 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * 
 * You should have received a copy of the GNU General Public License 
 * along with this program; if not, write to the Free Software 
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
 */ 
 
 
#ifndef _GN3SFULL_ 
#define _GN3SFULL_ 

#define DEBUG 0

// Constants
#define MEGABYTE 1048576.0f

// FX2 stuff
#define VID 0x16c0
#define PID 0x072f

#define RX_ENDPOINT 0x86
 
#define VRT_VENDOR_IN  0xC0 
#define VRT_VENDOR_OUT 0x40 
 
#define RX_INTERFACE    2 
#define RX_ALTINTERFACE 0 
 
// IN commands 
#define VRQ_GET_STATUS 0x80 
 
#define GS_RX_OVERRUN 1   // wIndexL  // returns 1 byte 
 
// OUT commands 
#define VRQ_XFER 0x01 

struct fx2Config{
  int interface; 
  int altinterface; 
  usb_dev_handle *udev; 
  fusb_ephandle *d_ephandle; 
  fusb_devhandle *d_devhandle; 
};

class GN3SFull {
 protected:
  // Flags
  uint32_t debug;

  // Common variables
  int64_t ret;
  uint64_t numXfer;
  float bufSizeDiv;
  const static float FS = 16367667.0f;

  // FX2 Stuff
  struct usb_device *fx2;
  fx2Config fx2c;
  uint32_t vid, pid;
  const static uint32_t bufSize = 32 * 512; // 16 kB

  // Data stream variables
  int8_t *tempBuf8;
  int16_t *tempBuf16;

  // FUSB Constants 
  const static int FUSB_BUFFER_SIZE = 16 * (1L << 20); // 8 MB
  const static int FUSB_BLOCK_SIZE = 16 * (1L << 10); // 16KB is hard limit 
  const static int FUSB_NBLOCKS = FUSB_BUFFER_SIZE / FUSB_BLOCK_SIZE;

  // Internal Functions
  void print_endpoint(struct usb_endpoint_descriptor *endpoint); 
  void print_altsetting(struct usb_interface_descriptor *interface); 
  void print_interface(struct usb_interface *interface); 
  void print_configuration(struct usb_config_descriptor *config); 
  int write_cmd (struct usb_dev_handle *udh, int request, int value,  
		 int index, unsigned char *bytes, int len); 
  bool _get_status (struct usb_dev_handle *udh, int which, bool *trouble); 
  bool check_rx_overrun (struct usb_dev_handle *udh, bool *overrun_p); 
  bool usrp_xfer (struct usb_dev_handle *udh, char VRQ_TYPE, bool start); 
  fusb_devhandle *make_devhandle (usb_dev_handle *udh); 
  struct usb_device *usb_fx2_find(int vid, int pid, char info);
  bool usb_fx2_configure(struct usb_device *fx2, fx2Config *fx2c); 

 public:
  GN3SFull(uint32_t vid_in, uint32_t pid_in);
  ~GN3SFull();
  uint64_t read(char *filename, uint32_t sec, uint32_t config);
  uint64_t read16(char *filename, uint32_t sec, uint32_t config);
};

#endif
