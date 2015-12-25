/* 
 * Author: Marcus Junered (marcus.junered@gmail.com)
 *
 * GN3S - GNSS IF Streamer for Linux
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
 
using namespace std;

#include <iostream>
#include <ctime>
#include <cmath>
#include <cstring>

#include <usb.h>
#include <errno.h> 
#include <getopt.h> 
#include <pthread.h>
#include <inttypes.h>
#include <cstring>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/un.h>

#include "fusb.h" 
#include "fusb_linux.h" 
#include "usrp_bytesex.h" 
#include "usrp_prims.h" 
#include "TimeStuff.h"
#include "GN3SFull.h"
int tcp_port;
int sock_flag = 0;

/*
 * Test function showing the usage of GN3SFull. Runtime options are selected 
 * by passing arguments to the executable.
 */
int main(int argc, char *argv[])
{ 
  // Default options;
  char *filename = "data.bin";
  uint32_t vid = VID;
  uint32_t pid = PID;
  uint32_t sec = 1;
  uint32_t config = 0x0;
  uint32_t debug = DEBUG;

  // Parse arguments
  int32_t ch; 
  while ((ch = getopt (argc, argv, "?c:p:d:o:s:t:u")) != EOF) { 
    switch (ch) { 

    case 'o': 
      filename = optarg; 
      if (debug >= 2) 
	printf("filename: %s \n", filename); 
      break; 
 
    case 's': 
      sscanf(optarg, "%u", &sec);
      if (debug >= 2) 
	printf("numMb: %u \n", sec);
      break;
 
    case 'c': 
      sscanf(optarg, "%x", &config);
      if (debug >= 2) 
	printf("config: 0x%x \n", config);
      break; 
 
    case 'd':
      sscanf(optarg, "%x", &vid);
      if (debug >= 2)
	printf("vid: 0x%x \n", vid);
      break;

    case 'p':
      sscanf(optarg, "%x", &pid);
      if (debug >= 2)
	printf("pid: 0x%x \n", pid);
      break;

    case 't':
      sscanf(optarg, "%d", &tcp_port);
      sock_flag = 1;
      break;

    case 'u':
      sock_flag = 1;
      break;

      // Print HELP
    case '?': 
      printf("Usage:\n"); 
      printf("-o filename : Output file name \n"); 
      printf("-s seconds  : Number of seconds to collect \n"); 
      printf("-c config   : Config: \"0xDPSM\" \n");
      printf("                      M set = stream to memory \n");
      printf("                      S set = subsample (factor of 3) \n");
      printf("                      P set = pack (8 or 4 samples / byte) \n");
      printf("                      D set = decimate (1 bit instead of 2) \n");
      printf("-d vid      : Vendor ID \n");
      printf("-p pid      : Product ID \n");
      return -1; 
      break; 
 
    default: 
      break; 
    } 
  } 

  // Create a GN3S object associated with the specified device (VID/PID)
  GN3SFull gn3s(vid, pid);

  double start_wall_time = get_elapsed_time ();
  double start_cpu_time  = get_cpu_usage ();

  // Read "numMB" megabytes to "filename" using "config"
  uint64_t numRead = gn3s.read(filename, sec, config);

  // Calculate elapsed time and transfer rate
  double stop_wall_time = get_elapsed_time ();
  double stop_cpu_time  = get_cpu_usage ();
  double delta_wall = stop_wall_time - start_wall_time;
  double delta_cpu  = stop_cpu_time  - start_cpu_time;

  printf("numRead: %lld \n", numRead);

  float numMB = (float) numRead / MEGABYTE;
  printf ("xfered %.1f MB in %.2f seconds. %.1f MB/s. CPU usage = %d%% \n",
	  numMB,
	  delta_wall, 
	  (double) numMB / delta_wall,
	  (uint32_t) (delta_cpu*100 / delta_wall + 0.5f)
	  );
}

/*
 * Initializes temporary buffers and connects to the USB device
 */
GN3SFull::GN3SFull(uint32_t vid_in, uint32_t pid_in) {

  debug = DEBUG;
  vid = vid_in;
  pid = pid_in;

  // Temporary buffers
  tempBuf8 = new int8_t[bufSize*6];
  tempBuf16 = (int16_t*) tempBuf8;

  // Common variables
  ret = 0;
  numXfer = 0;
  bufSizeDiv = 1.0f / (float) bufSize;

  // Search all USB busses for the device specified by VID/PID
  fx2 = usb_fx2_find(vid, pid, debug);
  if (!fx2) { 
    fprintf(stderr, "Could not find a FX2 device \n"); 
    exit(-1);
  }

  // Open and configure FX2 device if found... 
  ret = usb_fx2_configure(fx2, &fx2c); 
  if (ret) { 
    fprintf(stderr, "Could not obtain a handle to the FX2 device \n"); 
    exit(-1);
  }
}

/*
 * Destructor
 */
GN3SFull::~GN3SFull() {
  delete fx2c.d_ephandle; 
  delete fx2c.d_devhandle; 

  usb_release_interface(fx2c.udev, fx2c.interface); 
  usb_close(fx2c.udev); 
}

/*
 * The read funtion streams "sec" seconds of IF data to the specified file. 
 * Data is processed in real-time according to "config":
 *
 *   0x"DPSM" where each letter represents one bit
 *     M set (0x1) = stream to memory then dump to disk 
 *     S set (0x2) = subsample by a factor of 3
 *     P set (0x4) = pack samples into bytes (4 or 8)
 *     D set (0x8) = decimate from 2 bits to 1 bit
 */
uint64_t GN3SFull::read(char *filename, uint32_t sec, uint32_t config) {

  int8_t LUT_SM[] = {1,-1,3,-3};
  int8_t LUT_Flip1[] = {0,1};
  int8_t LUT_Flip2[] = {0,2,1,3};
  int8_t *LUT;

  uint32_t mem = config & 0x1;
  uint32_t subSample = config & 0x2;
  uint32_t pack = config & 0x4;
  uint32_t decimate = config & 0x8;

  bool overRun = false; 
  bool loopCond = true;

  FILE *outputFile=NULL;

  if (sock_flag) { /* Socket */
    int sock_listen;
    sockaddr *sp = NULL;
    socklen_t slen = 0;
    struct sockaddr_un sun;
    struct sockaddr_in sin;

    if (tcp_port) { /* IP Socket */
      int on = 1;
      sin.sin_family = PF_INET;
      sin.sin_port = htons(tcp_port);
      sin.sin_addr.s_addr = htonl(INADDR_ANY);
      if ( (sock_listen = socket (PF_INET, SOCK_STREAM, 0)) < 0) {
	perror ("Unable to open socket");
	exit (EXIT_FAILURE);
      }
      if (setsockopt (sock_listen, SOL_SOCKET, SO_REUSEADDR,
		      &on, sizeof(int)) < 0) {
	perror ("Unable set socket option");
	exit (EXIT_FAILURE);
      }
      sp = (sockaddr *) &sin;
      slen = sizeof(struct sockaddr_in);
    }
    else { /* UNIX Domain socket */
      if ( (sock_listen = socket (PF_UNIX, SOCK_STREAM, 0)) < 0) {
	perror ("Unable to open socket");
	exit (EXIT_FAILURE);
      }

      /* In case it already exists */
      if ((unlink (filename) < 0) && (errno != ENOENT)) {
	fprintf (stderr, "Unable to unlink %s: %s\n", filename,
		 strerror(errno));
	exit (EXIT_FAILURE);
      }
      
      memset (&sun, 0, sizeof(sun));
      sun.sun_family = PF_UNIX;
      strcpy (sun.sun_path, filename);
      slen = sizeof(sun.sun_family) + strlen(sun.sun_path);
      sp = (sockaddr *) &sun;
    }
   
    if (bind (sock_listen, sp, slen) < 0) {
      perror ("Unable to bind listen socket");
      exit (EXIT_FAILURE);
    }
      
    if (listen (sock_listen, 5) < 0) {
      perror ("Unable to listen on socket");
      exit (EXIT_FAILURE);
    }
    
    int sock_out;
    struct sockaddr addr;
    socklen_t addrlen=sizeof(addr);

    if ((sock_out = accept (sock_listen, &addr, &addrlen)) < 0) {
      perror ("Unable to accept on socket");
      exit (EXIT_FAILURE);
    }
    
    outputFile = fdopen (sock_out, "wb");
  }
  else {
    outputFile = fopen64(filename, "wb");
    if (outputFile == NULL) {
      perror("GN3SFull: fopen");
      exit(-1);
    }
  }

  // Calculate number of loops and samples
  uint32_t loops = 1 + (uint32_t) (sec * FS * bufSizeDiv / 3);
  uint64_t numSamp = (uint64_t) loops * bufSize;
  uint64_t numRead = 0;

  // Calculate new sample count for memory allocation
  switch (config & 0xe) { // 0x"DPS"

    // All options, 3 * 8
  case 0xe:
    numSamp /= 24;
    break;
   
    // Decimate and Pack, 2 * 4
  case 0xc:
    numSamp /= 8;
    break;

    // Pack and Subsample, 4 * 3
  case 0x6:
    numSamp /= 12;
    break;
    
    // Decimate and Subsample, 3
  case 0xa:
    numSamp /= 3;
    break;

    // Subsample, 3
  case 0x2:
    numSamp /= 3;
    break;
    
    // Pack
  case 0x4:
    numSamp /= 4;
    break;
  }
  
  // Increase by one bufSize to compensate for fixed-point division
  numSamp += bufSize;

  // If streaming to memory, allocate BIG buffer
  int8_t *bigBuf8 = NULL;
  if (mem) {
    bigBuf8 = new int8_t[numSamp];

    // Check is memory allocation was succesful
    if (bigBuf8 == NULL) {
      perror("GN3SFull - new: bigBuf");
      exit(-1);
    }
  }

  // 
  // Start streaming...
  //

  // Start Waveform
  usrp_xfer (fx2c.udev, VRQ_XFER, 1); 

  // Loop until we have enough data...
  uint32_t j = 0;
  uint32_t start = 0;
  uint32_t lidx = 0;
  while(loopCond) {

    if (debug >= 3) 
      printf("Loop: %d \n", lidx); 

    // Check for buffer overrun 
    check_rx_overrun(fx2c.udev, &overRun); 
    if (overRun) { 
      fprintf(stderr, "Buffer overrun... \n"); 
      break;
    }  
    else if (debug >= 3) { 
      printf("No overrun detected \n"); 
    } 

    // FUSB Read... 
    ret  = fx2c.d_ephandle->read(tempBuf8, bufSize);
    ret += fx2c.d_ephandle->read(tempBuf8+bufSize, bufSize);
    ret += fx2c.d_ephandle->read(tempBuf8+2*bufSize, bufSize);
    if ((uint32_t) ret != 3*bufSize) {
      fprintf (stderr, 
	       "read - fusb_read: ret = %lld (bufsize: %d) \n", 
	       ret, bufSize);
      fprintf (stderr, 
	       "%s\n", 
	       usb_strerror()); 
    }
    else if (debug >=3) { 
      printf("Received %lld bytes of data \n", ret); 
    }

    // Decide which LUT to use...
    // Flip bits if packing, otherwise flip and interpret (Sgn-Mag)
    if (decimate > 0) {
      LUT = (pack > 0) ? LUT_Flip1 : LUT_SM;
    }
    else {
      LUT = (pack > 0) ? LUT_Flip2 : LUT_SM;
    }

    // Decide which bit mask to use (1 or 2 bits)
    uint8_t bitMask = (decimate > 0) ? 0x1 : 0x3;

    uint32_t idx = 0;
    uint32_t kidx = 0;
  
    // No subsampling
    if (subSample == 0) {
      for (j=0; j<3*bufSize; j++) {
	// Use LUT to rearrange and interpret bits (SGN-MAG) 
	tempBuf8[j] = LUT[tempBuf8[j] & bitMask]; 
      }
      idx = 3*bufSize;
    }
    // Subsample by a factor of 3
    else { 
      for (j=start; j<3*bufSize; j+=3) {
	// Use LUT to rearrange and interpret bits (SGN-MAG) 
	tempBuf8[idx] = LUT[tempBuf8[j] & bitMask]; 
	idx++;
      }
      start = 3 - (3*bufSize - (j-3));
    }

    // Pack samples
    if (pack > 0) {
      
      // 1 bit format -> 8 samples per byte (7 downto 0)
      if (decimate > 0) {
	for (uint32_t k=0; k<idx; k+=8) {
	  tempBuf8[kidx] = ( tempBuf8[k+7] << 7 |
			     tempBuf8[k+6] << 6 |
			     tempBuf8[k+5] << 5 |
			     tempBuf8[k+4] << 4 |
			     tempBuf8[k+3] << 3 |
			     tempBuf8[k+2] << 2 |
			     tempBuf8[k+1] << 1 |
			     tempBuf8[k] );
	  kidx++;
	}
      }
      // 2 bit format -> 4 samples per byte (3 downto 0)
      else {
	for (uint32_t k=0; k<idx; k+=4) {
	  tempBuf8[kidx] = ( tempBuf8[k+3] << 6 |
			     tempBuf8[k+2] << 4 |
			     tempBuf8[k+1] << 2 |
			     tempBuf8[k] );
	  kidx++;
	}
      }
      idx = kidx;
    }

    // If streaming to memory, copy to buffer 
    if (mem) {
      memcpy(&bigBuf8[numRead], tempBuf8, sizeof(int8_t) * idx);
    }
    // otherwise write to disk
    else {
      numXfer = fwrite(tempBuf8, sizeof(int8_t), idx, outputFile);
      
      // Check if write was succesful
      if (numXfer != idx) { 
	perror ("read - fwrite"); 
	fprintf(stderr, "Could only write %lld of %d IF samples \n", 
		numXfer, idx); 
      }
    }
    numRead += idx; // Increment pointer for BIG buffer

    lidx++;
    if (lidx == loops) {
      loopCond = false;
    }
  } // End while loop
  
  // Stop waveform 
  usrp_xfer (fx2c.udev, VRQ_XFER, 0);

  // Dump buffer to disk 
  if (mem) {
    numXfer = fwrite (bigBuf8, sizeof(int8_t), numRead, outputFile);
    
    // Check if write was succesful
    if (numXfer != numRead) {
      perror ("read - fwrite");
      fprintf(stderr, "Could only write %lld of %lld IF samples \n", 
	      numXfer, numRead); 
    }
  }

  // Clean up
  fclose(outputFile);

  delete [] tempBuf8;
  delete [] bigBuf8;

  return numRead;
}

/* 
 * Same as the read function except output format is int16_t instead
 * (useful for MMX/SSE processing where 16bit data formats are used).
 */
uint64_t GN3SFull::read16(char *filename, uint32_t sec, uint32_t config) {

  int16_t LUT[] = {1,-1,3,-3};

  uint32_t mem = config & 0x1;
  uint32_t subSample = config & 0x2;
  uint32_t pack = config & 0x4;
  uint32_t decimate = config & 0x8;
  
  bool overRun = false; 
  bool loopCond = true;

  if (pack > 0) {
    printf("Packing option not supported, use \"read\" instead! \n");
  }

  FILE *outputFile = fopen64(filename, "wb");
  if (outputFile == NULL) {
    perror("GN3SFull: fopen");
    exit(-1);
  }

  // Calculate number of loops and samples
  uint32_t loops = 1 + (uint32_t) (sec * FS * bufSizeDiv);
  uint64_t numSamp = (uint64_t) loops * bufSize;
  uint64_t numRead = 0;
  
  // Calculate new sample count
  if (subSample > 0) {
    numSamp /= 3;
    
    // Increase by one bufSize to compensate for fixed-point division
    numSamp += bufSize;
  }

  // If streaming to memory, allocate BIG buffer
  int16_t *bigBuf16 = NULL;
  if (mem) {
    bigBuf16 = new int16_t[numSamp];

    // Check is memory allocation was succesful
    if (bigBuf16 == NULL) {
      perror("GN3SFull - new: bigBuf");
      exit(-1);
    }
  }

  // 
  // Start streaming...
  //

  // Start Waveform
  usrp_xfer (fx2c.udev, VRQ_XFER, 1); 

  // Loop until we have enough data...
  uint32_t j = 0;
  uint32_t start = 0;
  uint32_t lidx = 0;
  while(loopCond) {

    if (debug >= 3) 
      printf("Loop: %d \n", lidx); 

    // Check for buffer overrun 
    check_rx_overrun(fx2c.udev, &overRun); 
    if (overRun) { 
      fprintf(stderr, "Buffer overrun... \n"); 
      break;
    }  
    else if (debug >= 3) { 
      printf("No overrun detected \n"); 
    } 

    // FUSB Read... 
    ret = fx2c.d_ephandle->read(tempBuf8, bufSize);
    if ((uint32_t) ret != bufSize) {
      fprintf (stderr, 
	       "read - fusb_read: ret = %lld (bufsize: %d) \n", 
	       ret, bufSize);
      fprintf (stderr, 
	       "%s\n", 
	       usb_strerror()); 
    }
    else if (debug >=3) { 
      printf("Received %lld bytes of data \n", ret); 
    }

    // Decide which bit mask to use (1 or 2 bits)
    uint8_t bitMask = (decimate > 0) ? 0x1 : 0x3;

    // Store IF data according to "config"
      
    uint32_t idx = 0;

    // No subsampling
    if (subSample == 0) {
      for (j=0; j<bufSize; j++) {
	// Use LUT to rearrange and interpret bits (SGN-MAG) 
	tempBuf16[j] = LUT[tempBuf16[j] & bitMask]; 
      }
      idx = bufSize;
    }
    // Subsample by a factor of 3
    else {
      for (j=start; j<bufSize; j+=3) {
	// Use LUT to rearrange and interpret bits (SGN-MAG) 
	tempBuf16[idx] = LUT[tempBuf16[j] & bitMask]; 
	idx++;
      }
      start = 3 - (bufSize - (j-3));
    }

    // If streaming to memory, copy to buffer 
    if (mem) {
      memcpy(&bigBuf16[numRead], tempBuf16, sizeof(int16_t) * idx);
    }
    // otherwise write to disk
    else {
      numXfer = fwrite(tempBuf16, sizeof(int16_t), idx, outputFile);
      
      // Check if write was succesful
      if (numXfer != idx) { 
	perror ("read - fwrite"); 
	fprintf(stderr, "Could only write %lld of %d IF samples \n", 
		numXfer, idx); 
      }
    }
    numRead += idx; // Increment pointer for BIG buffer

    lidx++;
    if (lidx == loops) {
      loopCond = false;
    }
  } // End while loop
  
  // Stop waveform 
  usrp_xfer (fx2c.udev, VRQ_XFER, 0);

  // Dump buffer to disk 
  if (mem) {
    numXfer = fwrite (bigBuf16, sizeof(int16_t), numRead, outputFile);
    
    // Check if write was succesful
    if (numXfer != numRead) {
      perror ("read - fwrite");
      fprintf(stderr, "Could only write %lld of %lld IF samples \n", 
	      numXfer, numRead); 
    }
  }

  // Clean up
  fclose(outputFile);

  delete [] tempBuf16;
  delete [] bigBuf16;

  return numRead;
}


void GN3SFull::print_endpoint(struct usb_endpoint_descriptor *endpoint) { 
  printf(" bEndpointAddress: %02xh\n", endpoint->bEndpointAddress); 
  printf(" bmAttributes: %02xh\n", endpoint->bmAttributes); 
  printf(" wMaxPacketSize: %d\n", endpoint->wMaxPacketSize); 
  printf(" bInterval: %d\n", endpoint->bInterval); 
  printf(" bRefresh: %d\n", endpoint->bRefresh); 
  printf(" bSynchAddress: %d\n", endpoint->bSynchAddress); 
} 
 
 
void GN3SFull::print_altsetting(struct usb_interface_descriptor *interface) { 
  int i; 
 
  printf(" bInterfaceNumber: %d\n", interface->bInterfaceNumber); 
  printf(" bAlternateSetting: %d\n", interface->bAlternateSetting); 
  printf(" bNumEndpoints: %d\n", interface->bNumEndpoints); 
  printf(" bInterfaceClass: %d\n", interface->bInterfaceClass); 
  printf(" bInterfaceSubClass: %d\n", interface->bInterfaceSubClass); 
  printf(" bInterfaceProtocol: %d\n", interface->bInterfaceProtocol); 
  printf(" iInterface: %d\n", interface->iInterface); 
 
  for (i = 0; i < interface->bNumEndpoints; i++) 
	print_endpoint(&interface->endpoint[i]); 
} 
 
 
void GN3SFull::print_interface(struct usb_interface *interface) { 
  int i; 
 
  for (i = 0; i < interface->num_altsetting; i++) 
	print_altsetting(&interface->altsetting[i]); 
} 
 
 
void GN3SFull::print_configuration(struct usb_config_descriptor *config) { 
  int i; 
 
  printf(" wTotalLength: %d\n", config->wTotalLength); 
  printf(" bNumInterfaces: %d\n", config->bNumInterfaces); 
  printf(" bConfigurationValue: %d\n", config->bConfigurationValue); 
  printf(" iConfiguration: %d\n", config->iConfiguration); 
  printf(" bmAttributes: %02xh\n", config->bmAttributes); 
  printf(" MaxPower: %d\n", config->MaxPower); 
 
  for (i = 0; i < config->bNumInterfaces; i++) 
	print_interface(&config->interface[i]); 
} 
 
 
int GN3SFull::write_cmd (struct usb_dev_handle *udh, int request, int value,  
		      int index, unsigned char *bytes, int len) 
{ 
  // int r = write_cmd (udh, VRQ_XFER, start, 0, 0, 0); 
  int requesttype = (request & 0x80) ? VRT_VENDOR_IN : VRT_VENDOR_OUT;  
  int r = usb_control_msg (udh, requesttype, request, value, index,  
						   (char *) bytes, len, 1000); 
  if (r < 0){ 
	// we get EPIPE if the firmware stalls the endpoint. 
	if (errno != EPIPE) 
	  fprintf (stderr, "usb_control_msg failed: %s\n", usb_strerror ()); 
  } 
  return r; 
} 
 

bool GN3SFull::_get_status (struct usb_dev_handle *udh, int which, bool *trouble) { 
  unsigned char status; 
  *trouble = true; 
 
  if (write_cmd (udh, VRQ_GET_STATUS, 0, which, 
				 &status, sizeof (status)) != sizeof (status)) 
	return false; 
 
  *trouble = status; 
  return true; 
} 
 
 
bool GN3SFull::check_rx_overrun (struct usb_dev_handle *udh, bool *overrun_p) { 
  return _get_status (udh, GS_RX_OVERRUN, overrun_p); 
} 
 
 
bool GN3SFull::usrp_xfer (struct usb_dev_handle *udh, char VRQ_TYPE, bool start) {    
  int r = write_cmd (udh, VRQ_TYPE, start, 0, 0, 0); 
  return r == 0; 
} 

 
fusb_devhandle* GN3SFull::make_devhandle (usb_dev_handle *udh) { 
  return new fusb_devhandle_linux (udh); 
} 
 
 
struct usb_device* GN3SFull::usb_fx2_find(int vid, int pid, char info) {
 
  struct usb_bus *bus; 
  struct usb_device *dev; 
  struct usb_device *fx2 = NULL; 
  usb_dev_handle *udev; 
 
  int ret; 
  char str[256];

  usb_init(); 
  usb_find_busses(); 
  usb_find_devices(); 
 
  if (debug >= 2) 
	printf("bus/device idVendor/idProduct\n");  
 
  for (bus = usb_busses; bus; bus = bus->next) { 
    for (dev = bus->devices; dev; dev = dev->next) { 
	   
      if (debug >= 2) 
	printf("%s/%s %04X/%04X\n", bus->dirname, dev->filename, 
	       dev->descriptor.idVendor, dev->descriptor.idProduct); 
 
      /*
      if (debug >= 1) { 
	printf("filename: %s \n", dev->filename); 
      } 

      printf("%s/%s %04X/%04X\n", 
	     bus->dirname, 
	     dev->filename,
	     dev->descriptor.idVendor, 
	     dev->descriptor.idProduct);
      */

      if (dev->descriptor.idVendor == vid && 
	  dev->descriptor.idProduct == pid) {
	/*
	printf("vid: %d, pid: %d \n", 
	       dev->descriptor.idVendor, 
	       dev->descriptor.idProduct);
	*/
	
	fx2 = dev;
      }

      if (fx2 != NULL && info) { 
 
	udev = usb_open(fx2); 
	if (udev) { 
	  if (fx2->descriptor.iManufacturer) { 
	    ret = usb_get_string_simple(udev, 
					fx2->descriptor.iManufacturer, 
					str,  
					sizeof(str)); 
 
	    if (ret > 0) 
	      printf("- Manufacturer : %s\n", str); 
	    else 
	      printf("- Unable to fetch manufacturer string\n"); 
	  } 
 
	  if (fx2->descriptor.iProduct) { 
	    ret = usb_get_string_simple(udev, 
					fx2->descriptor.iProduct,  
					str,  
					sizeof(str)); 
 
	    if (ret > 0) 
	      printf("- Product : %s\n", str); 
	    else 
	      printf("- Unable to fetch product string\n"); 
	  } 
 
	  if (fx2->descriptor.iSerialNumber) { 
	    ret = usb_get_string_simple(udev, 
					fx2->descriptor.iSerialNumber, 
					str, 
					sizeof(str)); 
	     
	    if (ret > 0) 
	      printf("- Serial Number: %s\n", str); 
	    else 
	      printf("- Unable to fetch serial number string\n"); 
	  } 
	   
	  usb_close (udev); 
	} 
	 
	if (!fx2->config) { 
	  printf(" Could not retrieve descriptors\n"); 
	  continue; 
	} 
	 
	for (int i = 0; i < fx2->descriptor.bNumConfigurations; i++) { 
	  print_configuration(&fx2->config[i]); 
	} 
      } 
    } 
  }	 
   
  return fx2; 
} 
 
 
bool GN3SFull::usb_fx2_configure(struct usb_device *fx2, fx2Config *fx2c) {  
   
  char status = 0; 
  int interface = RX_INTERFACE; 
  int altinterface = RX_ALTINTERFACE; 
  usb_dev_handle *udev; 
  fusb_ephandle *d_ephandle; 
  fusb_devhandle *d_devhandle; 
   
  udev = usb_open(fx2); 
   
  if (!udev) { 
    fprintf(stderr, "Could not obtain a handle to GNSS Front-End device \n"); 
    return -1; 
  } 
  else { 
 
    if (debug >= 1) 
      printf("Received handle for GNSS Front-End device \n"); 
 
    if (usb_set_configuration (udev, 1) < 0) { 
      fprintf (stderr,  
	       "error in %s, \n%s \n", 
	       __FUNCTION__, 
	       usb_strerror()); 
      usb_close (udev); 
      status = -1; 
    } 
 
    if (usb_claim_interface (udev, interface) < 0) { 
      fprintf (stderr,  
	       "error in %s, \n%s \n", 
	       __FUNCTION__, 
	       usb_strerror()); 
      usb_close (udev); 
      fprintf (stderr, "\nDevice not programmed? \n");  
      usb_close (udev);  
      status = -1;  
      exit(0);  
    } 
 
    if (usb_set_altinterface (udev, altinterface) < 0) { 
      fprintf (stderr,  
	       "error in %s, \n%s \n", 
	       __FUNCTION__, 
	       usb_strerror()); 
      usb_close (udev); 
      usb_release_interface (udev, interface); 
      usb_close (udev); 
      status = -1; 
    } 
 
    d_devhandle=make_devhandle(udev); 
    d_ephandle = d_devhandle->make_ephandle (RX_ENDPOINT,  
					     true, 
					     FUSB_BLOCK_SIZE,  
					     FUSB_NBLOCKS); 
 
    if (!d_ephandle->start ()){ 
      fprintf (stderr, "usrp0_rx: failed to start end point streaming"); 
      usb_strerror (); 
      status = -1; 
    } 
 
    if (status == 0) { 
      fx2c->interface = interface; 
      fx2c->altinterface = altinterface; 
      fx2c->udev = udev; 
      fx2c->d_devhandle = d_devhandle; 
      fx2c->d_ephandle = d_ephandle; 
       
      return 0; 
    } 
    else { 
      return -1; 
    } 
  } 
} 
