/* 
 * File originates from: fx2-programmer-2.0.2 
 *  
 * Modifications for use with the SiGe USB module to accompany 
 * the textbook: "A Software-Defined GPS and Galileo Receiver: A 
 * Single-Frequency Approach" by Kai Borre, Dennis Akos, et.al. by: 
 * 
 * Marcus Junered, GNSS Research Group 
 * Lulea University of Technology 
 * Lulea, Sweden  
 * junered@ltu.se 
 * 
 * --------------------------------------------------------------------- 
 * 
 * GN3S - GNSS IF Streamer for Windows 
 * Copyright (C) 2006 Marcus Junered 
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
 
 
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <string.h>
#include <math.h>
/* needed for stat() */
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
/* needed for usb functions */
#include <usb.h>


static char debug = 1; // 1 = Verbose
static char *serial_filename = "serial.txt";
//static int serial_addr = 0x1201;

struct usb_device *current_device;
usb_dev_handle *current_handle;


int atoz(char *s)
{
  int a;
  if(!strncasecmp("0x", s, 2)){
    sscanf(s, "%x", &a);
    return a;
  }
  return atoi(s);
}


void dump_busses(void)
{
  struct usb_bus *p;
  struct usb_device *q;
  p=usb_busses;
  printf("Dump of USB subsystem:\n");
  
  while(p!=NULL) {
	q=p->devices;
	while(q!=NULL) {
	  printf(" bus %s device %s vendor id=0x%04x product id=0x%04x %s\n",
		 p->dirname, q->filename, q->descriptor.idVendor, 
		 q->descriptor.idProduct,
		 (q->descriptor.idVendor==0x4b4) && (q->descriptor.idProduct==0x8613)?
		 "(UNCONFIGURED FX2)":"");
	  q=q->next;
	}
	p=p->next;
  }
  fflush(stdout);
}


int str2num(char *str, int length) {
  int i;
  int result = 0;

  for (i=0; i<length; i++) {
    result += (str[i]-48) * (int) pow(10,length-i-1);
  }

  return result;
}


void num2str(int num_arg, int length, char *str) {
  int i;
  int num = num_arg;
  int ten_pow = 0;

  for (i=0; i<length; i++) {
    if (num != 0) {
      ten_pow = (int) pow(10, length-i-1);      
      str[i] = (num / ten_pow) + 48;
      num -= (num / ten_pow) * ten_pow;
    }
    else {
      str[i] = 48;
    }
  }
}


void serialize_fw(char *fw_file, char *serial_file) {
  FILE *fid_serial, *fid_fw;
  char vec_len = 5;
  char num_wr = 0;
  char num_rd = 0;
  char serial_vec[6];
  char serial_vec_zero[11];
  int serial = 0;
  int i = 0;

  // Terminate "string"
  serial_vec[5] = 0;
  serial_vec[10] = 0;

  // Open files...
  fid_serial = fopen(serial_file, "r+b");  
  if (fid_serial == NULL) {
    perror("fopen");
    printf("The file serial.txt is missing! See README for details \n");
    exit(0);
  }

  fid_fw = fopen(fw_file, "r+b");   
  if (fid_fw == NULL) {
    fclose(fid_serial);
    perror("fopen");
    exit(0);
  }
    
  // Read current serial number from file "serial.txt"
  num_rd = fread(serial_vec, sizeof(char), vec_len, fid_serial);
  if (num_rd != vec_len) {
    fprintf(stderr, "Could only read %d of %d bytes... \n", num_rd, vec_len);
    fclose(fid_serial);
    fclose(fid_fw);
    exit(0);
  }

  if (debug >= 2) {
    for (i=0; i<vec_len; i++) {
      printf("serial_vec[%d]: %d \n", i, serial_vec[i]);
    }
  }

  // Convert serial string to integer
  serial = str2num(serial_vec, vec_len);
  
  if (debug >= 2)
    printf("Serial: %d \n", serial);
  
  // Convert serial to "zero" string
  num2str(serial, vec_len, serial_vec);
  for (i=0; i<vec_len; i++) {
    serial_vec_zero[i*2] = serial_vec[i];
    serial_vec_zero[i*2+1] = 0;
  }

  // Seek position of serial number in firmware
  //fseek(fid_fw, serial_addr, SEEK_SET);
  fseek(fid_fw, -15, SEEK_END);

  // Write new serial to firmware
  num_wr = fwrite(serial_vec_zero, sizeof(char), vec_len*2, fid_fw);
  if (num_wr != vec_len*2) {
    fprintf(stderr, "Could only write %d of %d bytes... \n", num_rd, vec_len*2);
    fclose(fid_serial);
    fclose(fid_fw);
    exit(0);
  }

  printf("Wrote serial: \"%s\" to file \"%s\" \n", serial_vec, fw_file);
 
  // Increment serial number
  serial++;

  if (serial > 99999) {
    serial = 10000;
  }

  // Convert back to string
  num2str(serial, vec_len, serial_vec);

  if (debug >= 2) {
    for (i=0; i<vec_len*2; i++) {
      printf("serial_vec_zero[%d]: %d \n", i, serial_vec_zero[i]);
    }
  }

  // Write new serial back to "serial.txt"
  fseek(fid_serial, 0, SEEK_SET);
  num_wr = fwrite(serial_vec, sizeof(char), vec_len, fid_serial);
  if (num_wr != vec_len) {
    fprintf(stderr, "Could only write %d of %d bytes... \n", num_rd, vec_len);
    fclose(fid_serial);
    fclose(fid_fw);
    exit(0);
  }

  // Clean up
  fclose(fid_serial);
  fclose(fid_fw);
}


void upload_ram(char *buf, int start, int len)
{
  int i;
  int tlen;
  int quanta=16;
  int a;
  
  for(i=start;i<start+len;i+=quanta) {
    tlen=len+start-i;
    
    if(tlen>quanta)
      tlen=quanta;

    if (debug >= 3)
      printf("i = %d, tlen = %d \n", i, tlen);
    a=usb_control_msg(current_handle, 0x40, 0xa0, 
		      i, 0, buf+(i-start), tlen, 1000);
    
    if(a<0) {
      fprintf(stderr,"Request to upload ram contents failed: %s\n",
	      usb_strerror());
      return;
    }
  }
}


struct usb_device *find_device(char *busname, char *devicename)
{
struct usb_bus *p;
struct usb_device *q;
p=usb_busses;
while(p!=NULL){
	q=p->devices;
	if(strcmp(p->dirname, busname)){
		p=p->next;
		continue;
		}
	while(q!=NULL){
		if(!strcmp(q->filename, devicename))return q;
		q=q->next;
		}
	p=p->next;
	}
return NULL;
}


void program_fx2(char *filename, char mem)
{
  FILE *f;
  char s[1024];
  int length;
  int addr;
  int type;
  char data[256];
  char checksum,a;
  unsigned int b;
  int i;

  f=fopen(filename, "r");

  if(f==NULL){
    fprintf(stderr,"Cannot open file \"%s\" for reading:", filename);
    perror("");
    return;
  }

  printf("Using file \"%s\"\n", filename);

  while(!feof(f)) {
    fgets(s, 1024, f); /* we should not use more than 263 bytes normally */

    if(s[0]!=':'){
      fprintf(stderr,"%s: invalid string: \"%s\"\n", filename, s);
      continue;
    }

    sscanf(s+1, "%02x", &length);
    sscanf(s+3, "%04x", &addr);
    sscanf(s+7, "%02x", &type);

    if(type==0){
      printf("Programming %3d byte%s starting at 0x%04x", 
	     length, length==1?" ":"s", addr);
      a=length+(addr &0xff)+(addr>>8)+type;

      for(i=0;i<length;i++){
	sscanf(s+9+i*2,"%02x", &b);
	data[i]=b;
	a=a+data[i];
      }

      sscanf(s+9+length*2,"%02x", &b);
      checksum=b;

      if(((a+checksum)&0xff)!=0x00) {
	printf("  ** Checksum failed: got 0x%02x versus 0x%02x\n", 
	       (-a)&0xff, checksum);
	continue;
      } else {
	printf(", checksum ok\n");
      }

      upload_ram(data, addr, length);

    } else {
      if(type==0x01) {
	printf("End of file\n");
	fclose(f);

	return;
      } else {
	if(type==0x02) {
	  printf("Extended address: whatever I do with it ?\n");
	  continue;
	}
      }
    }
  }

  fclose(f);
}


void program_vendor(int vid, int pid) {
  int ret;
  short boot = 0xC0;
  short conf = 0x04;
  int did = 0x0001;
  char buf[8];

  buf[0] = boot;
  buf[1] = vid & 0xFF;
  buf[2] = (vid >> 8) & 0xFF;
  buf[3] = pid & 0xFF;
  buf[4] = (pid >> 8) & 0xFF;
  buf[5] = did & 0xFF;
  buf[6] = (did >> 8) & 0xFF;
  buf[7] = conf;
 
  printf("Writing 8 bytes to eeprom address 0x0 \n");

  printf("Boot command: 0x%02x \n", boot);
  printf("VID: 0x%04x \n", vid);
  printf("PID: 0x%04x \n", pid);
  printf("DID: 0x%04x \n", did);
  printf("I2C config: 0x%02x \n", conf);

  ret = usb_control_msg(current_handle, 0x40, 0xA9, 0x0, 0x0, buf, 8, 1000);

  if(ret < 0) {
    fprintf(stderr,"Request to upload eeprom contents failed: %s\n",
	    usb_strerror());
    return;
  }
}

void read_vendor() {
  int ret;
  int vid, pid, did;
  short boot, conf;
  char buf[8];

  printf("Reading 8 bytes from eeprom address 0x0 \n");

  ret = usb_control_msg(current_handle, 0xC0, 0xA9, 0x0, 0x0, buf, 8, 1000);

  if(ret < 0) {
    fprintf(stderr,"Request to upload eeprom contents failed: %s\n",
	    usb_strerror());
    return;
  }

  boot = buf[0];
  vid = (buf[2] << 8) | buf[1];
  pid = (buf[4] << 8) | buf[3];
  did = (buf[6] << 8) | buf[5];
  conf = buf[7];

  printf("Boot command: 0x%02x \n", boot);
  printf("VID: 0x%04x \n", vid);
  printf("PID: 0x%04x \n", pid);
  printf("DID: 0x%04x \n", did);
  printf("I2C config: 0x%02x \n", conf);
}


void program_fw(char *filename) {
  int ret = 0;
  int idx = 0;
  FILE* fid;
  char buf[64];
  char loop = 1;
  char num_read = 0;

  if (debug >= 2)
    fprintf(stderr, "Open: %s \n", filename);

  fid = fopen(filename, "rb");
  if (fid == NULL) {
    perror("fopen");
  } else {

    while(loop) {
      num_read = fread(buf, sizeof(char), 64, fid);
      
      if (debug >= 2)
	printf("Read %d bytes \n", num_read);

      if (num_read > 0) {
	ret = usb_control_msg(current_handle, 
			      0x40, 
			      0xA9, 
			      idx, 
			      0x0, 
			      buf, 
			      num_read, 
			      1000);
	idx += ret;

	if(ret < 0) {
	  fprintf(stderr,"Request to upload eeprom contents failed: %s\n",
		  usb_strerror());
	  return;
	}	
      }
      else {
	loop = 0;
	if (debug >= 1)
	  printf("Wrote %d bytes to EEPROM \n", idx);
      }
    }
  }
}


void dump_strings() {
  int ret = 0;
  char string[100];

  if (current_handle) {
    if (current_device->descriptor.iManufacturer) {
      ret = usb_get_string_simple(current_handle,
				  current_device->descriptor.iManufacturer,
				  string, 
				  sizeof(string));

      if (ret > 0)
	printf("- Manufacturer : %s\n", string);
      else
	printf("- Unable to fetch manufacturer string\n");
    }
    else
      printf("- Unable to fetch manufacturer string\n");

    if (current_device->descriptor.iProduct) {
      ret = usb_get_string_simple(current_handle,
				  current_device->descriptor.iProduct, 
				  string, 
				  sizeof(string));

      if (ret > 0)
	printf("- Product : %s\n", string);
      else
	printf("- Unable to fetch product string\n");
    }
    else
      printf("- Unable to fetch product string\n");

    if (current_device->descriptor.iSerialNumber) {
      ret = usb_get_string_simple(current_handle,
				  current_device->descriptor.iSerialNumber,
				  string,
				  sizeof(string));
	    
      if (ret > 0)
	printf("- Serial Number: %s\n", string);
      else
	printf("- Unable to fetch serial number string\n");
    }
    else
      printf("- Unable to fetch serial number string\n");
  }
  else
    printf("Unable to find USB device\n");
}


void show_help(void)
{
  printf( "fx2_programmer VID PID function [parameters]\n"
	  "\n"
	  "   Function         Parameters     Description\n"
	  "   dump_busses                     show all available devices\n"
	  "   set              address byte   changes values of a single byte\n"
	  "   program          file.ihx       programs fx2 using Intel hex format file\n"
	  "   program_vendor   VID PID        program VID/PID to EEPROM (C0 boot)\n"
	  "   read_vendor                     read first 8 bytes of EEPROM \n"
	  "                                   (boot, VID, PID, DID and conf)\n"
	  "   program_fw       file.iic       program EEPROM with firmware in iic format \n"
	  "   dump_strings                    print device strings\n"
	  //	  "   serialize_fw     fw serial      serialize firmware (debug only)\n" 
	  );
}

int main(int argc, char *argv[])
{
  char a;

  struct usb_bus *bus;
  struct usb_device *dev;

  int vid, pid;

  if(argc<4){
	show_help();
	return -1;
  }

  usb_init();
  usb_find_busses();
  usb_find_devices();

  if(!strcasecmp(argv[3], "dump_busses")){
    dump_busses();
    return 0;
  }

  vid = atoz(argv[1]);
  pid = atoz(argv[2]);

  for (bus = usb_busses; bus; bus = bus->next) {
    for (dev = bus->devices; dev; dev = dev->next) {
      /*
      printf("%s/%s %04X/%04X\n", bus->dirname, dev->filename,
	     dev->descriptor.idVendor, dev->descriptor.idProduct);
      */

      if ( (dev->descriptor.idVendor == vid) && 
	   (dev->descriptor.idProduct == pid) ) {
	current_device = dev;
      }  
    }
  }

  if(current_device==NULL){
    fprintf(stderr,"Cannot find vid 0x%x pid 0x%x \n", vid, pid);
    return -1;
  }
  printf("Using device vendor id 0x%04x product id 0x%04x\n",
	 current_device->descriptor.idVendor, 
	 current_device->descriptor.idProduct);

  current_handle=usb_open(current_device);

  if(!strcasecmp(argv[3], "set")){
    if(argc<6){
      fprintf(stderr,"Incorrect set command syntax\n");
      return -1;
    }
    a=atoz(argv[5]);
    upload_ram(&a, atoz(argv[4]), 1);
    return 0;
  }
  else if(!strcasecmp(argv[3], "dump_strings")) {
    dump_strings();
    return 0;
  }	
  else if(!strcasecmp(argv[3], "program")){
    if(argc<5){
      fprintf(stderr,"Incorrect program command syntax\n");
      return -1;
    }
    program_fx2(argv[4], 1);
    return 0;
  }
  else if(!strcasecmp(argv[3], "program_vendor")){
    if(argc<6){
      fprintf(stderr,"Incorrect program command syntax\n");
      return -1;
    }
    program_vendor(atoz(argv[4]), atoz(argv[5]));
    return 0;
  }
  else if(!strcasecmp(argv[3], "read_vendor")){
    if(argc<4){
      fprintf(stderr,"Incorrect program command syntax\n");
      return -1;
    }
    
    read_vendor();
    return 0;
  }	
  else if(!strcasecmp(argv[3], "program_fw")){
    if (argc<5){
      fprintf(stderr,"Incorrect program command syntax\n");
      return -1;
    }

    serialize_fw(argv[4], serial_filename);
    program_fw(argv[4]);
    return 0; 
  }	

  /*
  else if(!strcasecmp(argv[3], "serialize_fw")){
    if (argc<6){
      fprintf(stderr,"Incorrect program command syntax\n");
      return -1;
    }

    serialize_fw(argv[4], argv[5]);
    return 0;
  }
  */

  else {
    fprintf(stderr, "Unknown command\n");
    show_help();
    return -1;
  }

  usb_close(current_handle);
  return 0;
}
