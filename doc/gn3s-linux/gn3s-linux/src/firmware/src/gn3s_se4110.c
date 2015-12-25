/* 
 * Code from: USRP - Universal Software Radio Peripheral (GNU Radio) 
 * 
 * Initial modifications by: 
 * 
 * Stephan Esterhuizen, Aerospace Engineering Sciences 
 * University of Colorado at Boulder 
 * Boulder CO, USA 
 *  
 * Further modifications for use with the SiGe USB module to accompany 
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
 
 
#include "gn3s_se4110.h"


char init_se4110(void)
{
  D0 = 0; 	
  D2 = 0; 
  D3 = 1; 	// P3V3_EN 
  D4 = 0; 	// EXT_REF_CTRL 
  /* D1,D5-D7 are inputs */

  OED = 0x1D; 	// Set Port D as outputs

  /* Set all "config" ports */
  A0 = 1;
  A1 = 0;
  A2 = 0;
  A3 = 1;
  A4 = 0;	// RX_EN
  A5 = 0;	// OSC_EN
  A6 = 0; 	// AGC_EN (active low) 
  A7 = 0;
  
  OEA = 0xFF; 	// Set Port A as output

  return 0;
}

char enable_se4110(void)
{
  /* Enable chip */
  A4 = 1; 	// RX_EN 
  A5 = 1; 	// OSC_EN
  return 0;
}

char disable_se4110(void)
{
  /* Enable chip */
  A4 = 0; 	// RX_EN 
  A5 = 0; 	// OSC_EN 

  return 0;
}

char reset_se4110(void)
{
  unsigned short i = 0;

  /* Disable chip */
  A4 = 0;
  A5 = 0;

  /* Delay */
  mdelay(1); // 1 ms

  /* Enable chip */
  A4 = 1;
  A5 = 1;

  return 0;
}


