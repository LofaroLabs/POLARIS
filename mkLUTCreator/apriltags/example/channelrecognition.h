/* -*-	indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */ 
/* 
Copyright (c) 2014, Daniel M. Lofaro <dan@danlofaro.com> 
All rights reserved. 
 
Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are met: 
    * Redistributions of source code must retain the above copyright 
      notice, this list of conditions and the following disclaimer. 
    * Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in the 
      documentation and/or other materials provided with the distribution. 
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission. 
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
*/ 
 
 
 
 
//888888888888888888888888888888888888888888 
//---------[Prerequisites for ACH]---------- 
#include <stdint.h> 
#include <time.h> 
#include <string.h> 
#include <pthread.h> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include <ach.h> 
//888888888888888888888888888888888888888888 
 
 
 
 
#define RECOGNITION_NAME "recognition" 

 

 
//888888888888888888888888888888888888888888 
//-----[Static Definitions and Offsets]----- 
//888888888888888888888888888888888888888888 
 
typedef struct controller_ref { 
  //only need a char to transmit user input 
  double key; 
  double count;
  uint32_t detectionID0;
  uint32_t detectionID1;
  uint32_t detectionID2;
  uint32_t detectionID3;
  uint32_t detectionID4;
  uint32_t detectionID5;
  uint32_t detectionID6;
  uint32_t detectionID7;
  double detectionX0;
  double detectionX1;
  double detectionX2;
  double detectionX3;
  double detectionX4;
  double detectionX5;
  double detectionX6;
  double detectionX7;
  double detectionY0;
  double detectionY1;
  double detectionY2;
  double detectionY3;
  double detectionY4;
  double detectionY5;
  double detectionY6;
  double detectionY7;
  double detectionZ0;
  double detectionZ1;
  double detectionZ2;
  double detectionZ3;
  double detectionZ4;
  double detectionZ5;
  double detectionZ6;
  double detectionZ7;
  double detectionTHETA0;
  double detectionTHETA1;
  double detectionTHETA2;
  double detectionTHETA3;
  double detectionTHETA4;
  double detectionTHETA5;
  double detectionTHETA6;
  double detectionTHETA7;
  double detectionPITCH0;
  double detectionPITCH1;
  double detectionPITCH2;
  double detectionPITCH3;
  double detectionPITCH4;
  double detectionPITCH5;
  double detectionPITCH6;
  double detectionPITCH7;
  double detectionROLL0;
  double detectionROLL1;
  double detectionROLL2;
  double detectionROLL3;
  double detectionROLL4;
  double detectionROLL5;
  double detectionROLL6;
  double detectionROLL7;

 
}__attribute__((packed)) controller_ref_t; 


