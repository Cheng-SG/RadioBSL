/*
 * =====================================================================================
 *
 *       Filename:  printf.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  04/04/2013 23:14:13
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */
#ifndef MPRINTF_H
#define MPRINTF_H

#include <stdio.h>

static char PrintfBuf[100];
static char* UartBuf;

#define printfInit() {U1CTL=0x10;U1TCTL=0x20;U1RCTL=0x00;U1BR0=0x09;U1BR1=0x00;U1MCTL=0x08;P3SEL|=(1<<6);P3DIR|=(1<<6);P2SEL|=(1<<2);P2DIR&=0xFB;ME2=0x30;}
#define printf(format, ...) {sprintf(PrintfBuf,format, ## __VA_ARGS__);UartBuf=PrintfBuf;while(*UartBuf!='\0'){U1TXBUF=*UartBuf++;while(!(IFG2&0x20));}}

#endif
