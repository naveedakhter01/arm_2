/****************************************************************************
 *
 *            Copyright (c) 2007 by HCC Embedded
 *
 * This software is copyrighted by and is the sole property of
 * HCC.  All rights, title, ownership, or other interests
 * in the software remain the property of HCC.  This
 * software may only be used in accordance with the corresponding
 * license agreement.  Any unauthorized use, duplication, transmission,
 * distribution, or disclosure of this software is expressly forbidden.
 *
 * This Copyright notice may not be removed or modified without prior
 * written consent of HCC.
 *
 * HCC reserves the right to modify this software without notice.
 *
 * HCC Embedded
 * Budapest 1133
 * Vaci ut 110
 * Hungary
 *
 * Tel:  +36 (1) 450 1302
 * Fax:  +36 (1) 450 1303
 * http: www.hcc-embedded.com
 * email: info@hcc-embedded.com
 *
 ***************************************************************************/
#ifndef _MMCI_H_
#define _MMCI_H_

#include "../fat/ff.h"

/* enable SDHC support */
#define SDHC_ENABLE		1

/* Valid voltage range. Use 0x00ff8000 means 2.7-3.6 V. */
#define VALID_VOLTAGE_RANGE 0x00ff8000u

/* Shall we use 4 bit mode? */
#define SD_ALLOW_4BIT               1
/* Allow use of GDMA module. Without DMA it has a high chance
   to have FIFO over and underruns when operating in 4 bit mode
   at high speeds. */
#define SD_ALLOW_DMA                1
/* Define how much sectors shall be able the DMA buffer hold. The DMA buffer
   is allocated the the end of the USB ram. Value must be in range 1-8. */
#define DMA_SECTORS                 8

/* Which DMA channel to use. */
#define DMA_CHANNEL                 4

extern F_DRIVER *mci_initfunc(unsigned long driver_param);

#define MMC_ERR_NOTPLUGGED -1 /* for high level */

enum {
  MMC_NO_ERROR,
  MMC_ERR_NOTINITIALIZED=101,
  MMC_ERR_INIT,
  MMC_ERR_CMD,
  MMC_ERR_STARTBIT,
  MMC_ERR_BUSY,
  MMC_ERR_CRC,
  MMC_ERR_WRITE,
  MMC_ERR_WRITEPROTECT,
  MMC_ERR_NOTAVAILABLE
};


#define _T_LOWVOLTAGE	        0x80
#define _T_UNKNOWN              0x00
#define _T_MMC			0x01
#define _T_SD			0x02
#define _T_SDV2			0x04
#define _T_SDHC			0x08

typedef struct {
  unsigned long rca;
  unsigned long spd;
  unsigned long nac;
  unsigned long wnac;
  unsigned char initok;		/* card initialized */
  unsigned char cardtype;
  unsigned char use_crc;
  unsigned long number_of_sectors;
  unsigned char bcs;		/* block count supported 0-no 1-yes */

  unsigned char CSD[16];

  unsigned char TRANSPEED;
  unsigned char R_BL_LEN;
  unsigned short CSIZE;
  unsigned char CSIZE_M;

  unsigned char TAAC;
  unsigned char NSAC;
  unsigned char R2W;
} t_mmc_dsc;


#endif

/****************************** END OF FILE **********************************/
