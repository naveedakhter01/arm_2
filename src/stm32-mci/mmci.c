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
#include "mmci.h"
#include "../hcc_types.h"
#include "../stm32f10xreg.h"
#if SD_ALLOW_DMA
#include "gdma.h"
#include <string.h>  /* For memcpy. */
#endif

/* CCLK in KHz */
#define CCLK_KHZ 8000ul

typedef enum {
  mmr_none,
  mmr_short,
  mmr_long
} mmc_response_t;

typedef struct {
  char cmd;
  mmc_response_t resp_t;
} mmc_cmd_t;

typedef struct {
  mmc_cmd_t *cmd;
  unsigned long data;
  hcc_u32 resp[4];
} mmc_cmd_req_t;

static t_mmc_dsc g_mmcsd_type;

static hcc_u32 curr_speed;

static F_DRIVER mci_drv;

#if SD_ALLOW_DMA
static const gdma_periph_info_t mci_dma_dsc = {
  (void *)&SDIO_FIFO       /* data source address */
  , DMA_BS_8        /* transfer 8 units in one go */
  , DMA_TR_WIDTH32  /* unit width is 32 bits*/
  , 0               /* do not increment address */
  , DMA_PI_SD      /* peripheral is MCI */
};

#define __KEIL__
#ifdef __ICCARM__
/* 2K DMA buffer at the end of the USB RAM. */
__no_init static hcc_u32 dma_buffer[(512/4)*DMA_SECTORS] /*@ (0x7fd02000 - 512*DMA_SECTORS)*/;
#elif defined(__GNUC__)
/* You need to map the segment defined below (.dmabuf) to the USB RAM (0x7fd00000-0x7fd02000)
   using your linker command file. */
static hcc_u32 dma_buffer[(512/4)*DMA_SECTORS] __attribute__((section(".dmabuf")));
#elif defined(__KEIL__)
static hcc_u32 dma_buffer[(512/4)*DMA_SECTORS] __attribute__((section(".dmabuf")));
#else
#error "I dont know how to map a variable to a static address."
#endif

static const gdma_periph_info_t memory_dsc = {
      (void*) dma_buffer  /* destination address */
      , DMA_BS_256        /* burst size */
      , DMA_TR_WIDTH32    /* transfer width */
      , 1                 /* increment address */
      , 0                 /* ignored because of transfer type */
};

#endif

/* Generic in module return values. */
#define MCI_OK     1
#define MCI_FAILED 0

#define MCISTATUS_TXERRORS (MCISTATUS_DATACRCFAIL | MCISTATUS_DATATIMEOUT\
                           | MCISTATUS_TXUNDERRUN | MCISTATUS_STARTBITERR)
#define MCISTATUS_RXERRORS (MCISTATUS_DATACRCFAIL | MCISTATUS_DATATIMEOUT\
                           | MCISTATUS_RXOVERRUN | MCISTATUS_STARTBITERR)


/* Used MMC command indexes. */
#define GO_IDLE_STATE	              0
#define SEND_OP_COND	              1
#define ALL_SEND_CID	              2
#define SET_REL_ADDR	              3
#define SELECT_CARD	              7
#define SEND_IF_COND                  8
#define SEND_CSD	              9
#define STOP_TRANS	             12
#define SEND_STATUS	             13
#define SET_BLOCK_LEN	             16
#define READ_BLOCK	             17
#define READ_MULTIPLE_BLOCK	     18
#define SET_BLOCK_COUNT          23
#define WRITE_BLOCK	             24
#define WRITE_MULTIPLE_BLOCK	     25
#define APP_CMD		             55
#define APP_SD_SET_BUS_WIDTH	      6
#define APP_SD_SEND_OP_COND	     41
#define APP_SD_SET_CLR_CARD_DETECT   42

#define CARD_ST_MASK                 (0xfu<<9)
#define CARD_ST_IDLE                 0
#define CARD_ST_READY_FOR_DATA       (1<<8)


#define GO_IDLE_CMD                     (&mmc_commands[0])
#define SEND_OP_COND_CMD                (&mmc_commands[1])
#define ALL_SEND_CID_CMD                (&mmc_commands[2])
#define SET_REL_ADDR_CMD                (&mmc_commands[3])
#define SELECT_CARD_CMD                 (&mmc_commands[4])
#define SEND_CSD_CMD                    (&mmc_commands[5])
#define STOP_TRANS_CMD                  (&mmc_commands[6])
#define SEND_STATUS_CMD                 (&mmc_commands[7])
#define SEND_IF_COND_CMD                (&mmc_commands[8])
#define SET_BLOCK_LEN_CMD               (&mmc_commands[9])
#define READ_BLOCK_CMD                  (&mmc_commands[10])
#define READ_MULTIPLE_BLOCK_CMD         (&mmc_commands[11])
#define SET_BLOCK_COUNT_CMD             (&mmc_commands[12])
#define WRITE_BLOCK_CMD                 (&mmc_commands[13])
#define WRITE_MULTIPLE_BLOCK_CMD        (&mmc_commands[14])
#define APP_CMD_CMD                     (&mmc_commands[15])
#define APP_SD_SET_BUS_WIDTH_CMD        (&mmc_commands[16])
#define APP_SD_SEND_OP_COND_CMD         (&mmc_commands[17])
#define APP_SD_SET_CLR_CARD_DETECT_CMD  (&mmc_commands[18])

static mmc_cmd_t mmc_commands[] =
{
  {GO_IDLE_STATE             , mmr_none},
  {SEND_OP_COND              , mmr_short}, /*R3*/
  {ALL_SEND_CID              , mmr_long},  /*R2*/
  {SET_REL_ADDR              , mmr_short}, /*R1*/
  {SELECT_CARD               , mmr_short}, /*R1B*/
  {SEND_CSD                  , mmr_long},  /*R2*/
  {STOP_TRANS                , mmr_short}, /*R1 or R1B*/
  {SEND_STATUS               , mmr_short}, /*R1*/
  {SEND_IF_COND              , mmr_short}, /*R7*/
  {SET_BLOCK_LEN             , mmr_short}, /*R1*/
  {READ_BLOCK                , mmr_short}, /*R1*/
  {READ_MULTIPLE_BLOCK       , mmr_short}, /*R1*/
  {SET_BLOCK_COUNT           , mmr_short}, /*R1*/
  {WRITE_BLOCK               , mmr_short}, /*R1*/
  {WRITE_MULTIPLE_BLOCK      , mmr_short}, /*R1*/
  {APP_CMD                   , mmr_short}, /*R1*/
  {APP_SD_SET_BUS_WIDTH      , mmr_short}, /*R1*/
  {APP_SD_SEND_OP_COND       , mmr_short}, /*R3*/
  {APP_SD_SET_CLR_CARD_DETECT, mmr_short}  /*R1*/
};

#define OCR_HCS (1u<<30)

static int mci_write(F_DRIVER *driver,void *data, unsigned long sector, int cnt);
static int mci_read(F_DRIVER *driver,void *data, unsigned long sector, int cnt);
static volatile unsigned long delay_cnt;
static void delay(unsigned long t)
{
 delay_cnt = t;
 while (0 < delay_cnt--)
 ;
}

/*****************************************************************************
 * Name:
 *    get_wp
 * In:
 *    n/a
 * Out:
 *    0 - card is not write protected
 *    1 - card is write protected
 *
 * Description:
 *    Return state of WP pin.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int get_wp(void)
{
  /* Return write protect inactive status. */
  return(0);
}

/*****************************************************************************
 * Name:
 *    get_cd
 * In:
 *    n/a
 * Out:
 *    0 - card is not connected
 *    1 - card is connected
 *
 * Description:
 *    Return state of CD pin.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int get_cd(void)
{
  /* Return card connected state allways. */
  return(1);
}

/*****************************************************************************
 * Name:
 *    mmc_power
 * In:
 *    on - on or off
 * Out:
 *    n/a
 *
 * Description:
 *    Turn on card power. Shall only return after power level is ok.
 *
 * Assumptions:
 *
 *****************************************************************************/
static void mmc_power(hcc_u8 on)
{
  /* empty */
}

#define COMMAND_COMPLETION ( MCISTATUS_CMDREND | MCISTATUS_CMDSENT )
#define COMMAND_FAIL ( MCISTATUS_CMDTIMEOUT )

/*****************************************************************************
 * Name:
 *    mci_cmd
 * In:
 *    cmd   - command byte
 *    _data - argument of the command
 *    resp  - buffer for response
 * Out:
 *    0 - answer invalid
 *    1 - answer valid
 *
 * Description:
 *    Execute a command which does not use the data channel in paralell.
 *    (All non data read operations.)
 *
 * Assumptions:
 *
 *****************************************************************************/
static volatile unsigned int activeCmd=0;
static int mci_cmd(mmc_cmd_req_t *req)
{
  hcc_u32 status;
  unsigned int cmdval=req->cmd->cmd;

  while (SDIO_STA & MCISTATUS_CMDACTIVE)
  {
  	activeCmd++;
        SDIO_CMD = 0;
  }

  SDIO_ARG=req->data;

  switch(req->cmd->resp_t)
  {
  case mmr_none:
    break;
  case mmr_long:
    cmdval |= MCICOMMAND_LONGRESP;
  case mmr_short:
    cmdval |= MCICOMMAND_RESP;
    break;
  default:
    HCC_ASSERT(0);
  }

  req->resp[0]=req->resp[1]=req->resp[2]=req->resp[3]=0;
  /* Clear static status flags. */
  SDIO_ICR =  CEATAEND | SDIOIT | DBCKENDC | STBITERRC | DATAENDC | CMDSENTC | CMDRENDC |
              RXOVERRC | TXUNDERRC | DTIMEOUTC | CTIMEOUTC | DCRCFAILC | DCRCFAIL;
  delay(1);

  SDIO_CMD = cmdval| MCICOMMAND_ENABLE;
  delay(10);

  /* Wait while the command is not sent.  */
  while((status = SDIO_STA & 0x000000C5) == 0)
  ;

  delay(10);
  
 
  /* Check outcome. */
  if (status & MCISTATUS_CMDTIMEOUT)
  {
    SDIO_CMD = 0;
    SDIO_ICR = ~0;
    return(MCI_FAILED);
  }

  SDIO_ICR = ~0;
  /* Copy response. */
  if (req->cmd->resp_t != mmr_none)
  {
    HCC_ASSERT(SDIO_RESPCMD == 0x3f || SDIO_RESPCMD == req->cmd->cmd);

    req->resp[0]=SDIO_RESP1;
    req->resp[1]=SDIO_RESP2;
    req->resp[2]=SDIO_RESP3;
    req->resp[3]=SDIO_RESP4;
  }
  SDIO_CMD = 0;
  return(MCI_OK);
}

/*****************************************************************************
 * Name:
 *    mci_set_clk
 * In:
 *    khz - desired freq in KHz
 * Out:
 *    Achieved clock freq in KHz.
 *
 * Description:
 *    Comfigure MCI to generate a specific clock frequency.
 *
 * Assumptions:
 *
 *****************************************************************************/
static hcc_u32 mci_set_clk(hcc_u32 khz)
{
  hcc_u32 d=0;
  hcc_u32 mclk=CCLK_KHZ;
  SDIO_CLKCR &= ~SDIO_CLKCR_EN;

  switch(RCC_CFGR & HPRE_512)
  {
  case HPRE_512:
    mclk >>= 9;
    break;
  case HPRE_256:
    mclk >>= 8;
    break;
  case HPRE_128:
    mclk >>= 7;
    break;
  case HPRE_64:
    mclk >>= 6;
    break;
  case HPRE_16:
    mclk >>= 4;
    break;
  case HPRE_8:
    mclk >>= 3;
    break;
  case HPRE_4:
    mclk >>= 2;
    break;
  case HPRE_2:
    mclk >>= 1;
    break;
  }

  if (khz >= mclk)
  {
    d=0;
  }
  else
  {
    d=(mclk/khz)-2;
    /* Ensure clock speed is less than the specified value. */
    if (mclk%khz != 0)
    {
      d++;
    }
  }

  if (d>0xff)
  {
    /* In this case we should use slower clock (change PCLKSEL1). */
    d=0xff;
  }

  SDIO_CLKCR = (SDIO_CLKCR & ~0xff)| (d & 0xff);
  delay(1000);
  SDIO_CLKCR |= SDIO_CLKCR_EN;
  return(mclk/(d+2));
}

/*****************************************************************************
 * Name:
 *    get_bits
 * In:
 *    src: source address
 *    st:  start bit offset
 *    len: number of bits to get
 * Out:
 *    required bits
 *
 * Description:
 *    Retrun specified bits.
 *
 * Assumptions:
 *
 *****************************************************************************/
unsigned short get_bits(unsigned char *src, unsigned char st, unsigned char len)
{
  hcc_u16 res=0;
  hcc_u8 bitofs;

  for(bitofs=st+len-1; bitofs>=st; bitofs--)
  {
    hcc_u8 byte=src[bitofs>>3];
    res<<=1;
    if (byte & (1<<(bitofs & 7)))
    {
      res|= 1;
    }
  }
  return(res);
}

/*****************************************************************************
 * Name:
 *    mci_init_card
 * In:
 *    n/a
 * Out:
 *    MMC_ERR_CMD or MMC_NO_ERROR
 *
 * Description:
 *    Initialize card.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int mci_init_card(void)
{
  int t, trc;
  mmc_cmd_req_t cmd_req;
  static const unsigned char m[16]={  0,10,12,13,15,20,25,30,35,40,45,50,55
                                  ,60,70,80};

  SDIO_CLKCR=0;
  /* By default we use 150 KHz. This value is avaiable even if MCI input clock
     egualt to its maximum value (72MHz). */
  mci_set_clk(150);

  /* Set card to non initialized state. */
  g_mmcsd_type.initok=0;
  g_mmcsd_type.cardtype=_T_UNKNOWN;

  /* Send CMD0. */
  cmd_req.cmd=GO_IDLE_CMD;
  cmd_req.data=0;
  (void)mci_cmd(&cmd_req);

  /* Configure CMD pin to be open collector. This would be really needed only if
     multiple cards could be connected. */

#if SDHC_ENABLE
  /* Send CMD8 to notify v2 cards host is v2 capable. */
  cmd_req.cmd=SEND_IF_COND_CMD;
  cmd_req.data=0x1a5;
  trc=mci_cmd(&cmd_req);
  if (trc==MCI_OK)
  {
    if (cmd_req.resp[0] == 0x1a5)
    {
      g_mmcsd_type.cardtype = _T_SDV2;
    }
    else
    { /* invalid response -> unusable card */
      return(MMC_ERR_INIT);
    }
  }
#endif

    /* Tell card next command is application specific. */
  cmd_req.cmd=APP_CMD_CMD;
  cmd_req.data=0;
  (void)mci_cmd(&cmd_req);

  /* Ask card for compatible voltage range using application specific
     command. */
  cmd_req.cmd=APP_SD_SEND_OP_COND_CMD;
  cmd_req.data=VALID_VOLTAGE_RANGE;
#if SDHC_ENABLE
  if (g_mmcsd_type.cardtype & _T_SDV2)
  {
    cmd_req.data |= OCR_HCS;
  }
#endif
  trc=mci_cmd(&cmd_req);

  /* At this point the SD card is doing it's initialization. This may
     take a while. Unfortunately there is no specified timeout for this.
     We query the card 5000 times, and expect it to exit busy state during
     5000 requests.
     For SD cards this is more than 5000*2*(48+64+136) bit times.
     For MMC cards this is more than 5000*(48+64+136) bit times. (150KHz->8,5s)
     This should be enough for any card.
     (see wait loops below) */
  t=5000;


  if (trc == MCI_OK)
  {
    /* This is an SD card. */
    g_mmcsd_type.cardtype |= _T_SD;

    /* wait while SD card finishes initialization. */
    while (((cmd_req.resp[0] & BIT31)==0) /* the card is busy */
             && --t                       /* need to retry */
             && (trc == MCI_OK))	  /* no command error */
    {
      cmd_req.cmd=APP_CMD_CMD;
      cmd_req.data=0;
      trc=mci_cmd(&cmd_req);
      if (trc==MCI_OK)
      {
        cmd_req.cmd=APP_SD_SEND_OP_COND_CMD;
        cmd_req.data=VALID_VOLTAGE_RANGE;
#if SDHC_ENABLE
        if (g_mmcsd_type.cardtype & _T_SDV2)
        {
          cmd_req.data |= OCR_HCS;
        }
#endif
        trc=mci_cmd(&cmd_req);
      }
    }

#if SDHC_ENABLE
    if (t && trc==MCI_OK)
    {
      if (cmd_req.resp[0] & OCR_HCS)
      {
        g_mmcsd_type.cardtype |= _T_SDHC;
      }
    }
#endif
  }
  else
  {
    /* This is an MMC card. */
    g_mmcsd_type.cardtype = _T_MMC;

      /* Send CMD1. (Ask for compatible voltage range.) */
    cmd_req.cmd=SEND_OP_COND_CMD;
    cmd_req.data=VALID_VOLTAGE_RANGE;
    trc=mci_cmd(&cmd_req);

   /* wait while MMC card finishes initialization. */
    while (((cmd_req.resp[0] & BIT31)==0) /* the card is busy */
             && --t                       /* still need to retry */
             && (trc == MCI_OK))	    /* no command error */
    {
      cmd_req.cmd=SEND_OP_COND_CMD;
      cmd_req.data=VALID_VOLTAGE_RANGE;
      trc=mci_cmd(&cmd_req);
    }
  }

  /* If we retried maximum number of times and the card is still busy,
     or we encountered a communication error. */
  if (t==0 || trc==MCI_FAILED)
  {
    return(MMC_ERR_INIT);
  }

  /* Get card identification. */
  cmd_req.cmd=ALL_SEND_CID_CMD;
  cmd_req.data=0;
  trc=mci_cmd(&cmd_req);
  if (trc==MCI_OK)
  {
    /* Set relative address. */
    cmd_req.cmd=SET_REL_ADDR_CMD;
    cmd_req.data=0;
    trc=mci_cmd(&cmd_req);	/* stby state after this command */
  }

  if (trc==MCI_OK)
  {
    /* store relative address MMC=0 / SD=given by SD */
    if (g_mmcsd_type.cardtype & _T_SD)
    {
      g_mmcsd_type.rca=cmd_req.resp[0] & 0xffff0000;
    }
    else
    {
      g_mmcsd_type.rca=0;
    }

    /* get CSD block */
    cmd_req.cmd=SEND_CSD_CMD;
    cmd_req.data=g_mmcsd_type.rca;
    trc=mci_cmd(&cmd_req);

    if (trc==MCI_OK)
    {
      hcc_u32 tmp;
      /* Fix byte order. */
      tmp=cmd_req.resp[0];
      cmd_req.resp[0]=cmd_req.resp[3];
      cmd_req.resp[3]=tmp;
      tmp=cmd_req.resp[1];
      cmd_req.resp[1]=cmd_req.resp[2];
      cmd_req.resp[2]=tmp;

      if ((g_mmcsd_type.cardtype & _T_SDHC)==0)
      {
        g_mmcsd_type.TAAC=get_bits((hcc_u8*)cmd_req.resp,112,8) ;
        g_mmcsd_type.NSAC=get_bits((hcc_u8*)cmd_req.resp, 104, 8);
        g_mmcsd_type.TRANSPEED=get_bits((hcc_u8*)cmd_req.resp,96,8);
        g_mmcsd_type.R_BL_LEN=get_bits((hcc_u8*)cmd_req.resp,80,4);
        g_mmcsd_type.CSIZE=get_bits((hcc_u8*)cmd_req.resp,62,12);
        g_mmcsd_type.CSIZE_M=get_bits((hcc_u8*)cmd_req.resp,47,3);
        g_mmcsd_type.R2W=get_bits((hcc_u8*)cmd_req.resp,26,3);

        /* Calculate number of sectors. */
        g_mmcsd_type.number_of_sectors=1UL<<g_mmcsd_type.CSIZE_M;
        g_mmcsd_type.number_of_sectors<<=2;
        g_mmcsd_type.number_of_sectors<<=g_mmcsd_type.R_BL_LEN;
        g_mmcsd_type.number_of_sectors*=(g_mmcsd_type.CSIZE+1);
        g_mmcsd_type.number_of_sectors>>=9;
      }
      else
      {
        g_mmcsd_type.TRANSPEED=get_bits((hcc_u8*)cmd_req.resp,96,8);
        g_mmcsd_type.number_of_sectors=get_bits((hcc_u8 *)cmd_req.resp,48,22)*1024;
      }


      /* select card and go to transfer state */
      cmd_req.cmd=SELECT_CARD_CMD;
      cmd_req.data=g_mmcsd_type.rca;
      trc=mci_cmd(&cmd_req);
      if (trc==MCI_OK)
      {
        /* set block length to 512 */
        cmd_req.cmd=SET_BLOCK_LEN_CMD;
        cmd_req.data=512;
        trc=mci_cmd(&cmd_req);
        /* Check if card support multiple block commands. */
        if (cmd_req.resp[0] & BIT22)
        {
          g_mmcsd_type.bcs=0;
          trc=MCI_OK;
        }
        else
        {
          g_mmcsd_type.bcs=1;
        }
      }
#if SD_ALLOW_4BIT
      if (g_mmcsd_type.cardtype & _T_SD)
      {
        if (trc==MCI_OK)
        {
          cmd_req.cmd=APP_CMD_CMD;
          cmd_req.data=g_mmcsd_type.rca;
          (void)mci_cmd(&cmd_req);
          /* disconnect SD_DAT[3] pull-up for data transfer */
          cmd_req.cmd=APP_SD_SET_CLR_CARD_DETECT_CMD;
          cmd_req.data=0;
          trc=mci_cmd(&cmd_req);
        }
        if (trc==MCI_OK)
        {
          cmd_req.cmd=APP_CMD_CMD;
          cmd_req.data=g_mmcsd_type.rca;
          (void)mci_cmd(&cmd_req);
          /* set 4 bit mode */
          cmd_req.cmd=APP_SD_SET_BUS_WIDTH_CMD;
          cmd_req.data=2;
          if (MCI_OK==mci_cmd(&cmd_req))
          {
            SDIO_CLKCR|=SDIO_CLKCR_WIDEBUS_4;
          }
        }
      }
#endif
    } /* end CSD no error */
  } /* end stby commands no error */

  if (trc==MCI_FAILED)
  {
    return(MMC_ERR_CMD);
  }

  /* Calculate and set transfer speed. */
  {
    unsigned long spd;
    int i;

    spd=1;
    for (i=0;i<(g_mmcsd_type.TRANSPEED&0x7);i++)
    {
      spd*=10;
    }

    spd*=m[(g_mmcsd_type.TRANSPEED>>3)&0x7];
    spd*=10;	/* speed in kHz */

    g_mmcsd_type.spd=spd;

    curr_speed=mci_set_clk(spd);
  }

  /* Calculate and set timeout values. */
#if SDHC_ENABLE
  if (g_mmcsd_type.cardtype & _T_SDHC)
  {
    /* HCSD cards use predefined timeout values. */
    g_mmcsd_type.nac=curr_speed*100;   /* 100 mS */
    g_mmcsd_type.wnac=curr_speed*250;  /* 250 mS */
  }
  else
#endif
  {
    int i;
    unsigned long taac,nac;

    taac=10000000;
    for (i=0;i<(g_mmcsd_type.TAAC&0x7);i++)
    {
      taac/=10;
    }

    nac=10*(((curr_speed*m[(g_mmcsd_type.TAAC>>3)&0x7]+taac-1)/taac)
            +(100*g_mmcsd_type.NSAC));

    g_mmcsd_type.nac=nac;				/* calculate max. read wait clock cycles */
    g_mmcsd_type.wnac=nac*(1<<(g_mmcsd_type.R2W+2));	/* calculate max. write clock cycles */
  }
    /* Configure CMD pin to be push-pull. */
  g_mmcsd_type.initok=1;
  return(MMC_NO_ERROR);
}

/*****************************************************************************
 * Name:
 *    mci_getstatus
 * In:
 *    driver - driver structure
 * Out:
 *    F_ST_WPROTECT - card is write protected.
 *    F_ST_MISSING  - card is not connected
 *    F_ST_CHANGED  - card has been changed
 *
 * Description:
 *    Get status of card, missing or/and removed,changed,writeprotected.
 *
 * Assumptions:
 *
 *****************************************************************************/
long mci_getstatus(F_DRIVER *driver)
{
  long state=0;

  /* Check if card is write protected. */
  if (get_wp())
  {
    state|=F_ST_WRPROTECT;
  }

  /* Check if card is connected. */
  if (get_cd()==0)
  {
    g_mmcsd_type.initok=0;
    state|=F_ST_MISSING;
  }
  else
  {
    /* Check if card has been initialized. */
    if (g_mmcsd_type.initok)
    {
      /* Check card status. */
      mmc_cmd_req_t cmd_req;
      cmd_req.cmd=SEND_STATUS_CMD;
      cmd_req.data=g_mmcsd_type.rca;
      if (MCI_OK==mci_cmd(&cmd_req))
      {
        /* If the card is in idle state then it has not been initialized and the
           value of initok does not reflects the reality. This means the card has
           been changed. */
        if ((cmd_req.resp[0] & CARD_ST_MASK)== CARD_ST_IDLE)
        {
          state|=F_ST_CHANGED;
        }
      }
      else
      { /* If there is a communication error, then we set F_ST_CHANGED,
           which will force re-initialization below. */
        g_mmcsd_type.initok=0;
      }
    }

    /* If the card has been canged, then we need to initialize it. */
    if ((state & F_ST_CHANGED)
        || !g_mmcsd_type.initok)
    {
      if (MMC_NO_ERROR!=mci_init_card())
      {
        /* Disable MMC clock. */
        SDIO_CLKCR &= ~SDIO_CLKCR_EN;
        g_mmcsd_type.initok=0;
      };
    }

    /* If at this point initok is 0, then we can not communicate with the card
       so we treat it as not connected. */
    if (!g_mmcsd_type.initok)
    {
      state|=F_ST_MISSING;
    }
  }
  return state;
}

/*****************************************************************************
 * Name:
 *    mci_getphy
 * In:
 *    driver - driver structure
 *    phy - this structure has to be filled with physical information
 * Out:
 *    MMC_MO_ERROR           - finished ok.
 *    MMC_ERR_NOTINITIALIZED - card is not initialized.
 *
 * Description:
 *    Determinate flash card physicals.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int mci_getphy(F_DRIVER *driver,F_PHY *phy)
{
   if (g_mmcsd_type.initok==0) return MMC_ERR_NOTINITIALIZED;
   phy->number_of_cylinders=0;
   phy->sector_per_track=63;
   phy->number_of_heads=255;
   phy->number_of_sectors=g_mmcsd_type.number_of_sectors;
   phy->media_descriptor=0xf0;

   return MMC_NO_ERROR;
}

/*****************************************************************************
 * Name:
 *    mmc_release
 * In:
 *    driver - pointer to driver to be released
 * Out:
 *    n/a
 *
 * Description:
 *    Release specified driver.
 *
 * Assumptions:
 *
 *****************************************************************************/
static void mci_release(F_DRIVER *driver)
{
  /* Stop MMC bus clock. */
  SDIO_CLKCR=0;

  /* Turn off MMC card power. */
  mmc_power(0);
}

/*****************************************************************************
 * Name:
 *    mci_writesector
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start.
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting a datablock from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int mci_writesector(F_DRIVER *driver,void *data, unsigned long sector)
{
#if SD_ALLOW_DMA
  memcpy(dma_buffer, data, 512);
  return mci_write(driver, data, sector, 1);
#else
  return mci_write(driver, data, sector, 1);
#endif

}

/*****************************************************************************
 * Name:
 *    mci_writemultiplesector
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start.
 *    cnt    - number of sectors
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting a datablock from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
int mci_writemultiplesector(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
#if SD_ALLOW_DMA
  while(cnt)
  {
    /* Determine next chunk size. */
    int chunk_size=cnt > sizeof(dma_buffer)/512 ?
                                                sizeof(dma_buffer)/512 : cnt;
    int r;

    /* Fill DMA bufer. */
    memcpy(dma_buffer, data, chunk_size<<9);

    /* Write data to card. */
    r = mci_write(driver, data, sector, chunk_size);
    if (MMC_NO_ERROR != r)
    {
      return(r);
    }

    cnt-=chunk_size;
    sector+=chunk_size;
    data=(void*)(((hcc_u8*)data)+(chunk_size<<9));
  }
  return(MMC_NO_ERROR);
#else
  return mci_write(driver, data, sector, cnt);
#endif
}


/*****************************************************************************
 * Name:
 *    wait_busy
 * Out:
 *    MCI_OK if no error was found and the card is
 *    ready to accept data
 *
 * Description:
 *    waits for the card to get out of busy - we don't have a direct
 *    method for checking the busy signal.
 *****************************************************************************/
static int wait_busy( )
{
  mmc_cmd_req_t cmd_req;

  cmd_req.cmd = SEND_STATUS_CMD;
  cmd_req.data = g_mmcsd_type.rca;
  do {
    if( mci_cmd( &cmd_req ) != MCI_OK )
      return MCI_FAILED;
  } while( !( cmd_req.resp[0] & CARD_ST_READY_FOR_DATA ) );

  return MCI_OK;
} /* wait_busy( ) */


/*****************************************************************************
 * Name:
 *    mci_write
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start.
 *    cnt    - number of sectors to write
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting a datablock from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
#if SD_ALLOW_DMA
static int mci_write(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
  int multi;
  mmc_cmd_req_t cmd_req;

  /* card missing */
  if (!get_cd())
  {
    g_mmcsd_type.initok=0;
    return MMC_ERR_NOTPLUGGED;
  }

  /* card is write protected */
  if (get_wp())
  {
    return MMC_ERR_WRITEPROTECT;
  }

  /* card is not in the right state */
  if (!g_mmcsd_type.initok)
  {
    return MMC_ERR_NOTINITIALIZED;
  }

  /* wait for previous transfers to end */
  if( wait_busy( ) != MCI_OK )
    return MMC_ERR_CMD;

  /* Check if we can use multisector write. */
  multi=(cnt > 1) && (g_mmcsd_type.bcs==1) ? 1:0;


  SDIO_DTIMER=g_mmcsd_type.wnac;

  /* Configure next block write command. */
  if (multi)
  {
    cmd_req.cmd=WRITE_MULTIPLE_BLOCK_CMD;
    SDIO_DLEN=cnt<<9; /*=cnt*512*/
  }
  else
  {
    cmd_req.cmd=WRITE_BLOCK_CMD;
    SDIO_DLEN=512;
  }

  /* If multi block write is used, this loop will execute only
     once. */
  while(cnt)
  {
#if SDHC_ENABLE
    if ((g_mmcsd_type.cardtype & _T_SDHC)==0)
    {
      cmd_req.data=sector<<9;
    }
    else
    {
      cmd_req.data=sector;
    }
#else
    cmd_req.data=sector<<9;
#endif

    /* Clear static status flags. */
    SDIO_ICR = MCICLEAR_CMDCRCFAILCLR
                | MCICLEAR_CMDTIMEOUTCLR
                | MCICLEAR_CMDRESPENDCLR
                | MCICLEAR_CMDSENTCLR
                | MCICLEAR_DATACRCFAILCLR
                | MCICLEAR_DATATIMEOUTCLR
                | MCICLEAR_TXUNDERRUNCLR
                | MCICLEAR_RXOVERRUNCLR
                | MCICLEAR_DATAENDCLR
                | MCICLEAR_STARTBITERRCLR
                | MCICLEAR_DATABLOCKENDCLR;


    if (gdma_start_channel(DMA_CHANNEL, (gdma_periph_info_t *)&memory_dsc,
                           (gdma_periph_info_t *)&mci_dma_dsc, SDIO_DLEN, DMA_FW_MP_P))
    {
      return MMC_ERR_CMD;
    };
    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    SDIO_DCTRL = MCIDATACTRL_ENABLE | MCIDATACTRL_DMAEN | (9<< 4);
    delay(10);
    while ((SDIO_STA & MCISTATUS_TXACTIVE) != 0)
    ;
    while((gdma_get_status(DMA_CHANNEL) == DMA_CHNST_BUSY
          || (SDIO_STA & MCISTATUS_DATAEND) == 0)
          && (SDIO_STA & MCISTATUS_TXERRORS)==0)
      ;

    SDIO_DCTRL &= ~MCIDATACTRL_ENABLE;
    if (multi)
    {
      break;
    }


    /* Check outcome. */
    if (gdma_get_status(DMA_CHANNEL) == DMA_CHNST_ERROR
        || (SDIO_STA & MCISTATUS_TXERRORS))
    {
      SDIO_DCTRL=0;
      gdma_stop_channel(DMA_CHANNEL, 1);
      return(MMC_ERR_CMD);
    }

    cnt--;
    sector++;
    SDIO_DTIMER=g_mmcsd_type.wnac;
    SDIO_DLEN=512;
  }

  if (multi)
  {
    cmd_req.cmd=STOP_TRANS_CMD;
    cmd_req.data=0;
    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    /* Wait till card exits busy state. */

    /* Check outcome. */
    if (gdma_get_status(DMA_CHANNEL) == DMA_CHNST_ERROR
        || (SDIO_STA & MCISTATUS_TXERRORS))
    {
      SDIO_DCTRL=0;
      gdma_stop_channel(DMA_CHANNEL, 1);
      return(MMC_ERR_CMD);
    }
  }
  return MMC_NO_ERROR;
}
#else
static int mci_write(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
  int multi;
  mmc_cmd_req_t cmd_req;

  /* card missing */
  if (!get_cd())
  {
    g_mmcsd_type.initok=0;
    return MMC_ERR_NOTPLUGGED;
  }

  /* card is write protected */
  if (get_wp())
  {
    return MMC_ERR_WRITEPROTECT;
  }

  /* card is not in the right state */
  if (!g_mmcsd_type.initok)
  {
    return MMC_ERR_NOTINITIALIZED;
  }

  /* Check if we can use multisector write. */
  multi=(cnt > 1) && (g_mmcsd_type.bcs==1) ? 1:0;


  SDIO_DTIMER=g_mmcsd_type.wnac;

  /* Configure next block write command. */
  if (multi)
  {
    cmd_req.cmd=WRITE_MULTIPLE_BLOCK_CMD;
    SDIO_DLEN=cnt<<9; /*=cnt*512*/
  }
  else
  {
    cmd_req.cmd=WRITE_BLOCK_CMD;
    SDIO_DLEN=512;
  }

  /* If multi block write is used, this loop will execute only
     once. */
  while(cnt)
  {
#if SDHC_ENABLE
    if ((g_mmcsd_type.cardtype & _T_SDHC)==0)
    {
      cmd_req.data=sector<<9;
    }
    else
    {
      cmd_req.data=sector;
    }
#else
    cmd_req.data=sector<<9;
#endif

    /* Clear static status flags. */
    SDIO_ICR = MCICLEAR_CMDCRCFAILCLR
                | MCICLEAR_CMDTIMEOUTCLR
                | MCICLEAR_CMDRESPENDCLR
                | MCICLEAR_CMDSENTCLR
                | MCICLEAR_DATACRCFAILCLR
                | MCICLEAR_DATATIMEOUTCLR
                | MCICLEAR_TXUNDERRUNCLR
                | MCICLEAR_RXOVERRUNCLR
                | MCICLEAR_DATAENDCLR
                | MCICLEAR_STARTBITERRCLR
                | MCICLEAR_DATABLOCKENDCLR;

    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    SDIO_DCTRL = MCIDATACTRL_ENABLE | (9<< 4);

    /* Send data. */
    while(1)
    {
      /* Fill FIFO. */
      while((SDIO_STA & (MCISTATUS_TXERRORS
                            | MCISTATUS_TXFIFOFULL | MCISTATUS_DATAEND))==0)
      {
        SDIO_FIFO=*(hcc_u32*)data;
        data=((hcc_u32*)data)+1;
      }

      /* Look for errors and transfer end. */
      if (SDIO_STA & (MCISTATUS_TXERRORS | MCISTATUS_DATAEND))
      {
        break;
      }
    }


    if (multi)
    {
      break;
    }

    /* Check outcome. */
    if (SDIO_STA & MCISTATUS_TXERRORS)
    {
      SDIO_DCTRL=0;
      return(MMC_ERR_CMD);
    }

    cnt--;
    sector++;
    SDIO_DTIMER=g_mmcsd_type.wnac;
    SDIO_DLEN=512;
  }

  if (multi)
  {
    cmd_req.cmd=STOP_TRANS_CMD;
    cmd_req.data=0;
    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    /* Check outcome. */
    if (SDIO_STA & MCISTATUS_TXERRORS)
    {
      SDIO_DCTRL=0;
      return(MMC_ERR_CMD);
    }
  }

  return MMC_NO_ERROR;
}
#endif
/*****************************************************************************
 * Name:
 *    mci_readsector
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start read card.
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting a datablock from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
static int mci_readsector(F_DRIVER *driver,void *data, unsigned long sector)
{
#if SD_ALLOW_DMA
  int r=mci_read(driver, data, sector, 1);

  if (r != MMC_NO_ERROR)
  {
    return(r);
  }
  memcpy(data, dma_buffer, 512);
  return(MMC_NO_ERROR);
#else
  return mci_read(driver, data, sector, 1);
#endif
}

/*****************************************************************************
 * Name:
 *    mci_readmultiplesector
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start read card.
 *    cnt    - number of setcors to be read
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting datablocks from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
int mci_readmultiplesector(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
#if SD_ALLOW_DMA
  while(cnt)
  {
    /* Determine next chunk size. */
    int chunk_size=cnt > sizeof(dma_buffer)/512 ?
                                                sizeof(dma_buffer)/512 : cnt;
    int r;

    /* Read some sectors. */
    r = mci_read(driver, data, sector, chunk_size);
    if (MMC_NO_ERROR != r)
    {
      return(r);
    }

    cnt-=chunk_size;
    sector+=chunk_size;
    /* Copy data to user buffer. */
    chunk_size<<=9;
    memcpy(data, dma_buffer, chunk_size);
    data=(void*)(((hcc_u8*)data)+chunk_size);
  }
  return(MMC_NO_ERROR);
#else
  return mci_read(driver, data, sector, cnt);
#endif
}


/*****************************************************************************
 * Name:
 *    mci_read
 * In:
 *    driver - driver structure
 *    data   - pointer where to store received data (512bytes)
 *    sector - sector number where to start read card.
 *    cnt    - number of setcors to be read
 * Out:
 *    0 - if successful
 *    other if any error (crc,timeouts)
 *
 * Description:
 *    Getting datablocks from the card.
 *
 * Assumptions:
 *
 *****************************************************************************/
#if SD_ALLOW_DMA
static int mci_read(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
  hcc_u32 n_words;
  hcc_u8 multi;
  hcc_u8 error;
  int    iStatus;
#if INTERRUPT_ENABLE  
  hcc_u32 cpu_sr;
#endif
  if (!get_cd())
  {
    g_mmcsd_type.initok=0;
    return MMC_ERR_NOTPLUGGED;
  }

  if (!g_mmcsd_type.initok)
  {
    return MMC_ERR_NOTINITIALIZED;
  }

  /* wait for previous transfers to end */
  if( wait_busy( ) != MCI_OK )
    return MMC_ERR_CMD;

  error=0;

  multi=g_mmcsd_type.bcs && cnt > 1 ? 1 : 0;

  /* We add 50 to nac because we start data channel before command channel
     is started. nac is counted from the end of the command so we need to add
     at least 48+2 bit times to the timeout. */
  SDIO_DTIMER=g_mmcsd_type.nac > 0xfffffffful-50
                                   ? 0xfffffffful : g_mmcsd_type.nac+50;
  SDIO_DCTRL = MCIDATACTRL_FROM_CARD | (9<< 4);

  while(cnt)
  {
    /* Prepare command channel. */
    if (multi)
    {
      SDIO_CMD=READ_MULTIPLE_BLOCK_CMD->cmd;
      n_words=cnt<<7; /*(cnt*512)/4*/
      SDIO_DLEN=n_words<<2; /*cnt*512*/
    }
    else
    {
      SDIO_CMD=READ_BLOCK_CMD->cmd;
      SDIO_DLEN=512;
      n_words=512/4;
    }

#if SDHC_ENABLE
    if ((g_mmcsd_type.cardtype & _T_SDHC)==0)
    {
      SDIO_ARG=sector<<9;
    }
    else
    {
      SDIO_ARG=sector;
    }
#else
    SDIO_ARG=sector<<9;
#endif

    SDIO_CMD |= MCICOMMAND_RESP;

    /* Clear static status flags. */
    SDIO_ICR = MCICLEAR_CMDCRCFAILCLR
                | MCICLEAR_CMDTIMEOUTCLR
                | MCICLEAR_CMDRESPENDCLR
                | MCICLEAR_CMDSENTCLR
                | MCICLEAR_DATACRCFAILCLR
                | MCICLEAR_DATATIMEOUTCLR
                | MCICLEAR_TXUNDERRUNCLR
                | MCICLEAR_RXOVERRUNCLR
                | MCICLEAR_DATAENDCLR
                | MCICLEAR_STARTBITERRCLR
                | MCICLEAR_DATABLOCKENDCLR;

    /* Configure DMA channel. */
    SDIO_CLKCR |= SDIO_CLKCR_HWFC_EN;
    if (gdma_start_channel(DMA_CHANNEL, (gdma_periph_info_t *)&memory_dsc,
                         (gdma_periph_info_t *)&mci_dma_dsc, SDIO_DLEN, DMA_FW_PM_P))
    {
      return MMC_ERR_CMD;
    };
    /* Command channel must be started right after data channel has been
       started. Otherwise we may have a data timeout. */
#if INTERRUPT_ENABLE
  /* Disable interrurp and save current state of the interrupt flags */
    cpu_sr=__get_CPSR();
    __disable_interrupt();
#endif
    /* Send command. */
  while (SDIO_STA & MCISTATUS_CMDACTIVE)
  {
        SDIO_CMD = 0;
  }
    SDIO_CMD |= MCICOMMAND_ENABLE;
    delay(10);
    // while ((SDIO_STA & MCISTATUS_CMDREND) == 0)
    do {
      iStatus = SDIO_STA;
    } while ((iStatus & ( MCISTATUS_RXERRORS | 0xff )) == 0);
    SDIO_DCTRL = MCIDATACTRL_ENABLE | MCIDATACTRL_DMAEN | MCIDATACTRL_FROM_CARD | (9<< 4);
#if INTERRUPT_ENABLE
    __set_CPSR(cpu_sr);
#endif

    delay(10);
    while (( SDIO_STA & MCISTATUS_RXACTIVE) != 0)
    ;
    while(gdma_get_status(DMA_CHANNEL) == DMA_CHNST_BUSY
          && (SDIO_STA & (MCISTATUS_RXERRORS | MCISTATUS_CMDTIMEOUT))==0)
      ;

    SDIO_CLKCR &= ~SDIO_CLKCR_HWFC_EN;
    delay(10);
    SDIO_DCTRL &= ~MCIDATACTRL_ENABLE;
    if (gdma_get_status(DMA_CHANNEL) == DMA_CHNST_ERROR
        || (SDIO_STA & (MCISTATUS_RXERRORS | 0x3f)))
    {
      error=1;
    }
    SDIO_ICR =~0;

    if (multi)
    {
      break;
    }

    /* Check outcome. */
    if (error)
    {
      SDIO_DCTRL=0;
      gdma_stop_channel(DMA_CHANNEL, 1);
      return(MMC_ERR_CMD);
    }

    cnt--;
    sector++;
  }

  if (multi)
  {
    mmc_cmd_req_t cmd_req;

    cmd_req.cmd=STOP_TRANS_CMD;
    cmd_req.data=0;
    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    /* Check outcome. */
    if (error)
    {
      SDIO_DCTRL=0;
      return(MMC_ERR_CMD);
    }
  }
  return MMC_NO_ERROR;
}

#else
static int mci_read(F_DRIVER *driver,void *data, unsigned long sector, int cnt)
{
  hcc_u32 n_words;
  hcc_u8 multi;
  hcc_u8 error;
#if INTERRUPT_ENABLE  
  hcc_u32 cpu_sr;
#endif

  if (!get_cd())
  {
    g_mmcsd_type.initok=0;
    return MMC_ERR_NOTPLUGGED;
  }

  if (!g_mmcsd_type.initok)
  {
    return MMC_ERR_NOTINITIALIZED;
  }

  error=0;

  multi=g_mmcsd_type.bcs && cnt > 1 ? 1 : 0;

  /* We add 50 to nac because we start data channel before command channel
     is started. nac is counted from the end of the command so we need to add
     at least 48+2 bit times to the timeout. */
  SDIO_DTIMER=g_mmcsd_type.nac > 0xfffffffful-50
                                   ? 0xfffffffful : g_mmcsd_type.nac+50;

  SDIO_DCTRL = MCIDATACTRL_FROM_CARD | (9<< 4);

  while(cnt)
  {
    /* Prepare command channel. */
    if (multi)
    {
      SDIO_CMD=READ_MULTIPLE_BLOCK_CMD->cmd;
      n_words=cnt<<7; /*(cnt*512)/4*/
      SDIO_DLEN=n_words<<2; /*cnt*512*/
    }
    else
    {
      SDIO_CMD=READ_BLOCK_CMD->cmd;
      SDIO_DLEN=512;
      n_words=512/4;
    }

#if SDHC_ENABLE
    if ((g_mmcsd_type.cardtype & _T_SDHC)==0)
    {
      SDIO_ARG=sector<<9;
    }
    else
    {
      SDIO_ARG=sector;
    }
#else
    SDIO_ARG=sector<<9;
#endif

    SDIO_CMD |= MCICOMMAND_RESP;

    /* Clear static status flags. */
    SDIO_ICR = MCICLEAR_CMDCRCFAILCLR
                | MCICLEAR_CMDTIMEOUTCLR
                | MCICLEAR_CMDRESPENDCLR
                | MCICLEAR_CMDSENTCLR
                | MCICLEAR_DATACRCFAILCLR
                | MCICLEAR_DATATIMEOUTCLR
                | MCICLEAR_TXUNDERRUNCLR
                | MCICLEAR_RXOVERRUNCLR
                | MCICLEAR_DATAENDCLR
                | MCICLEAR_STARTBITERRCLR
                | MCICLEAR_DATABLOCKENDCLR;

    /* Command channel must be started right after data channel has been
       started. Otherwise we may have a data timeout. */
#if INTERRUPT_ENABLE
    /* Disable interrurp and save current state of the interrupt flags */
    cpu_sr=__get_CPSR();
    __disable_interrupt();
#endif
    /* Send command. */
    SDIO_CMD |= MCICOMMAND_ENABLE;
    SDIO_DCTRL |= MCIDATACTRL_ENABLE | (9<< 4);
#if INTERRUPT_ENABLE
    __set_CPSR(cpu_sr);
#endif

    /* Wait end of transfer. */
    while(1)
    {
      while(SDIO_STA & MCISTATUS_RXDATAAVLBL)
      {
        *(hcc_u32*)data=SDIO_FIFO;
        data=((hcc_u32*)data)+1;
      }

      if (SDIO_STA & (MCISTATUS_RXERRORS | MCISTATUS_CMDTIMEOUT
                       | MCISTATUS_DATAEND))
      {
        if (SDIO_STA & (MCISTATUS_RXERRORS | MCISTATUS_CMDTIMEOUT))
        {
          error=1;
        }
        break;
      }
    }

    if (multi)
    {
      break;
    }

    /* Check outcome. */
    if (error)
    {
      SDIO_DCTRL=0;
      return(MMC_ERR_CMD);
    }

    cnt--;
    sector++;
  }

  if (multi)
  {
    mmc_cmd_req_t cmd_req;

    cmd_req.cmd=STOP_TRANS_CMD;
    cmd_req.data=0;
    if (MCI_OK!=mci_cmd(&cmd_req))
    {
      return MMC_ERR_CMD;
    }

    /* Check outcome. */
    if (error)
    {
      SDIO_DCTRL=0;
      return(MMC_ERR_CMD);
    }
  }
  return MMC_NO_ERROR;
}
#endif

/*****************************************************************************
 * Name:
 *    mci_initfunc
 * In:
 *    driver_param - identifier for device identification (not used)
 * Out:
 *    F_DRIVER* - pointer to allocated driver descriptor.
 *
 * Description:
 *    Allocate and initialize new driver for specified device.
 *
 * Assumptions:
 *
 *****************************************************************************/
F_DRIVER *mci_initfunc(unsigned long driver_param)
{
  hcc_u32 tmp;


  /* GPIOC and GPIOD Periph clock enable */
  RCC_APB2ENR |= RCC_APB2_PERIPH_GPIOC | RCC_APB2_PERIPH_GPIOD ;
  RCC_AHBENR |=  RCC_AHBENR_PERIPH_DMA2EN;

  tmp = GPIOx_CRH(GPIO_C_BASE);
  tmp &= ~( (GPIO_MODE_OUT_OD << (PORT8+2))
  		  | (GPIO_MODE_OUT_OD << (PORT9+2))
  		  | (GPIO_MODE_OUT_OD << (PORT10+2))
  		  | (GPIO_MODE_OUT_OD << (PORT11+2))
  		  | (GPIO_MODE_OUT_OD << (PORT12+2)));
  tmp |=  ( ((GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT8))
  		  | ((GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT9))
  		  | ((GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT10))
  		  | ((GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT11))
  		  | ((GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT12)));
  GPIOx_CRH(GPIO_C_BASE) = tmp;

  tmp = GPIOx_CRL(GPIO_D_BASE);
  tmp &= ~(GPIO_MODE_OUT_OD << (PORT2+2));
  tmp |= (GPIO_SPEED_50MHz  | (GPIO_MODE_ALTF_PP <<2)) << (PORT2); /* PD2 */
  GPIOx_CRL(GPIO_D_BASE) = tmp;

  DMA_CCR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 0;
  DMA_CPAR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 512;
  DMA_CCR(DMA_2,DMA_CH_4) = 0;

  /* Power on card. */
  SDIO_POWER=MCIPOWER_PWR_UP;
  mmc_power(1);
  SDIO_POWER=MCIPOWER_PWR_ON;
  SDIO_ICR =~0;
  /* Fill driver descriptor. */
  mci_drv.readsector=mci_readsector;
  mci_drv.writesector=mci_writesector;
  mci_drv.readmultiplesector=mci_readmultiplesector;
  mci_drv.writemultiplesector=mci_writemultiplesector;
  mci_drv.getstatus=mci_getstatus;
  mci_drv.getphy=mci_getphy;
  mci_drv.release=mci_release;

  g_mmcsd_type.initok=0;
  return &mci_drv;

}

/****************************** END OF FILE **********************************/

