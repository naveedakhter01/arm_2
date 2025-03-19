/****************************************************************************
 *
 *            Copyright (c) 2003-2008 by HCC Embedded
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
 * Vaci Ut 110
 * Hungary
 *
 * Tel:  +36 (1) 450 1302
 * Fax:  +36 (1) 450 1303
 * http: www.hcc-embedded.com
 * email: info@hcc-embedded.com
 *
 ***************************************************************************/

#include "../stm32f10xreg.h"
#include "gdma.h"

static unsigned long dma_chn[]={0, DMA_CH_1, DMA_CH_2, DMA_CH_3, DMA_CH_4};

void gdma_init(void)
{
   /* Configure DMA channel */
  DMA_CCR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 0;
  DMA_CPAR(DMA_2,DMA_CH_4) = 0;
  DMA_CNDTR(DMA_2,DMA_CH_4) = 512;
  DMA_CCR(DMA_2,DMA_CH_4) = 0;
}


void gdma_stop_channel(hcc_u8 chn, hcc_u8 destructive)
{
  /* If channel is not enabled nothing to do. */
  if (!(DMA_CCR(DMA_2,dma_chn[chn]) & DMA_CCR_EN))
  {
    return;
  }

  /* If data in the fifo is allowed to be lost. */
  if (destructive)
  {
    /* Disable channel. */
    DMA_CCR(DMA_2,dma_chn[chn]) &= ~DMA_CCR_EN;
    while (DMA_ISR(DMA_2) & BIT5)
      ;
    DMA_IFCR(DMA_2) = DMA_ISR(DMA_2);
    DMA_CCR(DMA_2,dma_chn[chn]) = 0;
    return;
  }
  while (DMA_ISR(DMA_2) & BIT5)
  ;
  DMA_IFCR(DMA_2) = DMA_ISR(DMA_2);
  DMA_CCR(DMA_2,chn) = 0;
}

/* chn is channel number: 1,2,3,4,5,6,7 */
hcc_u8 gdma_start_channel(hcc_u8 chn, gdma_periph_info_t *src
                     , gdma_periph_info_t *dst
                     , hcc_u16 t_size
                     , hcc_u8 flow)
{

  /* Check parameters */
  if (src->peripheral > DMA_PI_SD || dst->peripheral > DMA_PI_SD
      || src->burst_size > 7 || src->width > DMA_TR_WIDTH32 || src->increment > 1
      || dst->burst_size > 7 || dst->width > DMA_TR_WIDTH32 || dst->increment > 1
      || flow > DMA_FW_PP_SP )
  {
    return(1);
  }
  DMA_CCR(DMA_2,dma_chn[chn]) &= ~DMA_CCR_EN;
  /* Clear interrupt status flags. */
  DMA_IFCR(DMA_2) = (8<<( ( (chn - 1) << 2) )) | (1<<( ( (chn - 1) << 2)));

  /* Set source and destination addresses */
  DMA_CMAR(DMA_2,dma_chn[chn]) = (hcc_u32)src->addr;
  DMA_CPAR(DMA_2,dma_chn[chn]) = (hcc_u32)dst->addr;

  DMA_CNDTR(DMA_2,dma_chn[chn]) = t_size / 4;
  DMA_IFCR(DMA_2) |= DMA_ISR_TCIF_2 | DMA_ISR_TCIF_3;                 
  DMA_CCR(DMA_2,dma_chn[chn]) |= DMA_CCR_MINC;
  DMA_CCR(DMA_2,dma_chn[chn]) &= ~DMA_CCR_PINC;
  if (flow ==  DMA_FW_MP_P)
  {
      DMA_CCR(DMA_2,dma_chn[chn]) |= DMA_CCR_MEM_TO_PERIPH;
  }
  else if (flow ==  DMA_FW_PM_P)
  {
      DMA_CCR(DMA_2,dma_chn[chn]) &= ~DMA_CCR_MEM_TO_PERIPH;
  }
  DMA_CCR(DMA_2,dma_chn[chn]) |= DMA_CCR_EN | BIT9 | BIT11 | BIT13;
  return 0;
}

hcc_u8 gdma_get_status(hcc_u8 chn)
{
  if (DMA_ISR(DMA_2) & (8<<( ( (chn - 1) << 2) ) ) ? 1 : 0)
  {
    return(DMA_CHNST_ERROR);
  }

  if (DMA_ISR(DMA_2) & (1<<( ( (chn - 1) << 2))) ? 0 : 1)
  {
    return(DMA_CHNST_BUSY);
  }

  return(DMA_CHNST_IDLE);
}

void gdma_stop(void)
{
  gdma_stop_channel(4, 1);
}
