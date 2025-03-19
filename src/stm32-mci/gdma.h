#ifndef _GDMA_H_
#define _GDMA_H_

#include "../hcc_types.h"

/* DMA transfer widths */
#define DMA_TR_WIDTH8    0u
#define DMA_TR_WIDTH16   1u
#define DMA_TR_WIDTH32   2u

/* DMA peripheral IDs */
#define DMA_PI_SSP0_TX     0u
#define DMA_PI_SSP0_RX     1u
#define DMA_PI_SSP1_TX     2u
#define DMA_PI_SSP1_RX     3u
#define DMA_PI_MCI         4u
#define DMA_PI_I2S0        5u
#define DMA_PI_I2S1        6u
#define DMA_PI_SD          7u

/* DMA burst sizes */
#define DMA_BS_1    0u
#define DMA_BS_4    1u
#define DMA_BS_8    2u
#define DMA_BS_16   3u
#define DMA_BS_32   4u
#define DMA_BS_64   5u
#define DMA_BS_128  6u
#define DMA_BS_256  7u

typedef struct {
  void *addr;        /* address */
  hcc_u8 burst_size; /* burst size (DMA_BS_XXX) */
  hcc_u8 width;      /* transfer width to be used (DMA_TR_XXX)*/
  hcc_u8 increment;  /* set to one if address shall be incremented */
  hcc_u8 peripheral; /* peripheral id (DMA_PI_XXXX) */
} gdma_periph_info_t;


extern void gdma_init(void);
/* DMA flow control and transfer types.
   decode transfer type as:
          MM=memory to memory, MP= memory toperipheral
          PM= peripheral to memory, PP= peripheral to peripheral,
   decode flow control type as:
          DM= DMA controller
          DP= desttination peripheral
          SP= source peripheral
          P=  peripheral */
#define DMA_FW_MM_DM 0u
#define DMA_FW_MP_DM 1u
#define DMA_FW_PM_DM 2u
#define DMA_FW_PP_DM 3u
#define DMA_FW_PP_DP 4u
#define DMA_FW_MP_P  5u
#define DMA_FW_PM_P  6u
#define DMA_FW_PP_SP 7u
extern hcc_u8 gdma_start_channel(hcc_u8 chn /* 0 or 1 */
                     , gdma_periph_info_t *src
                     , gdma_periph_info_t *dst
                     , hcc_u16 t_size /* number of transfers to do */
                     , hcc_u8 flow);  /* DMA_FW_XX_YYY */

#define DMA_CHNST_IDLE  0
#define DMA_CHNST_ERROR 1
#define DMA_CHNST_BUSY  2
extern hcc_u8 gdma_get_status(hcc_u8 chn);
extern void gdma_stop_channel(hcc_u8 chn, hcc_u8 destructive);
extern void gdma_stop(void);
#endif
