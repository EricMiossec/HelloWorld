/*
    ChibiOS/RT - Copyright (C) 2006,2007,2008,2009,2010,
                 2011,2012 Giovanni Di Sirio.

    This file is part of ChibiOS/RT.

    ChibiOS/RT is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS/RT is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

                                      ---

    A special exception to the GPL can be applied should you wish to distribute
    a combined work that includes ChibiOS/RT, without being obliged to provide
    the source code for any proprietary components. See the file exception.txt
    for full details of how and when the exception can be applied.
*/

/**
 * @file    STM32F1xx/stm32_dma.h
 * @brief   DMA helper driver header.
 * @note    This file requires definitions from the ST header file stm32f10x.h.
 * @note    This driver uses the new naming convention used for the STM32F2xx
 *          so the "DMA channels" are referred as "DMA streams".
 *
 * @addtogroup STM32F1xx_DMA
 * @{
 */

#ifndef _STM32_DMA_H_
#define _STM32_DMA_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @brief   Total number of DMA streams.
 * @note    This is the total number of streams among all the DMA units.
 */
#if STM32_HAS_DMA2 || defined(__DOXYGEN__)
#define STM32_DMA_STREAMS           12
#else
#define STM32_DMA_STREAMS           7
#endif

/**
 * @brief   Mask of the ISR bits passed to the DMA callback functions.
 */
#define STM32_DMA_ISR_MASK          0x0F

/**
 * @brief   Returns the channel associated to the specified stream.
 *
 * @param[in] n         the stream number (0...STM32_DMA_STREAMS-1)
 * @param[in] c         a stream/channel association word, one channel per
 *                      nibble, not associated channels must be set to 0xF
 * @return              Always zero, in this platform there is no dynamic
 *                      association between streams and channels.
 */
#define STM32_DMA_GETCHANNEL(n, c)  0

/**
 * @brief   Returns a DMA stream identifier mask.
 *
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              A DMA stream identifier mask.
 */
#define STM32_DMA_STREAM_ID_MSK(dma, stream)                                \
  (1 << STM32_DMA_STREAM_ID(dma, stream))

/**
 * @brief   Checks if a DMA stream unique identifier belongs to a mask.
 * @param[in] id        the stream numeric identifier
 * @param[in] mask      the stream numeric identifiers mask
 *
 * @retval              The check result.
 * @retval FALSE        id does not belong to the mask.
 * @retval TRUE         id belongs to the mask.
 */
#define STM32_DMA_IS_VALID_ID(id, mask) (((1 << (id)) & (mask)))

/**
 * @name    DMA streams identifiers
 * @{
 */
/**
 * @brief   Returns an unique numeric identifier for a DMA stream.
 *
 * @param[in] dma       the DMA unit number
 * @param[in] stream    the stream number
 * @return              An unique numeric stream identifier.
 */
#define STM32_DMA_STREAM_ID(dma, stream) ((((dma) - 1) * 7) + ((stream) - 1))

/**
 * @brief   Returns a pointer to a stm32_dma_stream_t structure.
 *
 * @param[in] id        the stream numeric identifier
 * @return              A pointer to the stm32_dma_stream_t constant structure
 *                      associated to the DMA stream.
 */
#define STM32_DMA_STREAM(id)        (&_stm32_dma_streams[id])

#define STM32_DMA1_STREAM1          STM32_DMA_STREAM(0)
#define STM32_DMA1_STREAM2          STM32_DMA_STREAM(1)
#define STM32_DMA1_STREAM3          STM32_DMA_STREAM(2)
#define STM32_DMA1_STREAM4          STM32_DMA_STREAM(3)
#define STM32_DMA1_STREAM5          STM32_DMA_STREAM(4)
#define STM32_DMA1_STREAM6          STM32_DMA_STREAM(5)
#define STM32_DMA1_STREAM7          STM32_DMA_STREAM(6)
#define STM32_DMA2_STREAM1          STM32_DMA_STREAM(7)
#define STM32_DMA2_STREAM2          STM32_DMA_STREAM(8)
#define STM32_DMA2_STREAM3          STM32_DMA_STREAM(9)
#define STM32_DMA2_STREAM4          STM32_DMA_STREAM(10)
#define STM32_DMA2_STREAM5          STM32_DMA_STREAM(11)
/** @} */

/**
 * @name    CR register constants common to all DMA types
 * @{
 */
#define STM32_DMA_CR_EN             DMA_CCR1_EN
#define STM32_DMA_CR_TEIE           DMA_CCR1_TEIE
#define STM32_DMA_CR_HTIE           DMA_CCR1_HTIE
#define STM32_DMA_CR_TCIE           DMA_CCR1_TCIE
#define STM32_DMA_CR_DIR_MASK       (DMA_CCR1_DIR | DMA_CCR1_MEM2MEM)
#define STM32_DMA_CR_DIR_P2M        0
#define STM32_DMA_CR_DIR_M2P        DMA_CCR1_DIR
#define STM32_DMA_CR_DIR_M2M        DMA_CCR1_MEM2MEM
#define STM32_DMA_CR_CIRC           DMA_CCR1_CIRC
#define STM32_DMA_CR_PINC           DMA_CCR1_PINC
#define STM32_DMA_CR_MINC           DMA_CCR1_MINC
#define STM32_DMA_CR_PSIZE_MASK     DMA_CCR1_PSIZE
#define STM32_DMA_CR_PSIZE_BYTE     0
#define STM32_DMA_CR_PSIZE_HWORD    DMA_CCR1_PSIZE_0
#define STM32_DMA_CR_PSIZE_WORD     DMA_CCR1_PSIZE_1
#define STM32_DMA_CR_MSIZE_MASK     DMA_CCR1_MSIZE
#define STM32_DMA_CR_MSIZE_BYTE     0
#define STM32_DMA_CR_MSIZE_HWORD    DMA_CCR1_MSIZE_0
#define STM32_DMA_CR_MSIZE_WORD     DMA_CCR1_MSIZE_1
#define STM32_DMA_CR_SIZE_MASK      (STM32_DMA_CR_MSIZE_MASK |              \
                                     STM32_DMA_CR_MSIZE_MASK)
#define STM32_DMA_CR_PL_MASK        DMA_CCR1_PL
#define STM32_DMA_CR_PL(n)          ((n) << 12)
/** @} */

/**
 * @name    CR register constants only found in enhanced DMA
 * @{
 */
#define STM32_DMA_CR_DMEIE          0   /**< @brief Ignored by normal DMA.  */
#define STM32_DMA_CR_CHSEL_MASK     0   /**< @brief Ignored by normal DMA.  */
#define STM32_DMA_CR_CHSEL(n)       0   /**< @brief Ignored by normal DMA.  */
/** @} */

/**
 * @name    Status flags passed to the ISR callbacks
 * @{
 */
#define STM32_DMA_ISR_FEIF          0
#define STM32_DMA_ISR_DMEIF         0
#define STM32_DMA_ISR_TEIF          DMA_ISR_TEIF1
#define STM32_DMA_ISR_HTIF          DMA_ISR_HTIF1
#define STM32_DMA_ISR_TCIF          DMA_ISR_TCIF1
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   STM32 DMA stream descriptor structure.
 */
typedef struct {
  DMA_Channel_TypeDef   *channel;       /**< @brief Associated DMA channel. */
  volatile uint32_t     *ifcr;          /**< @brief Associated IFCR reg.    */
  uint8_t               ishift;         /**< @brief Bits offset in xIFCR
                                             register.                      */
  uint8_t               selfindex;      /**< @brief Index to self in array. */
  uint8_t               vector;         /**< @brief Associated IRQ vector.  */
} stm32_dma_stream_t;

/**
 * @brief   STM32 DMA ISR function type.
 *
 * @param[in] p         parameter for the registered function
 * @param[in] flags     pre-shifted content of the ISR register, the bits
 *                      are aligned to bit zero
 */
typedef void (*stm32_dmaisr_t)(void *p, uint32_t flags);

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @name    Macro Functions
 * @{
 */
/**
 * @brief   Associates a peripheral data register to a DMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @param[in] addr      value to be written in the CPAR register
 *
 * @special
 */
#define dmaStreamSetPeripheral(dmastp, addr) {                              \
  (dmastp)->channel->CPAR  = (uint32_t)(addr);                              \
}

/**
 * @brief   Associates a memory destination to a DMA stream.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @param[in] addr      value to be written in the CMAR register
 *
 * @special
 */
#define dmaStreamSetMemory0(dmastp, addr) {                                 \
  (dmastp)->channel->CMAR  = (uint32_t)(addr);                              \
}

/**
 * @brief   Sets the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @param[in] size      value to be written in the CNDTR register
 *
 * @special
 */
#define dmaStreamSetTransactionSize(dmastp, size) {                         \
  (dmastp)->channel->CNDTR  = (uint32_t)(size);                             \
}

/**
 * @brief   Returns the number of transfers to be performed.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @return              The number of transfers to be performed.
 *
 * @special
 */
#define dmaStreamGetTransactionSize(dmastp) ((size_t)((dmastp)->channel->CNDTR))

/**
 * @brief   Programs the stream mode settings.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @param[in] mode      value to be written in the CCR register
 *
 * @special
 */
#define dmaStreamSetMode(dmastp, mode) {                                    \
  (dmastp)->channel->CCR  = (uint32_t)(mode);                               \
}

/**
 * @brief   DMA stream enable.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamEnable(dmastp) {                                           \
  (dmastp)->channel->CCR |= STM32_DMA_CR_EN;                                \
}

/**
 * @brief   DMA stream disable.
 * @details The function disables the specified stream and then clears any
 *          pending interrupt.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamDisable(dmastp) {                                          \
  (dmastp)->channel->CCR &= ~STM32_DMA_CR_EN;                               \
  dmaStreamClearInterrupt(dmastp);                                          \
}

/**
 * @brief   DMA stream interrupt sources clear.
 * @note    This function can be invoked in both ISR or thread context.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 *
 * @special
 */
#define dmaStreamClearInterrupt(dmastp) {                                   \
  *(dmastp)->ifcr = STM32_DMA_ISR_MASK << (dmastp)->ishift;                 \
}

/**
 * @brief   Starts a memory to memory operation using the specified stream.
 * @note    The default transfer data mode is "byte to byte" but it can be
 *          changed by specifying extra options in the @p mode parameter.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 * @param[in] mode      value to be written in the CCR register, this value
 *                      is implicitly ORed with:
 *                      - @p STM32_DMA_CR_MINC
 *                      - @p STM32_DMA_CR_PINC
 *                      - @p STM32_DMA_CR_DIR_M2M
 *                      - @p STM32_DMA_CR_EN
 *                      .
 * @param[in] src       source address
 * @param[in] dst       destination address
 * @param[in] n         number of data units to copy
 */
#define dmaStartMemCopy(dmastp, mode, src, dst, n) {                        \
  dmaStreamSetPeripheral(dmastp, src);                                      \
  dmaStreamSetMemory0(dmastp, dst);                                         \
  dmaStreamSetTransactionSize(dmastp, n);                                   \
  dmaStreamSetMode(dmastp, (mode) |                                         \
                           STM32_DMA_CR_MINC | STM32_DMA_CR_PINC |          \
                           STM32_DMA_CR_DIR_M2M | STM32_DMA_CR_EN);         \
}

/**
 * @brief   Polled wait for DMA transfer end.
 * @pre     The stream must have been allocated using @p dmaStreamAllocate().
 * @post    After use the stream can be released using @p dmaStreamRelease().
 *
 * @param[in] dmastp    pointer to a stm32_dma_stream_t structure
 */
#define dmaWaitCompletion(dmastp)                                           \
  while ((dmastp)->channel->CNDTR > 0)                                      \
    ;                                                                       \
  dmaStreamDisable(dmastp);                                                 \
}

/** @} */

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if !defined(__DOXYGEN__)
extern const stm32_dma_stream_t _stm32_dma_streams[STM32_DMA_STREAMS];
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void dmaInit(void);
  bool_t dmaStreamAllocate(const stm32_dma_stream_t *dmastp,
                           uint32_t priority,
                           stm32_dmaisr_t func,
                           void *param);
  void dmaStreamRelease(const stm32_dma_stream_t *dmastp);
#ifdef __cplusplus
}
#endif

#endif /* _STM32_DMA_H_ */

/** @} */
