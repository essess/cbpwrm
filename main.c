/**
 * Developed by: Sean Stasiak <sstasiak@gmail.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 *
 * (C)AN(B)ody(P)o(W)e(R)(M)odule:
 * Generic & wake inputs for controlling BODY+ power
 * A first attempt to reduce harness bulk in trade for firmware complexity.
 *
 * Built upon: https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill with
 * a C6T6 onboard (32k flash, 10k sram)
 *
 * IN USE:
 *    PA.11 - CAN_RX
 *    PA.12 - CAN_TX
 *    PA.13 - SWDIO
 *    PA.14 - SWCLK
 *    PA.15 - HOLD
 *    PA.0  - DIN0
 *    PA.1  - DIN1
 *    PA.2  - DIN2
 *    PA.3  - DIN3
 *    PA.4  - DIN4
 *    PA.5  - DIN5
 *    PA.6  - DIN6
 *    PA.7  - DIN7
 *    PA.8  - DOUT0 (BODY+ SSR DRIVER)
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <debugio.h>
#include "stm32f1xx.h"
#include "system_stm32f1xx.h"
#include "pt.h"
#include "pt-sem.h"

/** ---------------------------------------------------------------------------
 * @internal
 * @brief constants
 */
#define INC(var)  do { if((var) < (unsigned)~0) (var)++; } while(0)
#define DEC(var)  do { if((var)               ) (var)--; } while(0)

enum _msgid_t {                   /*! FIXED MSG IDs - can use for static filters */
  ID_CBPWRM_PO         = 0x1A0,                      /**< module just powered up */
  ID_CBPWRM_STATE      = 0x1A1,  /**< current DIN state (periodic and on change) */
  ID_CBPWRM_TIME       = 0x1A2,            /**< current system time (by request) */
  ID_CBPWRM_LOOPS      = 0x1A3,                    /**< loops count (by request) */
  ID_CBPWRM_LOOPSREQ   = 0x1A6,         /**< loops request from external modules */
  ID_CBPWRM_TIMEREQ    = 0x1A7,   /**< system time request from external modules */
  ID_CBPWRM_HOLDREQ    = 0x1A8,         /**< HOLD 'tickle' from external modules */
} msgid_t;

enum _constants_t {          /*! CONSTANTS                       */
  LOCKOUT         = 20,      /**< on_change/req throttle         */
  HOLD_TIMEOUT    = 10000,   /**< 10s minimum timeout            */
  HOLD_WARN       = 3000,    /**< 3s poweroff warning            */
  STATUS_RATE     = 60,      /**< ~17Hz periodic (and on change) */
  DIN_GLITCH_WAIT = 6,

  FLAG_DIN0_ACTIVE     = (1<< 0),
  FLAG_DIN1_ACTIVE     = (1<< 1),
  FLAG_DIN2_ACTIVE     = (1<< 2),
  FLAG_DIN3_ACTIVE     = (1<< 3),
  FLAG_DIN4_ACTIVE     = (1<< 4),
  FLAG_DIN5_ACTIVE     = (1<< 5),
  FLAG_DIN6_ACTIVE     = (1<< 6),
  FLAG_DIN7_ACTIVE     = (1<< 7),
  FLAG_HOLD_ACTIVE     = (1<< 8),
  FLAG_HOLD_POWARN     = (1<< 9),    /**< poweroff warning window active */
  FLAG_HOLDREQ         = (1<<12),
  FLAG_TIMEREQ         = (1<<13),
  FLAG_LOOPSREQ        = (1<<14),

  CAN2SB            = 14,       /**< normally 0x0e (14) out of reset     */
  CAN1FILTBANK_SET6 = CAN2SB-5, /**< CAN1 filter bank _SET6 (of 14)      */
  CAN1FILTBANK_SET5 = CAN2SB-4, /**< CAN1 filter bank _SET5 (of 14)      */
  CAN1FILTBANK_SET4 = CAN2SB-3, /**< CAN1 filter bank _SET4 (of 14)      */
  CAN1FILTBANK_SET3 = CAN2SB-2, /**< CAN1 filter bank _SET3 (of 14)      */
  CAN1FILTBANK_SET1 = CAN2SB-1, /**< CAN1 filter bank _SET1 (of 14)      */
  CAN2FILTBANK_SET1 = CAN2SB+0, /**< CAN2 filter bank _SET1 (of 14)      */
                                /*   place 4 standard id's per set; list */
                                /*   mode (exact) matching used. add     */
                                /*   more sets as needed                 */
  FIFO0 = 0b0,
  FIFO1 = 0b1,
  FIFO_CNT,
  FIFO_UNKNOWN = FIFO_CNT,

  TXMBOX0  = 0,
  TXMBOX1  = 1,
  TXMBOX2  = 2,
  TXMBOX_CNT,
  TXMBOX_UNKNOWN = TXMBOX_CNT,

} constants_t;

/** ---------------------------------------------------------------------------
 * @internal
 * @brief protos/fwdrefs/data
 */
typedef struct _tmr_t {     /*! application timer */
  uint32_t interval;
  uint32_t start;
} tmr_t;

typedef struct _frame_t {   /*! generic CAN frame */
  uint16_t id;
  uint8_t  dlc;
  uint8_t  data[8];
} frame_t;

typedef struct _ptdin_t {   /*! din thread data wrapper */
  struct pt pt;
  unsigned (*din_read)(void);
  tmr_t t;
  unsigned prev;
  unsigned mask;
} ptdin_t;

typedef uint32_t flags_t;

static flags_t flags;
static uint32_t ticks;      /**< rolling timer tick counter */
static uint32_t ms;         /**< 0-999 millseconds counter  */
static uint32_t sec;        /**< 0-UINT_MAX seconds counter */
static uint32_t loops;      /**< mainloop counter           */

static int hold_requested(void);

/** ---------------------------------------------------------------------------
 * @internal
 * @brief re/set an application timer
 *        MAX interval is UINT32_MAX ticks (currently 1ms increments)
 */
__STATIC_FORCEINLINE void
  tmr_set(tmr_t *t, uint32_t interval) {
  assert(interval), assert(interval < UINT32_MAX);
  t->interval = interval;
  t->start    = ticks;
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief evaluate an application timer
 */
__STATIC_FORCEINLINE int
  tmr_done(tmr_t *t) {
  return (ticks - t->start) >= (t->interval);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief timebase
 */
void
  SysTick_Handler(void)
{
  ticks++;
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief non-blocking send of a CAN frame.
 *        returns zero if mailboxes are full, non-zero otherwise
 */
static unsigned
  can_tx(frame_t *pf)
{
  assert(pf);
  assert(pf->id < 2048);
  assert(pf->dlc <= 8);

  uint32_t const tsr = CAN1->TSR;

  if(tsr & CAN_TSR_TME0) {  /**< TXMBOX0 has room? */
    CAN1->sTxMailBox[TXMBOX0].TDTR = pf->dlc;
    CAN1->sTxMailBox[TXMBOX0].TDLR = (pf->data[0] << 0) | (pf->data[1] << 8) |
                                     (pf->data[2] <<16) | (pf->data[3] <<24);
    CAN1->sTxMailBox[TXMBOX0].TDHR = (pf->data[4] << 0) | (pf->data[5] << 8) |
                                     (pf->data[6] <<16) | (pf->data[7] <<24);
    CAN1->sTxMailBox[TXMBOX0].TIR  = CAN_TI0R_TXRQ | (pf->id << CAN_TI0R_STID_Pos);
    return ~0;
  }

  if(tsr & CAN_TSR_TME1) {  /**< TXMBOX1 has room? */
    CAN1->sTxMailBox[TXMBOX1].TDTR = pf->dlc;
    CAN1->sTxMailBox[TXMBOX1].TDLR = (pf->data[0] << 0) | (pf->data[1] << 8) |
                                     (pf->data[2] <<16) | (pf->data[3] <<24);
    CAN1->sTxMailBox[TXMBOX1].TDHR = (pf->data[4] << 0) | (pf->data[5] << 8) |
                                     (pf->data[6] <<16) | (pf->data[7] <<24);
    CAN1->sTxMailBox[TXMBOX1].TIR  = CAN_TI1R_TXRQ | (pf->id << CAN_TI1R_STID_Pos);
    return ~0;
  }

  if(tsr & CAN_TSR_TME2) {  /**< TXMBOX2 has room? */
    CAN1->sTxMailBox[TXMBOX2].TDTR = pf->dlc;
    CAN1->sTxMailBox[TXMBOX2].TDLR = (pf->data[0] << 0) | (pf->data[1] << 8) |
                                     (pf->data[2] <<16) | (pf->data[3] <<24);
    CAN1->sTxMailBox[TXMBOX2].TDHR = (pf->data[4] << 0) | (pf->data[5] << 8) |
                                     (pf->data[6] <<16) | (pf->data[7] <<24);
    CAN1->sTxMailBox[TXMBOX2].TIR  = CAN_TI2R_TXRQ | (pf->id << CAN_TI2R_STID_Pos);
    return ~0;
  }

  return 0; /* FULL */
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief non-blocking recv of a CAN frame.
 *        returns zero if FIFOs are empty, non-zero otherwise
 */
static unsigned
  can_rx(frame_t *pf)
{
  assert(pf);

  /* do anything if an overflow is detected? (CAN_RF0R_FOVR0) */
  if(CAN1->RF0R & CAN_RF0R_FMP0) {
    pf->id  = (CAN_RI0R_STID & CAN1->sFIFOMailBox[FIFO0].RIR)  >> CAN_TI0R_STID_Pos;
    pf->dlc = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[FIFO0].RDTR) >> CAN_RDT0R_DLC_Pos;
    unsigned const rdlr = CAN1->sFIFOMailBox[FIFO0].RDLR;
    pf->data[0] = (CAN_RDL0R_DATA0 & rdlr) >> CAN_RDL0R_DATA0_Pos;
    pf->data[1] = (CAN_RDL0R_DATA1 & rdlr) >> CAN_RDL0R_DATA1_Pos;
    pf->data[2] = (CAN_RDL0R_DATA2 & rdlr) >> CAN_RDL0R_DATA2_Pos;
    pf->data[3] = (CAN_RDL0R_DATA3 & rdlr) >> CAN_RDL0R_DATA3_Pos;
    unsigned const rdhr = CAN1->sFIFOMailBox[FIFO0].RDHR;
    pf->data[4] = (CAN_RDH0R_DATA4 & rdhr) >> CAN_RDH0R_DATA4_Pos;
    pf->data[5] = (CAN_RDH0R_DATA5 & rdhr) >> CAN_RDH0R_DATA5_Pos;
    pf->data[6] = (CAN_RDH0R_DATA6 & rdhr) >> CAN_RDH0R_DATA6_Pos;
    pf->data[7] = (CAN_RDH0R_DATA7 & rdhr) >> CAN_RDH0R_DATA7_Pos;
    SET_BIT(CAN1->RF0R, CAN_RF0R_RFOM0), __DSB();
    return ~0;
  }

  /* do anything if an overflow is detected? (CAN_RF1R_FOVR1) */
  if(CAN1->RF1R & CAN_RF1R_FMP1) {
    pf->id  = (CAN_RI0R_STID & CAN1->sFIFOMailBox[FIFO1].RIR)  >> CAN_TI0R_STID_Pos;
    pf->dlc = (CAN_RDT0R_DLC & CAN1->sFIFOMailBox[FIFO0].RDTR) >> CAN_RDT0R_DLC_Pos;
    unsigned const rdlr = CAN1->sFIFOMailBox[FIFO1].RDLR;
    pf->data[0] = (CAN_RDL0R_DATA0 & rdlr) >> CAN_RDL0R_DATA0_Pos;
    pf->data[1] = (CAN_RDL0R_DATA1 & rdlr) >> CAN_RDL0R_DATA1_Pos;
    pf->data[2] = (CAN_RDL0R_DATA2 & rdlr) >> CAN_RDL0R_DATA2_Pos;
    pf->data[3] = (CAN_RDL0R_DATA3 & rdlr) >> CAN_RDL0R_DATA3_Pos;
    unsigned const rdhr = CAN1->sFIFOMailBox[FIFO1].RDHR;
    pf->data[4] = (CAN_RDH0R_DATA4 & rdhr) >> CAN_RDH0R_DATA4_Pos;
    pf->data[5] = (CAN_RDH0R_DATA5 & rdhr) >> CAN_RDH0R_DATA5_Pos;
    pf->data[6] = (CAN_RDH0R_DATA6 & rdhr) >> CAN_RDH0R_DATA6_Pos;
    pf->data[7] = (CAN_RDH0R_DATA7 & rdhr) >> CAN_RDH0R_DATA7_Pos;
    SET_BIT(CAN1->RF1R, CAN_RF1R_RFOM1), __DSB();
    return ~0;
  }

  return 0;
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief manage system time (S:MS)
 */
static
  PT_THREAD(time(struct pt *pt))
{
  static uint32_t _curr, _prev, _delta;
  PT_BEGIN(pt);
  _curr  = ticks; /* simple atomic read */
  _delta = _curr - _prev;

  ms += _delta;
  if(ms > 999) { /**< catch rollover */
    ms %= 1000;
    INC(sec);
  }
  _prev = _curr;
  PT_END(pt);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief track loops/100ms (emitted via status)
 */
static
  PT_THREAD(loopcnt(struct pt *pt))
{
  static tmr_t t;
  static uint32_t _loops = 0;
  PT_BEGIN(pt);
  tmr_set(&t, 100);              /**< 10Hz counter update rate */
  while(~0) {
    if(tmr_done(&t)) {
      loops = _loops, _loops = 0;    /**< xfer counter & reset */
      PT_RESTART(pt);
    }
    _loops++;
    PT_YIELD(pt);
  }
  PT_END(pt);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief 0b0 == !ACTIVE, 0b1 == ACTIVE
 */
static unsigned
  din0_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR0)?0b0:0b1;
}
static unsigned
  din1_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR1)?0b0:0b1;
}
static unsigned
  din2_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR2)?0b0:0b1;
}
static unsigned
  din3_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR3)?0b0:0b1;
}
static unsigned
  din4_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR4)?0b0:0b1;
}
static unsigned
  din5_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR5)?0b0:0b1;
}
static unsigned
  din6_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR6)?0b0:0b1;
}
static unsigned
  din7_read(void)
{
  return READ_BIT(GPIOA->IDR, GPIO_IDR_IDR7)?0b0:0b1;
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief DINx debouncing and state tracking
 */
static
  PT_THREAD(din(struct pt *pt))
{
  ptdin_t * const ptdin = (ptdin_t*)pt;

  PT_BEGIN(pt);

  /* wait for an edge */
  ptdin->prev = READ_BIT(flags, ptdin->mask)?0b1:0b0;
  PT_WAIT_UNTIL(pt, ptdin->prev ^ ptdin->din_read());

  /* wait/detct glitch */
  tmr_set(&ptdin->t, DIN_GLITCH_WAIT);
  PT_WAIT_UNTIL(pt, tmr_done(&ptdin->t));
  if(!(ptdin->prev ^ ptdin->din_read())) { PT_RESTART(pt); }

  /* wait/detct glitch */
  tmr_set(&ptdin->t, DIN_GLITCH_WAIT);
  PT_WAIT_UNTIL(pt, tmr_done(&ptdin->t));
  if(!(ptdin->prev ^ ptdin->din_read())) { PT_RESTART(pt); }

  /* wait/detct glitch */
  tmr_set(&ptdin->t, DIN_GLITCH_WAIT);
  PT_WAIT_UNTIL(pt, tmr_done(&ptdin->t));
  if(!(ptdin->prev ^ ptdin->din_read())) { PT_RESTART(pt); }

  /* wait/detct glitch */
  tmr_set(&ptdin->t, DIN_GLITCH_WAIT);
  PT_WAIT_UNTIL(pt, tmr_done(&ptdin->t));
  if(!(ptdin->prev ^ ptdin->din_read())) { PT_RESTART(pt); }

  /* debounced, make it official after 5x in a row */
  MODIFY_REG(flags, ptdin->mask, ptdin->din_read()?ptdin->mask:0);

  PT_END(pt);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief process received CAN frames
 */
static
  PT_THREAD(can(struct pt *pt))
{
  static frame_t f;
  PT_BEGIN(pt);
  /* pull until FIFOs empty */
  while(can_rx(&f)) {
    switch(f.id) {
      case ID_CBPWRM_HOLDREQ:
        SET_BIT(flags, FLAG_HOLDREQ);
        break;
      case ID_CBPWRM_TIMEREQ:
        SET_BIT(flags, FLAG_TIMEREQ);
        break;
      case ID_CBPWRM_LOOPSREQ:
        SET_BIT(flags, FLAG_LOOPSREQ);
        break;
      default:
        assert(!"unhandled frame");
    }
  }
  PT_END(pt);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief on-board led control
 */
__STATIC_FORCEINLINE void
  led_on(void) {
  GPIOC->BSRR = GPIO_BSRR_BR13;
}
__STATIC_FORCEINLINE void
  led_off(void) {
  GPIOC->BSRR = GPIO_BSRR_BS13;
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief if blinking, then good chance main loop ok/unblocked.
 */
static
  PT_THREAD(blink(struct pt *pt))
{
  static tmr_t t;
  PT_BEGIN(pt);
  while(~0) {
    led_on();
    tmr_set(&t, 150);
    PT_WAIT_UNTIL(pt, tmr_done(&t));
    led_off();
    tmr_set(&t, 350);
    PT_WAIT_UNTIL(pt, tmr_done(&t));
  }
  PT_END(pt);
}


/** ---------------------------------------------------------------------------
 * @internal
 * @brief dout control
 */
__STATIC_FORCEINLINE void
  dout_on(void) {
  GPIOA->BSRR = GPIO_BSRR_BR8;
}
__STATIC_FORCEINLINE void
  dout_off(void) {
  GPIOA->BSRR = GPIO_BSRR_BS8;
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief simple dout control - placeholder for other forms of future control
 */
static
  PT_THREAD(dout(struct pt *pt))
{
  PT_BEGIN(pt);
  PT_END(pt);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief supply control
 */
__STATIC_FORCEINLINE void
  hold_on(void) {
  GPIOA->BSRR = GPIO_BSRR_BS15;
  SET_BIT(flags, FLAG_HOLD_ACTIVE);
}
__STATIC_FORCEINLINE void
  hold_off(void) {
  GPIOA->BSRR = GPIO_BSRR_BR15;
  CLEAR_BIT(flags, FLAG_HOLD_ACTIVE);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief general watchdog that holds power on for a specified time
 */
static_assert(HOLD_TIMEOUT > HOLD_WARN);
static
  PT_THREAD(hold(struct pt *pt))
{
  static tmr_t t;

  PT_BEGIN(pt);
  if(hold_requested()) {
    hold_on();
    tmr_set(&t, HOLD_TIMEOUT-HOLD_WARN);
    CLEAR_BIT(flags, FLAG_HOLD_POWARN);
    PT_RESTART(pt);
  }

  if(tmr_done(&t)) {
    if(!(flags & FLAG_HOLD_POWARN)) {      /**< in warning period?  */
      SET_BIT(flags, FLAG_HOLD_POWARN);
      tmr_set(&t, HOLD_WARN);
      PT_RESTART(pt);
    }                                     /* else, warning expired */
    hold_off();                           /**< nite nite!          */
  }
  PT_END(pt);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief emit current state periodically, or upon change (if not locked out)
 */
static_assert(LOCKOUT < STATUS_RATE);
static
  PT_THREAD(status(struct pt *pt))
{
  static tmr_t t;
  static frame_t frame;
  static flags_t prev = 0; /**< guarantee emit on first pass through */

  PT_BEGIN(pt);
  frame.id = ID_CBPWRM_PO;
  frame.dlc = 0;
  PT_WAIT_UNTIL(pt, can_tx(&frame));
  while(~0) {
    PT_WAIT_UNTIL(pt, (flags ^ prev) || tmr_done(&t));
    tmr_set(&t, LOCKOUT);
    frame.id = ID_CBPWRM_STATE;
    frame.dlc = 2;
    frame.data[0] = flags >> 8;    /*! BIG ENDIAN !*/
    frame.data[1] = flags >> 0;
    prev = flags;
    PT_WAIT_UNTIL(pt, can_tx(&frame));
    PT_WAIT_UNTIL(pt, tmr_done(&t));
    tmr_set(&t, STATUS_RATE - LOCKOUT);
  }
  PT_END(pt);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief emit system time when/if requested
 */
static
  PT_THREAD(timereq(struct pt *pt))
{
  static tmr_t t;
  static frame_t frame;

  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, flags & FLAG_TIMEREQ);
  CLEAR_BIT(flags, FLAG_TIMEREQ);
  tmr_set(&t, LOCKOUT);
  frame.id = ID_CBPWRM_TIME;
  frame.dlc = 6;
  frame.data[0] = sec >> 24;  /*! BIG ENDIAN (seconds)      !*/
  frame.data[1] = sec >> 16;
  frame.data[2] = sec >> 8;
  frame.data[3] = sec >> 0;
  frame.data[4] = ms  >> 8;   /*! BIG ENDIAN (milliseconds) !*/
  frame.data[5] = ms  >> 0;
  PT_WAIT_UNTIL(pt, can_tx(&frame));
  PT_WAIT_UNTIL(pt, tmr_done(&t));
  PT_END(pt);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief emit loops when/if requested
 */
static
  PT_THREAD(loopsreq(struct pt *pt))
{
  static tmr_t t;
  static frame_t frame;

  PT_BEGIN(pt);
  PT_WAIT_UNTIL(pt, flags & FLAG_LOOPSREQ);
  CLEAR_BIT(flags, FLAG_LOOPSREQ);
  tmr_set(&t, LOCKOUT);
  frame.id = ID_CBPWRM_LOOPS;
  frame.dlc = 2;
  frame.data[0] = loops >> 8;    /*! BIG ENDIAN (hz := loops*10) !*/
  frame.data[1] = loops >> 0;
  PT_WAIT_UNTIL(pt, can_tx(&frame));
  PT_WAIT_UNTIL(pt, tmr_done(&t));
  PT_END(pt);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init LED (PC.13)
 */
static void
  led_init(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPCEN); __DSB();
  led_off();
  MODIFY_REG(GPIOC->CRH, GPIO_CRH_MODE13 | GPIO_CRH_CNF13,
                         (2<<GPIO_CRH_MODE13_Pos) | (1<<GPIO_CRH_CNF13_Pos));
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init internal HOLD line (PA.15)
 */
static void
  hold_init(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); __DSB();
  hold_on();
  MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE15 | GPIO_CRH_CNF15,
                        (2<<GPIO_CRH_MODE15_Pos) | (0<<GPIO_CRH_CNF15_Pos));
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief initialize all digital inputs
 */
static void
  din_init(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);  __DSB();
  MODIFY_REG(GPIOA->CRL, GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF2 |
                         GPIO_CRL_CNF3 | GPIO_CRL_CNF4 | GPIO_CRL_CNF5 |
                         GPIO_CRL_CNF6 | GPIO_CRL_CNF7,
                         (2<<GPIO_CRL_CNF0_Pos) | (2<<GPIO_CRL_CNF1_Pos) |
                         (2<<GPIO_CRL_CNF2_Pos) | (2<<GPIO_CRL_CNF3_Pos) |
                         (2<<GPIO_CRL_CNF4_Pos) | (2<<GPIO_CRL_CNF5_Pos) |
                         (2<<GPIO_CRL_CNF6_Pos) | (2<<GPIO_CRL_CNF7_Pos) );
  GPIOA->BSRR = 0x000000FF;
  CLEAR_BIT(flags, FLAG_DIN0_ACTIVE | FLAG_DIN1_ACTIVE | FLAG_DIN3_ACTIVE | FLAG_DIN3_ACTIVE |
                   FLAG_DIN4_ACTIVE | FLAG_DIN5_ACTIVE | FLAG_DIN6_ACTIVE | FLAG_DIN7_ACTIVE );
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init dout/ssr driver (PA.8)
 */
static void
  dout_init(void)
{
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN); __DSB();
  dout_on();
  MODIFY_REG(GPIOA->CRH, GPIO_CRH_MODE8 | GPIO_CRH_CNF8,
                        (2<<GPIO_CRH_MODE8_Pos) | (1<<GPIO_CRH_CNF8_Pos));
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief initialize can hardware
 */
static void
  can_init(void)
{
  MODIFY_REG(RCC->APB1ENR, RCC_APB1ENR_CAN1EN, RCC_APB1ENR_CAN1EN);
  MODIFY_REG(AFIO->MAPR, AFIO_MAPR_CAN_REMAP, 0b00<<AFIO_MAPR_CAN_REMAP_Pos);
  SET_BIT(RCC->APB2ENR, RCC_APB2ENR_IOPAEN);

  SET_BIT(CAN1->MCR, CAN_MCR_RESET);
  while(CAN1->MSR & CAN_MCR_RESET);

  SET_BIT(CAN1->MCR, CAN_MCR_ABOM);

  /* CAN1:  PA.11 <- CAN_RX (IN)   APB1 peripherals clk'd at 36MHz         */
  /*        PA.12 -> CAN_TX (OUT)                                          */
  uint32_t reg = GPIOA->CRH;
  MODIFY_REG(reg, GPIO_CRH_CNF11, (0b10<<GPIO_CRH_CNF11_Pos));
  MODIFY_REG(reg, GPIO_CRH_CNF12, (0b10<<GPIO_CRH_CNF12_Pos));
  GPIOA->CRH = reg;                /**< p-p tx, pu/pd rx                   */
  GPIOA->BSRR = GPIO_BSRR_BS11;    /**< rx = pu                            */

  reg = GPIOA->CRH;
  MODIFY_REG(reg, GPIO_CRH_MODE11, (0b00<<GPIO_CRH_MODE11_Pos));
  MODIFY_REG(reg, GPIO_CRH_MODE12, (0b01<<GPIO_CRH_MODE12_Pos));
  GPIOA->CRH = reg;                /**< switch in CAN1 on pins             */

  assert(CAN1->MSR & CAN_MSR_SLAK);    /**< is in sleep                    */
  CLEAR_BIT(CAN1->MCR, CAN_MCR_SLEEP);
  while((CAN1->MSR & CAN_MSR_SLAK) ^ (0b0<<CAN_MSR_SLAK_Pos));
  assert(!(CAN1->MSR & CAN_MSR_SLAK)); /**< now awake                      */

  assert(!(CAN1->MSR & CAN_MSR_INAK)); /**< not in init                    */
  SET_BIT(CAN1->MCR, CAN_MCR_INRQ);
  while((CAN1->MSR & CAN_MSR_INAK) ^ (0b1<<CAN_MSR_INAK_Pos));
  assert(CAN1->MSR & CAN_MSR_INAK);    /**< in init mode                   */

  reg = CAN1->BTR;           /**< set 1MBps speed (assumes 36MHz AHB1 clk) */
  MODIFY_REG(reg, CAN_BTR_SJW, ((2-1)<<CAN_BTR_SJW_Pos));
  MODIFY_REG(reg, CAN_BTR_TS1, ((4-1)<<CAN_BTR_TS1_Pos));
  MODIFY_REG(reg, CAN_BTR_TS2, ((1-1)<<CAN_BTR_TS2_Pos));
  MODIFY_REG(reg, CAN_BTR_BRP, ((6-1)<<CAN_BTR_BRP_Pos));
  CAN1->BTR = reg;

  /* filter init begin ----------------------------------------------------- */
  SET_BIT(CAN1->FMR, CAN_FMR_FINIT);
  assert(CAN2SB == ((CAN1->FMR & CAN_FMR_CAN2SB_Msk) >> CAN_FMR_CAN2SB_Pos));

  /* configure CAN1FILTBANK_SET1                    */
  /* set identifier list mode (exact match on id),  */
  MODIFY_REG(CAN1->FM1R, CAN_FM1R_FBM0<<CAN1FILTBANK_SET1, (0b1)<<CAN1FILTBANK_SET1);
  /* dual 16b scaling (4 standard id's per set),    */
  MODIFY_REG(CAN1->FS1R, CAN_FS1R_FSC0<<CAN1FILTBANK_SET1, (0b0)<<CAN1FILTBANK_SET1);
  /* assign the results of this bank to RX FIFO 0,  */
  MODIFY_REG(CAN1->FFA1R, CAN_FFA1R_FFA0<<CAN1FILTBANK_SET1, FIFO0<<CAN1FILTBANK_SET1);

  reg = CAN1->FA1R;
  MODIFY_REG(reg, CAN_FA1R_FACT0<<CAN1FILTBANK_SET1, (0b1)<<CAN1FILTBANK_SET1);
  CAN1->FA1R = reg;  /**< activate this filter bank */
  CAN1->sFilterRegister[CAN1FILTBANK_SET1].FR1 = (((ID_CBPWRM_HOLDREQ<<5)<< 0) |
                                                  ((ID_CBPWRM_TIMEREQ<<5)<<16));
  CAN1->sFilterRegister[CAN1FILTBANK_SET1].FR2 = (((ID_CBPWRM_LOOPSREQ<<5)<< 0) |
                                                  ((ID_CBPWRM_LOOPSREQ<<5)<<16));

  /* continue configure of CAN1FILTBANK_SETn as needed .. */
  CLEAR_BIT(CAN1->FMR, CAN_FMR_FINIT);
  /* filter init end ------------------------------------------------------- */

  assert(CAN1->MSR & CAN_MSR_INAK);    /**< in init mode                     */
  CLEAR_BIT(CAN1->MCR, CAN_MCR_INRQ);
  while((CAN1->MSR & CAN_MSR_INAK) ^ (0b0<<CAN_MSR_INAK_Pos));
  assert(!(CAN1->MSR & CAN_MSR_INAK)); /**< not in init                      */
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief lock GPIO configuration to its final function
 */
static void
  lock_io(void)
{
  assert(GPIOA->LCKR == 0);
  assert(GPIOC->LCKR == 0);

  enum {
    LOCKALL_LCKK0 = 0x0000FFFF,
    LOCKALL_LCKK1 = 0x0001FFFF
  };

  GPIOA->LCKR = LOCKALL_LCKK1;
  GPIOA->LCKR = LOCKALL_LCKK0;
  GPIOA->LCKR = LOCKALL_LCKK1;
  assert(GPIOA->LCKR);
  assert(GPIOA->LCKR & 0x00010000);

  GPIOC->LCKR = LOCKALL_LCKK1;
  GPIOC->LCKR = LOCKALL_LCKK0;
  GPIOC->LCKR = LOCKALL_LCKK1;
  assert(GPIOC->LCKR);
  assert(GPIOC->LCKR & 0x00010000);
}

/** -------------------------------------------------------------------------
 * @public
 * @brief entry point
 */
__NO_RETURN void
  main(void)
 {
  SystemCoreClockUpdate(), assert(SystemCoreClock = 72000000);

  hold_init();
  dout_init();
  din_init();
  can_init();
  led_init();

  static struct pt _time;    PT_INIT(&_time);
  static struct pt _loopcnt; PT_INIT(&_loopcnt);
  static ptdin_t _din[8] = {
    [0] = { .din_read = &din0_read, .mask = FLAG_DIN0_ACTIVE },
    [1] = { .din_read = &din1_read, .mask = FLAG_DIN1_ACTIVE },
    [2] = { .din_read = &din2_read, .mask = FLAG_DIN2_ACTIVE },
    [3] = { .din_read = &din3_read, .mask = FLAG_DIN3_ACTIVE },
    [4] = { .din_read = &din4_read, .mask = FLAG_DIN4_ACTIVE },
    [5] = { .din_read = &din5_read, .mask = FLAG_DIN5_ACTIVE },
    [6] = { .din_read = &din6_read, .mask = FLAG_DIN6_ACTIVE },
    [7] = { .din_read = &din7_read, .mask = FLAG_DIN7_ACTIVE },
  };
  PT_INIT(&_din[0].pt);
  PT_INIT(&_din[1].pt);
  PT_INIT(&_din[2].pt);
  PT_INIT(&_din[3].pt);
  PT_INIT(&_din[4].pt);
  PT_INIT(&_din[5].pt);
  PT_INIT(&_din[6].pt);
  PT_INIT(&_din[7].pt);
  static struct pt _can;      PT_INIT(&_can);
  static struct pt _dout;     PT_INIT(&_dout);
  static struct pt _blink;    PT_INIT(&_blink);
  static struct pt _hold;     PT_INIT(&_hold);
  static struct pt _status;   PT_INIT(&_status);
  static struct pt _timereq;  PT_INIT(&_timereq);
  static struct pt _loopsreq; PT_INIT(&_loopsreq);

  SysTick_Config(SystemCoreClock/1000);
  NVIC_SetPriority(SysTick_IRQn, 0);
  lock_io();

  while(~0) {
    PT_SCHEDULE(time(&_time));
    PT_SCHEDULE(loopcnt(&_loopcnt));
    /* IN ---------------------------------------------------------- */
    PT_SCHEDULE(din(&_din[0].pt));        /**< HAZARD (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[1].pt));        /**< KEYIN  (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[2].pt));        /**< DOORSW (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[3].pt));        /**< ALARM  (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[4].pt));        /**<   --   (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[5].pt));        /**<   --   (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[6].pt));        /**<   --   (WAKE CAPABLE) */
    PT_SCHEDULE(din(&_din[7].pt));        /**<   --   (WAKE CAPABLE) */
    PT_SCHEDULE(can(&_can));
    /* OUT/BEHAVIORS ----------------------------------------------- */
    PT_SCHEDULE(dout(&_dout));            /**< SSR DRIVE             */
    PT_SCHEDULE(blink(&_blink));
    PT_SCHEDULE(hold(&_hold));
    PT_SCHEDULE(status(&_status));
    PT_SCHEDULE(timereq(&_timereq));
    PT_SCHEDULE(loopsreq(&_loopsreq));
#ifndef DEBUG
    __WFI();
#endif
  }
  NVIC_SystemReset();
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief logic to determine if hold should be active
 */
static int
  hold_requested(void)
{
  int result = 0;

  /* any active DIN0..7 will hold BODY+ on */
  result |= ((flags & FLAG_DIN0_ACTIVE) || (flags & FLAG_DIN1_ACTIVE) ||
             (flags & FLAG_DIN2_ACTIVE) || (flags & FLAG_DIN3_ACTIVE) ||
             (flags & FLAG_DIN4_ACTIVE) || (flags & FLAG_DIN5_ACTIVE) ||
             (flags & FLAG_DIN6_ACTIVE) || (flags & FLAG_DIN7_ACTIVE));

  /* external modules can 'tickle' us to hold BODY+ on for them */
  result |= (flags & FLAG_HOLDREQ), CLEAR_BIT(flags, FLAG_HOLDREQ);

  return result;
}

#ifdef DEBUG
/** ---------------------------------------------------------------------------
 * @internal
 * @brief assert helper
 */
void
  __assert(const char *__expression,
           const char *__filename, int __line)
{
  static char s[256];
  snprintf(s, sizeof(s)-1, "[ %s : L%d ]   %s", __filename, __line, __expression);
  debug_runtime_error(s);
}
#endif