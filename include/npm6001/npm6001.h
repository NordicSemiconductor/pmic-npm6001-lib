/*
 * Copyright (c) 2022, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NPM6001_H
#define NPM6001_H

#ifdef __cplusplus
    extern "C" {
#endif

/* ====================================================== Include types ====================================================== */
#include "npm6001_types.h"

/* ========================================= Start of section using anonymous unions ========================================= */

#include "compiler_abstraction.h"

#if defined (__CC_ARM)
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
  #pragma clang diagnostic ignored "-Wgnu-anonymous-struct"
  #pragma clang diagnostic ignored "-Wnested-anon-types"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Unsupported compiler type
#endif

/* =========================================================================================================================== */
/* ================                                    Peripherals Section                                    ================ */
/* =========================================================================================================================== */


/* =========================================================================================================================== */
/* ================                                          DIGITAL                                          ================ */
/* =========================================================================================================================== */

/* ===================================================== Struct DIGITAL ====================================================== */
/**
  * @brief (unspecified)
  */
typedef struct {                                     /*!< DIGITAL Structure                                                    */
  __IM  uint8_t   RESERVED;
  __IOM uint8_t   SWREADY;                           /*!< (@ 0x00000001) Software ready                                        */
  __OM  uint8_t   TASKS_START_BUCK3;                 /*!< (@ 0x00000002) Start BUCK3                                           */
  __OM  uint8_t   TASKS_START_LDO0;                  /*!< (@ 0x00000003) Start LDO0                                            */
  __OM  uint8_t   TASKS_START_LDO1;                  /*!< (@ 0x00000004) Start LDO1                                            */
  __IM  uint8_t   RESERVED1;
  __OM  uint8_t   TASKS_START_THWARN;                /*!< (@ 0x00000006) Start thermal warning sensor                          */
  __OM  uint8_t   TASKS_START_TH_SHUTDN;             /*!< (@ 0x00000007) Start thermal shutdown sensor                         */
  __OM  uint8_t   TASKS_STOP_BUCK3;                  /*!< (@ 0x00000008) Stop BUCK3                                            */
  __OM  uint8_t   TASKS_STOP_LDO0;                   /*!< (@ 0x00000009) Stop LDO0                                             */
  __OM  uint8_t   TASKS_STOP_LDO1;                   /*!< (@ 0x0000000A) Stop LDO1                                             */
  __IM  uint8_t   RESERVED2;
  __OM  uint8_t   TASKS_STOP_THWARN;                 /*!< (@ 0x0000000C) Stop thermal warning sensor                           */
  __OM  uint8_t   TASKS_STOP_THSHUTDN;               /*!< (@ 0x0000000D) Stop thermal shutdown sensor                          */
  __OM  uint8_t   TASKS_UPDATE_VOUTPWM;              /*!< (@ 0x0000000E) Update output voltage settings for BUCK0, BUCK1 and
                                                                         BUCK2*/                                                  
  __IM  uint8_t   RESERVED3[15];
  __IOM uint8_t   EVENTS_THWARN;                     /*!< (@ 0x0000001E) Thermal warning event                                 */
  __IOM uint8_t   EVENTS_BUCK0OC;                    /*!< (@ 0x0000001F) BUCK0 overcurrent event                               */
  __IOM uint8_t   EVENTS_BUCK1OC;                    /*!< (@ 0x00000020) BUCK1 overcurrent event                               */
  __IOM uint8_t   EVENTS_BUCK2OC;                    /*!< (@ 0x00000021) BUCK2 overcurrent event                               */
  __IOM uint8_t   EVENTS_BUCK3OC;                    /*!< (@ 0x00000022) BUCK3 overcurrent event                               */
  __IM  uint8_t   RESERVED4[7];
  __IOM uint8_t   INTEN0;                            /*!< (@ 0x0000002A) Enable or disable interrupts                          */
  __IOM uint8_t   INTENSET0;                         /*!< (@ 0x0000002B) Interrupt enable SET                                  */
  __IOM uint8_t   INTENCLR0;                         /*!< (@ 0x0000002C) Interrupt enable CLEAR                                */
  __IM  uint8_t   INTPEND0;                          /*!< (@ 0x0000002D) Interrupt pending                                     */
  __IM  uint32_t  RESERVED5[3];
  __IOM uint8_t   BUCK0VOUTULP;                      /*!< (@ 0x0000003A) BUCK0 voltage setting (hysteretic mode)               */
  __IOM uint8_t   BUCK0VOUTPWM;                      /*!< (@ 0x0000003B) BUCK0 voltage setting (PWM mode)                      */
  __IOM uint8_t   BUCK1VOUTULP;                      /*!< (@ 0x0000003C) BUCK1 voltage setting (hysteretic mode)               */
  __IOM uint8_t   BUCK1VOUTPWM;                      /*!< (@ 0x0000003D) BUCK1 voltage setting (PWM mode)                      */
  __IM  uint16_t  RESERVED6;
  __IOM uint8_t   BUCK2VOUTULP;                      /*!< (@ 0x00000040) BUCK2 voltage setting (hysteretic mode)               */
  __IOM uint8_t   BUCK2VOUTPWM;                      /*!< (@ 0x00000041) BUCK2 voltage setting (PWM mode)                      */
  __IM  uint16_t  RESERVED7;
  __IOM uint8_t   BUCK3SELDAC;                       /*!< (@ 0x00000044) Internal DAC enable for BUCK3                         */
  __IOM uint8_t   BUCK3VOUT;                         /*!< (@ 0x00000045) BUCK3 voltage setting                                 */
  __IOM uint8_t   LDO0VOUT;                          /*!< (@ 0x00000046) LDO0 voltage setting                                  */
  __IM  uint8_t   RESERVED8[3];
  __IOM uint8_t   BUCK0CONFPWMMODE;                  /*!< (@ 0x0000004A) BUCK0 PWM mode configuration                          */
  __IOM uint8_t   BUCK1CONFPWMMODE;                  /*!< (@ 0x0000004B) BUCK1 PWM mode configuration                          */
  __IOM uint8_t   BUCK2CONFPWMMODE;                  /*!< (@ 0x0000004C) BUCK2 PWM mode configuration                          */
  __IOM uint8_t   BUCK3CONFPWMMODE;                  /*!< (@ 0x0000004D) BUCK3 PWM mode configuration                          */
  __IOM uint8_t   BUCKMODEPADCONF;                   /*!< (@ 0x0000004E) BUCK_MODE pin configuration                           */
  __IM  uint8_t   RESERVED9;
  __IOM uint8_t   THDYNPOWERUP;                      /*!< (@ 0x00000050) Thermal sensors' dynamic configuration                */
  __IM  uint16_t  RESERVED10;
  __IOM uint8_t   PADDRIVESTRENGTH;                  /*!< (@ 0x00000053) Drive strength control                                */
  __IOM uint8_t   WDARMEDVALUE;                      /*!< (@ 0x00000054) Arm watchdog or wake-up timer. Use strobe
                                                                         WDARMEDSTROBE.*/                                         
  __OM  uint8_t   WDARMEDSTROBE;                     /*!< (@ 0x00000055) Strobe for register WDARMEDVALUE                      */
  __IOM uint8_t   WDTRIGGERVALUE0;                   /*!< (@ 0x00000056) Watchdog and wake-up timer trigger value, lowest byte.
                                                                         Use strobe WDDATASTROBE*/                                
  __IOM uint8_t   WDTRIGGERVALUE1;                   /*!< (@ 0x00000057) Watchdog and wake-up timer trigger value, middle byte.
                                                                         Use strobe WDDATASTROBE*/                                
  __IOM uint8_t   WDTRIGGERVALUE2;                   /*!< (@ 0x00000058) Watchdog and wake-up timer trigger value, highest byte.
                                                                         Use strobe WDDATASTROBE*/                                
  __IM  uint32_t  RESERVED11;
  __OM  uint8_t   WDDATASTROBE;                      /*!< (@ 0x0000005D) Strobe for registers WDTRIGGERVALUE                   */
  __IOM uint8_t   WDPWRUPVALUE;                      /*!< (@ 0x0000005E) Watchdog and wake-up timer enable. Use Strobe
                                                                         WDPWRUPSTROBE*/                                          
  __OM  uint8_t   WDPWRUPSTROBE;                     /*!< (@ 0x0000005F) Strobe for register WDPWRUPVALUE                      */
  __OM  uint8_t   WDKICK;                            /*!< (@ 0x00000060) Watchdog kick                                         */
  __IM  uint8_t   RESERVED12;
  __IOM uint8_t   WDREQPOWERDOWN;                    /*!< (@ 0x00000062) Enter hibernate mode                                  */
  __IM  uint16_t  RESERVED13[3];
  __IOM uint8_t   GPIOOUTSET;                        /*!< (@ 0x00000069) GPIO output value SET                                 */
  __IOM uint8_t   GPIOOUTCLR;                        /*!< (@ 0x0000006A) GPIO output value CLEAR                               */
  __IM  uint8_t   GPIOIN;                            /*!< (@ 0x0000006B) GPIO input value                                      */
  __IOM uint8_t   GPIO0CONF;                         /*!< (@ 0x0000006C) GPIO0 configuration                                   */
  __IOM uint8_t   GPIO1CONF;                         /*!< (@ 0x0000006D) GPIO1 configuration                                   */
  __IOM uint8_t   GPIO2CONF;                         /*!< (@ 0x0000006E) GPIO2 configuration                                   */
  __IM  uint16_t  RESERVED14;
  __IOM uint8_t   LDO0CTRL;                          /*!< (@ 0x00000071) LDO0 current limiter, output high-impedance and
                                                                         pulldown control*/                                       
  __IM  uint8_t   RESERVED15;
  __IOM uint8_t   LDO1CTRL;                          /*!< (@ 0x00000073) LDO1 current limiter, output high-impedance and
                                                                         pulldown control*/                                       
  __IM  uint8_t   RESERVED16[55];
  __IOM uint8_t   OVERRIDEPWRUPBUCK;                 /*!< (@ 0x000000AB) Override for disabling BUCK1 or/and BUCK2 regulators  */
} __PACKED NRF_DIGITAL_Type;                         /*!< Size = 172 (0x0AC)                                                   */

/* DIGITAL_SWREADY: Software ready */
  #define DIGITAL_SWREADY_ResetValue (0x00UL)        /*!< Reset value of SWREADY register.                                     */

/* SWREADY @Bit 0 : Before this bit is set, nPM6001 does not react to BUCK_MODEx pins. */
  #define DIGITAL_SWREADY_SWREADY_Pos (0UL)          /*!< Position of SWREADY field.                                           */
  #define DIGITAL_SWREADY_SWREADY_Msk (0x1UL << DIGITAL_SWREADY_SWREADY_Pos) /*!< Bit mask of SWREADY field.                   */
  #define DIGITAL_SWREADY_SWREADY_Min (0x0UL)        /*!< Min enumerator value of SWREADY field.                               */
  #define DIGITAL_SWREADY_SWREADY_Max (0x1UL)        /*!< Max enumerator value of SWREADY field.                               */
  #define DIGITAL_SWREADY_SWREADY_NOTREADY (0x0UL)   /*!< Software configuration is not yet done. BUCKs are running in PWM
                                                          mode.*/                                                                 
  #define DIGITAL_SWREADY_SWREADY_READY (0x1UL)      /*!< Software configuration is ready. Pins BUCK_MODEx may be used to
                                                          control BUCKs' PWM modes.*/                                             


/* DIGITAL_TASKS_START_BUCK3: Start BUCK3 */
  #define DIGITAL_TASKS_START_BUCK3_ResetValue (0x00UL) /*!< Reset value of TASKS_START_BUCK3 register.                        */

/* TASKS_START_BUCK3 @Bit 0 : Start BUCK3 */
  #define DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Pos (0UL) /*!< Position of TASKS_START_BUCK3 field.                      */
  #define DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Msk (0x1UL << DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Pos) /*!< Bit
                                                                            mask of TASKS_START_BUCK3 field.*/                    
  #define DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Min (0x1UL) /*!< Min enumerator value of TASKS_START_BUCK3 field.        */
  #define DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Max (0x1UL) /*!< Max enumerator value of TASKS_START_BUCK3 field.        */
  #define DIGITAL_TASKS_START_BUCK3_TASKS_START_BUCK3_Trigger (0x1UL) /*!< Trigger task                                        */


/* DIGITAL_TASKS_START_LDO0: Start LDO0 */
  #define DIGITAL_TASKS_START_LDO0_ResetValue (0x00UL) /*!< Reset value of TASKS_START_LDO0 register.                          */

/* TASKS_START_LDO0 @Bit 0 : Start LDO0 */
  #define DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Pos (0UL) /*!< Position of TASKS_START_LDO0 field.                         */
  #define DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Msk (0x1UL << DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Pos) /*!< Bit mask
                                                                            of TASKS_START_LDO0 field.*/                          
  #define DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Min (0x1UL) /*!< Min enumerator value of TASKS_START_LDO0 field.           */
  #define DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Max (0x1UL) /*!< Max enumerator value of TASKS_START_LDO0 field.           */
  #define DIGITAL_TASKS_START_LDO0_TASKS_START_LDO0_Trigger (0x1UL) /*!< Trigger task                                          */


/* DIGITAL_TASKS_START_LDO1: Start LDO1 */
  #define DIGITAL_TASKS_START_LDO1_ResetValue (0x00UL) /*!< Reset value of TASKS_START_LDO1 register.                          */

/* TASKS_START_LDO1 @Bit 0 : Start LDO1 */
  #define DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Pos (0UL) /*!< Position of TASKS_START_LDO1 field.                         */
  #define DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Msk (0x1UL << DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Pos) /*!< Bit mask
                                                                            of TASKS_START_LDO1 field.*/                          
  #define DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Min (0x1UL) /*!< Min enumerator value of TASKS_START_LDO1 field.           */
  #define DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Max (0x1UL) /*!< Max enumerator value of TASKS_START_LDO1 field.           */
  #define DIGITAL_TASKS_START_LDO1_TASKS_START_LDO1_Trigger (0x1UL) /*!< Trigger task                                          */


/* DIGITAL_TASKS_START_THWARN: Start thermal warning sensor */
  #define DIGITAL_TASKS_START_THWARN_ResetValue (0x00UL) /*!< Reset value of TASKS_START_THWARN register.                      */

/* TASKS_START_THWARN @Bit 0 : Start thermal warning sensor */
  #define DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Pos (0UL) /*!< Position of TASKS_START_THWARN field.                   */
  #define DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Msk (0x1UL << DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Pos) /*!<
                                                                            Bit mask of TASKS_START_THWARN field.*/               
  #define DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Min (0x1UL) /*!< Min enumerator value of TASKS_START_THWARN field.     */
  #define DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Max (0x1UL) /*!< Max enumerator value of TASKS_START_THWARN field.     */
  #define DIGITAL_TASKS_START_THWARN_TASKS_START_THWARN_Trigger (0x1UL) /*!< Trigger task                                      */


/* DIGITAL_TASKS_START_TH_SHUTDN: Start thermal shutdown sensor */
  #define DIGITAL_TASKS_START_TH_SHUTDN_ResetValue (0x00UL) /*!< Reset value of TASKS_START_TH_SHUTDN register.                */

/* TASKS_START_TH_SHUTDN @Bit 0 : Start thermal shutdown sensor */
  #define DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Pos (0UL) /*!< Position of TASKS_START_TH_SHUTDN field.          */
  #define DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Msk (0x1UL << DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Pos)
                                                                            /*!< Bit mask of TASKS_START_TH_SHUTDN field.*/       
  #define DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Min (0x1UL) /*!< Min enumerator value of TASKS_START_TH_SHUTDN
                                                                            field.*/                                              
  #define DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Max (0x1UL) /*!< Max enumerator value of TASKS_START_TH_SHUTDN
                                                                            field.*/                                              
  #define DIGITAL_TASKS_START_TH_SHUTDN_TASKS_START_TH_SHUTDN_Trigger (0x1UL) /*!< Trigger task                                */


/* DIGITAL_TASKS_STOP_BUCK3: Stop BUCK3 */
  #define DIGITAL_TASKS_STOP_BUCK3_ResetValue (0x00UL) /*!< Reset value of TASKS_STOP_BUCK3 register.                          */

/* TASKS_STOP_BUCK3 @Bit 0 : Stop BUCK3 */
  #define DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Pos (0UL) /*!< Position of TASKS_STOP_BUCK3 field.                         */
  #define DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Msk (0x1UL << DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Pos) /*!< Bit mask
                                                                            of TASKS_STOP_BUCK3 field.*/                          
  #define DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Min (0x1UL) /*!< Min enumerator value of TASKS_STOP_BUCK3 field.           */
  #define DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Max (0x1UL) /*!< Max enumerator value of TASKS_STOP_BUCK3 field.           */
  #define DIGITAL_TASKS_STOP_BUCK3_TASKS_STOP_BUCK3_Trigger (0x1UL) /*!< Trigger task                                          */


/* DIGITAL_TASKS_STOP_LDO0: Stop LDO0 */
  #define DIGITAL_TASKS_STOP_LDO0_ResetValue (0x00UL) /*!< Reset value of TASKS_STOP_LDO0 register.                            */

/* TASKS_STOP_LDO0 @Bit 0 : Stop LDO0 */
  #define DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Pos (0UL) /*!< Position of TASKS_STOP_LDO0 field.                            */
  #define DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Msk (0x1UL << DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Pos) /*!< Bit mask of
                                                                            TASKS_STOP_LDO0 field.*/                              
  #define DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Min (0x1UL) /*!< Min enumerator value of TASKS_STOP_LDO0 field.              */
  #define DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Max (0x1UL) /*!< Max enumerator value of TASKS_STOP_LDO0 field.              */
  #define DIGITAL_TASKS_STOP_LDO0_TASKS_STOP_LDO0_Trigger (0x1UL) /*!< Trigger task                                            */


/* DIGITAL_TASKS_STOP_LDO1: Stop LDO1 */
  #define DIGITAL_TASKS_STOP_LDO1_ResetValue (0x00UL) /*!< Reset value of TASKS_STOP_LDO1 register.                            */

/* TASKS_STOP_LDO1 @Bit 0 : Stop LDO1 */
  #define DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Pos (0UL) /*!< Position of TASKS_STOP_LDO1 field.                            */
  #define DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Msk (0x1UL << DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Pos) /*!< Bit mask of
                                                                            TASKS_STOP_LDO1 field.*/                              
  #define DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Min (0x1UL) /*!< Min enumerator value of TASKS_STOP_LDO1 field.              */
  #define DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Max (0x1UL) /*!< Max enumerator value of TASKS_STOP_LDO1 field.              */
  #define DIGITAL_TASKS_STOP_LDO1_TASKS_STOP_LDO1_Trigger (0x1UL) /*!< Trigger task                                            */


/* DIGITAL_TASKS_STOP_THWARN: Stop thermal warning sensor */
  #define DIGITAL_TASKS_STOP_THWARN_ResetValue (0x00UL) /*!< Reset value of TASKS_STOP_THWARN register.                        */

/* TASKS_STOP_THWARN @Bit 0 : Stop thermal warning sensor */
  #define DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Pos (0UL) /*!< Position of TASKS_STOP_THWARN field.                      */
  #define DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Msk (0x1UL << DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Pos) /*!< Bit
                                                                            mask of TASKS_STOP_THWARN field.*/                    
  #define DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Min (0x1UL) /*!< Min enumerator value of TASKS_STOP_THWARN field.        */
  #define DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Max (0x1UL) /*!< Max enumerator value of TASKS_STOP_THWARN field.        */
  #define DIGITAL_TASKS_STOP_THWARN_TASKS_STOP_THWARN_Trigger (0x1UL) /*!< Trigger task                                        */


/* DIGITAL_TASKS_STOP_THSHUTDN: Stop thermal shutdown sensor */
  #define DIGITAL_TASKS_STOP_THSHUTDN_ResetValue (0x00UL) /*!< Reset value of TASKS_STOP_THSHUTDN register.                    */

/* TASKS_STOP_THSHUTDN @Bit 0 : Stop thermal shutdown sensor */
  #define DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Pos (0UL) /*!< Position of TASKS_STOP_THSHUTDN field.                */
  #define DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Msk (0x1UL << DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Pos)
                                                                            /*!< Bit mask of TASKS_STOP_THSHUTDN field.*/         
  #define DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Min (0x1UL) /*!< Min enumerator value of TASKS_STOP_THSHUTDN field.  */
  #define DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Max (0x1UL) /*!< Max enumerator value of TASKS_STOP_THSHUTDN field.  */
  #define DIGITAL_TASKS_STOP_THSHUTDN_TASKS_STOP_THSHUTDN_Trigger (0x1UL) /*!< Trigger task                                    */


/* DIGITAL_TASKS_UPDATE_VOUTPWM: Update output voltage settings for BUCK0, BUCK1 and BUCK2 */
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_ResetValue (0x00UL) /*!< Reset value of TASKS_UPDATE_VOUTPWM register.                  */

/* TASKS_UPDATE_VOUTPWM @Bit 0 : Update output voltage settings for BUCK0, BUCK1 and BUCK2 */
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Pos (0UL) /*!< Position of TASKS_UPDATE_VOUTPWM field.             */
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Msk (0x1UL << DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Pos)
                                                                            /*!< Bit mask of TASKS_UPDATE_VOUTPWM field.*/        
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Min (0x1UL) /*!< Min enumerator value of TASKS_UPDATE_VOUTPWM
                                                                            field.*/                                              
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Max (0x1UL) /*!< Max enumerator value of TASKS_UPDATE_VOUTPWM
                                                                            field.*/                                              
  #define DIGITAL_TASKS_UPDATE_VOUTPWM_TASKS_UPDATE_VOUTPWM_Trigger (0x1UL) /*!< Trigger task                                  */


/* DIGITAL_EVENTS_THWARN: Thermal warning event */
  #define DIGITAL_EVENTS_THWARN_ResetValue (0x00UL)  /*!< Reset value of EVENTS_THWARN register.                               */

/* EVENTS_THWARN @Bit 0 : Thermal warning event */
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Pos (0UL) /*!< Position of EVENTS_THWARN field.                                  */
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Msk (0x1UL << DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Pos) /*!< Bit mask of
                                                                            EVENTS_THWARN field.*/                                
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Min (0x0UL) /*!< Min enumerator value of EVENTS_THWARN field.                    */
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Max (0x1UL) /*!< Max enumerator value of EVENTS_THWARN field.                    */
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_NotGenerated (0x0UL) /*!< Event not generated                                    */
  #define DIGITAL_EVENTS_THWARN_EVENTS_THWARN_Generated (0x1UL) /*!< Event generated                                           */


/* DIGITAL_EVENTS_BUCK0OC: BUCK0 overcurrent event */
  #define DIGITAL_EVENTS_BUCK0OC_ResetValue (0x00UL) /*!< Reset value of EVENTS_BUCK0OC register.                              */

/* EVENTS_BUCK0OC @Bit 0 : BUCK0 overcurrent event */
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Pos (0UL) /*!< Position of EVENTS_BUCK0OC field.                               */
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Msk (0x1UL << DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Pos) /*!< Bit mask of
                                                                            EVENTS_BUCK0OC field.*/                               
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Min (0x0UL) /*!< Min enumerator value of EVENTS_BUCK0OC field.                 */
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Max (0x1UL) /*!< Max enumerator value of EVENTS_BUCK0OC field.                 */
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_NotGenerated (0x0UL) /*!< Event not generated                                  */
  #define DIGITAL_EVENTS_BUCK0OC_EVENTS_BUCK0OC_Generated (0x1UL) /*!< Event generated                                         */


/* DIGITAL_EVENTS_BUCK1OC: BUCK1 overcurrent event */
  #define DIGITAL_EVENTS_BUCK1OC_ResetValue (0x00UL) /*!< Reset value of EVENTS_BUCK1OC register.                              */

/* EVENTS_BUCK1OC @Bit 0 : BUCK1 overcurrent event */
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Pos (0UL) /*!< Position of EVENTS_BUCK1OC field.                               */
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Msk (0x1UL << DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Pos) /*!< Bit mask of
                                                                            EVENTS_BUCK1OC field.*/                               
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Min (0x0UL) /*!< Min enumerator value of EVENTS_BUCK1OC field.                 */
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Max (0x1UL) /*!< Max enumerator value of EVENTS_BUCK1OC field.                 */
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_NotGenerated (0x0UL) /*!< Event not generated                                  */
  #define DIGITAL_EVENTS_BUCK1OC_EVENTS_BUCK1OC_Generated (0x1UL) /*!< Event generated                                         */


/* DIGITAL_EVENTS_BUCK2OC: BUCK2 overcurrent event */
  #define DIGITAL_EVENTS_BUCK2OC_ResetValue (0x00UL) /*!< Reset value of EVENTS_BUCK2OC register.                              */

/* EVENTS_BUCK2OC @Bit 0 : BUCK2 overcurrent event */
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Pos (0UL) /*!< Position of EVENTS_BUCK2OC field.                               */
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Msk (0x1UL << DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Pos) /*!< Bit mask of
                                                                            EVENTS_BUCK2OC field.*/                               
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Min (0x0UL) /*!< Min enumerator value of EVENTS_BUCK2OC field.                 */
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Max (0x1UL) /*!< Max enumerator value of EVENTS_BUCK2OC field.                 */
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_NotGenerated (0x0UL) /*!< Event not generated                                  */
  #define DIGITAL_EVENTS_BUCK2OC_EVENTS_BUCK2OC_Generated (0x1UL) /*!< Event generated                                         */


/* DIGITAL_EVENTS_BUCK3OC: BUCK3 overcurrent event */
  #define DIGITAL_EVENTS_BUCK3OC_ResetValue (0x00UL) /*!< Reset value of EVENTS_BUCK3OC register.                              */

/* EVENTS_BUCK3OC @Bit 0 : BUCK3 overcurrent event */
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Pos (0UL) /*!< Position of EVENTS_BUCK3OC field.                               */
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Msk (0x1UL << DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Pos) /*!< Bit mask of
                                                                            EVENTS_BUCK3OC field.*/                               
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Min (0x0UL) /*!< Min enumerator value of EVENTS_BUCK3OC field.                 */
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Max (0x1UL) /*!< Max enumerator value of EVENTS_BUCK3OC field.                 */
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_NotGenerated (0x0UL) /*!< Event not generated                                  */
  #define DIGITAL_EVENTS_BUCK3OC_EVENTS_BUCK3OC_Generated (0x1UL) /*!< Event generated                                         */


/* DIGITAL_INTEN0: Enable or disable interrupts */
  #define DIGITAL_INTEN0_ResetValue (0x00UL)         /*!< Reset value of INTEN0 register.                                      */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_INTEN0_RESERVED0_Pos (0UL)         /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_INTEN0_RESERVED0_Msk (0x1UL << DIGITAL_INTEN0_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.               */

/* RESERVED1 @Bit 1 : Reserved1 */
  #define DIGITAL_INTEN0_RESERVED1_Pos (1UL)         /*!< Position of RESERVED1 field.                                         */
  #define DIGITAL_INTEN0_RESERVED1_Msk (0x1UL << DIGITAL_INTEN0_RESERVED1_Pos) /*!< Bit mask of RESERVED1 field.               */

/* RESERVED2 @Bit 2 : Reserved2 */
  #define DIGITAL_INTEN0_RESERVED2_Pos (2UL)         /*!< Position of RESERVED2 field.                                         */
  #define DIGITAL_INTEN0_RESERVED2_Msk (0x1UL << DIGITAL_INTEN0_RESERVED2_Pos) /*!< Bit mask of RESERVED2 field.               */

/* THWARN @Bit 3 : Enable or disable interrupt for event thermal warning */
  #define DIGITAL_INTEN0_THWARN_Pos (3UL)            /*!< Position of THWARN field.                                            */
  #define DIGITAL_INTEN0_THWARN_Msk (0x1UL << DIGITAL_INTEN0_THWARN_Pos) /*!< Bit mask of THWARN field.                        */
  #define DIGITAL_INTEN0_THWARN_Min (0x0UL)          /*!< Min enumerator value of THWARN field.                                */
  #define DIGITAL_INTEN0_THWARN_Max (0x1UL)          /*!< Max enumerator value of THWARN field.                                */
  #define DIGITAL_INTEN0_THWARN_DISABLED (0x0UL)     /*!< Disable                                                              */
  #define DIGITAL_INTEN0_THWARN_ENABLED (0x1UL)      /*!< Enable                                                               */

/* BUCK0OC @Bit 4 : Enable or disable interrupt for event BUCK0 overcurrent */
  #define DIGITAL_INTEN0_BUCK0OC_Pos (4UL)           /*!< Position of BUCK0OC field.                                           */
  #define DIGITAL_INTEN0_BUCK0OC_Msk (0x1UL << DIGITAL_INTEN0_BUCK0OC_Pos) /*!< Bit mask of BUCK0OC field.                     */
  #define DIGITAL_INTEN0_BUCK0OC_Min (0x0UL)         /*!< Min enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTEN0_BUCK0OC_Max (0x1UL)         /*!< Max enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTEN0_BUCK0OC_DISABLED (0x0UL)    /*!< Disable                                                              */
  #define DIGITAL_INTEN0_BUCK0OC_ENABLED (0x1UL)     /*!< Enable                                                               */

/* BUCK1OC @Bit 5 : Enable or disable interrupt for event BUCK1 overcurrent */
  #define DIGITAL_INTEN0_BUCK1OC_Pos (5UL)           /*!< Position of BUCK1OC field.                                           */
  #define DIGITAL_INTEN0_BUCK1OC_Msk (0x1UL << DIGITAL_INTEN0_BUCK1OC_Pos) /*!< Bit mask of BUCK1OC field.                     */
  #define DIGITAL_INTEN0_BUCK1OC_Min (0x0UL)         /*!< Min enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTEN0_BUCK1OC_Max (0x1UL)         /*!< Max enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTEN0_BUCK1OC_DISABLED (0x0UL)    /*!< Disable                                                              */
  #define DIGITAL_INTEN0_BUCK1OC_ENABLED (0x1UL)     /*!< Enable                                                               */

/* BUCK2OC @Bit 6 : Enable or disable interrupt for event BUCK2 overcurrent */
  #define DIGITAL_INTEN0_BUCK2OC_Pos (6UL)           /*!< Position of BUCK2OC field.                                           */
  #define DIGITAL_INTEN0_BUCK2OC_Msk (0x1UL << DIGITAL_INTEN0_BUCK2OC_Pos) /*!< Bit mask of BUCK2OC field.                     */
  #define DIGITAL_INTEN0_BUCK2OC_Min (0x0UL)         /*!< Min enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTEN0_BUCK2OC_Max (0x1UL)         /*!< Max enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTEN0_BUCK2OC_DISABLED (0x0UL)    /*!< Disable                                                              */
  #define DIGITAL_INTEN0_BUCK2OC_ENABLED (0x1UL)     /*!< Enable                                                               */

/* BUCK3OC @Bit 7 : Enable or disable interrupt for event BUCK3 overcurrent */
  #define DIGITAL_INTEN0_BUCK3OC_Pos (7UL)           /*!< Position of BUCK3OC field.                                           */
  #define DIGITAL_INTEN0_BUCK3OC_Msk (0x1UL << DIGITAL_INTEN0_BUCK3OC_Pos) /*!< Bit mask of BUCK3OC field.                     */
  #define DIGITAL_INTEN0_BUCK3OC_Min (0x0UL)         /*!< Min enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTEN0_BUCK3OC_Max (0x1UL)         /*!< Max enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTEN0_BUCK3OC_DISABLED (0x0UL)    /*!< Disable                                                              */
  #define DIGITAL_INTEN0_BUCK3OC_ENABLED (0x1UL)     /*!< Enable                                                               */


/* DIGITAL_INTENSET0: Interrupt enable SET */
  #define DIGITAL_INTENSET0_ResetValue (0x00UL)      /*!< Reset value of INTENSET0 register.                                   */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_INTENSET0_RESERVED0_Pos (0UL)      /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_INTENSET0_RESERVED0_Msk (0x1UL << DIGITAL_INTENSET0_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.         */

/* RESERVED1 @Bit 1 : Reserved1 */
  #define DIGITAL_INTENSET0_RESERVED1_Pos (1UL)      /*!< Position of RESERVED1 field.                                         */
  #define DIGITAL_INTENSET0_RESERVED1_Msk (0x1UL << DIGITAL_INTENSET0_RESERVED1_Pos) /*!< Bit mask of RESERVED1 field.         */

/* RESERVED2 @Bit 2 : Reserved2 */
  #define DIGITAL_INTENSET0_RESERVED2_Pos (2UL)      /*!< Position of RESERVED2 field.                                         */
  #define DIGITAL_INTENSET0_RESERVED2_Msk (0x1UL << DIGITAL_INTENSET0_RESERVED2_Pos) /*!< Bit mask of RESERVED2 field.         */

/* THWARN @Bit 3 : Thermal warning */
  #define DIGITAL_INTENSET0_THWARN_Pos (3UL)         /*!< Position of THWARN field.                                            */
  #define DIGITAL_INTENSET0_THWARN_Msk (0x1UL << DIGITAL_INTENSET0_THWARN_Pos) /*!< Bit mask of THWARN field.                  */
  #define DIGITAL_INTENSET0_THWARN_Min (0x0UL)       /*!< Min enumerator value of THWARN field.                                */
  #define DIGITAL_INTENSET0_THWARN_Max (0x1UL)       /*!< Max enumerator value of THWARN field.                                */
  #define DIGITAL_INTENSET0_THWARN_NOACTION (0x0UL)  /*!< No Action                                                            */
  #define DIGITAL_INTENSET0_THWARN_SET (0x1UL)       /*!< Set                                                                  */

/* BUCK0OC @Bit 4 : BUCK0 overcurrent */
  #define DIGITAL_INTENSET0_BUCK0OC_Pos (4UL)        /*!< Position of BUCK0OC field.                                           */
  #define DIGITAL_INTENSET0_BUCK0OC_Msk (0x1UL << DIGITAL_INTENSET0_BUCK0OC_Pos) /*!< Bit mask of BUCK0OC field.               */
  #define DIGITAL_INTENSET0_BUCK0OC_Min (0x0UL)      /*!< Min enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTENSET0_BUCK0OC_Max (0x1UL)      /*!< Max enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTENSET0_BUCK0OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENSET0_BUCK0OC_SET (0x1UL)      /*!< Set                                                                  */

/* BUCK1OC @Bit 5 : BUCK1 overcurrent */
  #define DIGITAL_INTENSET0_BUCK1OC_Pos (5UL)        /*!< Position of BUCK1OC field.                                           */
  #define DIGITAL_INTENSET0_BUCK1OC_Msk (0x1UL << DIGITAL_INTENSET0_BUCK1OC_Pos) /*!< Bit mask of BUCK1OC field.               */
  #define DIGITAL_INTENSET0_BUCK1OC_Min (0x0UL)      /*!< Min enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTENSET0_BUCK1OC_Max (0x1UL)      /*!< Max enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTENSET0_BUCK1OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENSET0_BUCK1OC_SET (0x1UL)      /*!< Set                                                                  */

/* BUCK2OC @Bit 6 : BUCK2 overcurrent */
  #define DIGITAL_INTENSET0_BUCK2OC_Pos (6UL)        /*!< Position of BUCK2OC field.                                           */
  #define DIGITAL_INTENSET0_BUCK2OC_Msk (0x1UL << DIGITAL_INTENSET0_BUCK2OC_Pos) /*!< Bit mask of BUCK2OC field.               */
  #define DIGITAL_INTENSET0_BUCK2OC_Min (0x0UL)      /*!< Min enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTENSET0_BUCK2OC_Max (0x1UL)      /*!< Max enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTENSET0_BUCK2OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENSET0_BUCK2OC_SET (0x1UL)      /*!< Set                                                                  */

/* BUCK3OC @Bit 7 : BUCK3 overcurrent */
  #define DIGITAL_INTENSET0_BUCK3OC_Pos (7UL)        /*!< Position of BUCK3OC field.                                           */
  #define DIGITAL_INTENSET0_BUCK3OC_Msk (0x1UL << DIGITAL_INTENSET0_BUCK3OC_Pos) /*!< Bit mask of BUCK3OC field.               */
  #define DIGITAL_INTENSET0_BUCK3OC_Min (0x0UL)      /*!< Min enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTENSET0_BUCK3OC_Max (0x1UL)      /*!< Max enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTENSET0_BUCK3OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENSET0_BUCK3OC_SET (0x1UL)      /*!< Set                                                                  */


/* DIGITAL_INTENCLR0: Interrupt enable CLEAR */
  #define DIGITAL_INTENCLR0_ResetValue (0x00UL)      /*!< Reset value of INTENCLR0 register.                                   */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_INTENCLR0_RESERVED0_Pos (0UL)      /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_INTENCLR0_RESERVED0_Msk (0x1UL << DIGITAL_INTENCLR0_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.         */

/* RESERVED1 @Bit 1 : Reserved1 */
  #define DIGITAL_INTENCLR0_RESERVED1_Pos (1UL)      /*!< Position of RESERVED1 field.                                         */
  #define DIGITAL_INTENCLR0_RESERVED1_Msk (0x1UL << DIGITAL_INTENCLR0_RESERVED1_Pos) /*!< Bit mask of RESERVED1 field.         */

/* RESERVED2 @Bit 2 : Reserved2 */
  #define DIGITAL_INTENCLR0_RESERVED2_Pos (2UL)      /*!< Position of RESERVED2 field.                                         */
  #define DIGITAL_INTENCLR0_RESERVED2_Msk (0x1UL << DIGITAL_INTENCLR0_RESERVED2_Pos) /*!< Bit mask of RESERVED2 field.         */

/* THWARN @Bit 3 : Thermal warning */
  #define DIGITAL_INTENCLR0_THWARN_Pos (3UL)         /*!< Position of THWARN field.                                            */
  #define DIGITAL_INTENCLR0_THWARN_Msk (0x1UL << DIGITAL_INTENCLR0_THWARN_Pos) /*!< Bit mask of THWARN field.                  */
  #define DIGITAL_INTENCLR0_THWARN_Min (0x0UL)       /*!< Min enumerator value of THWARN field.                                */
  #define DIGITAL_INTENCLR0_THWARN_Max (0x1UL)       /*!< Max enumerator value of THWARN field.                                */
  #define DIGITAL_INTENCLR0_THWARN_NOACTION (0x0UL)  /*!< No Action                                                            */
  #define DIGITAL_INTENCLR0_THWARN_CLEAR (0x1UL)     /*!< Clear                                                                */

/* BUCK0OC @Bit 4 : BUCK0 overcurrent */
  #define DIGITAL_INTENCLR0_BUCK0OC_Pos (4UL)        /*!< Position of BUCK0OC field.                                           */
  #define DIGITAL_INTENCLR0_BUCK0OC_Msk (0x1UL << DIGITAL_INTENCLR0_BUCK0OC_Pos) /*!< Bit mask of BUCK0OC field.               */
  #define DIGITAL_INTENCLR0_BUCK0OC_Min (0x0UL)      /*!< Min enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK0OC_Max (0x1UL)      /*!< Max enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK0OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENCLR0_BUCK0OC_CLEAR (0x1UL)    /*!< Clear                                                                */

/* BUCK1OC @Bit 5 : BUCK1 overcurrent */
  #define DIGITAL_INTENCLR0_BUCK1OC_Pos (5UL)        /*!< Position of BUCK1OC field.                                           */
  #define DIGITAL_INTENCLR0_BUCK1OC_Msk (0x1UL << DIGITAL_INTENCLR0_BUCK1OC_Pos) /*!< Bit mask of BUCK1OC field.               */
  #define DIGITAL_INTENCLR0_BUCK1OC_Min (0x0UL)      /*!< Min enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK1OC_Max (0x1UL)      /*!< Max enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK1OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENCLR0_BUCK1OC_CLEAR (0x1UL)    /*!< Clear                                                                */

/* BUCK2OC @Bit 6 : BUCK2 overcurrent */
  #define DIGITAL_INTENCLR0_BUCK2OC_Pos (6UL)        /*!< Position of BUCK2OC field.                                           */
  #define DIGITAL_INTENCLR0_BUCK2OC_Msk (0x1UL << DIGITAL_INTENCLR0_BUCK2OC_Pos) /*!< Bit mask of BUCK2OC field.               */
  #define DIGITAL_INTENCLR0_BUCK2OC_Min (0x0UL)      /*!< Min enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK2OC_Max (0x1UL)      /*!< Max enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK2OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENCLR0_BUCK2OC_CLEAR (0x1UL)    /*!< Clear                                                                */

/* BUCK3OC @Bit 7 : BUCK3 overcurrent */
  #define DIGITAL_INTENCLR0_BUCK3OC_Pos (7UL)        /*!< Position of BUCK3OC field.                                           */
  #define DIGITAL_INTENCLR0_BUCK3OC_Msk (0x1UL << DIGITAL_INTENCLR0_BUCK3OC_Pos) /*!< Bit mask of BUCK3OC field.               */
  #define DIGITAL_INTENCLR0_BUCK3OC_Min (0x0UL)      /*!< Min enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK3OC_Max (0x1UL)      /*!< Max enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTENCLR0_BUCK3OC_NOACTION (0x0UL) /*!< No Action                                                            */
  #define DIGITAL_INTENCLR0_BUCK3OC_CLEAR (0x1UL)    /*!< Clear                                                                */


/* DIGITAL_INTPEND0: Interrupt pending */
  #define DIGITAL_INTPEND0_ResetValue (0x00UL)       /*!< Reset value of INTPEND0 register.                                    */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_INTPEND0_RESERVED0_Pos (0UL)       /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_INTPEND0_RESERVED0_Msk (0x1UL << DIGITAL_INTPEND0_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.           */

/* RESERVED1 @Bit 1 : Reserved1 */
  #define DIGITAL_INTPEND0_RESERVED1_Pos (1UL)       /*!< Position of RESERVED1 field.                                         */
  #define DIGITAL_INTPEND0_RESERVED1_Msk (0x1UL << DIGITAL_INTPEND0_RESERVED1_Pos) /*!< Bit mask of RESERVED1 field.           */

/* RESERVED2 @Bit 2 : Reserved2 */
  #define DIGITAL_INTPEND0_RESERVED2_Pos (2UL)       /*!< Position of RESERVED2 field.                                         */
  #define DIGITAL_INTPEND0_RESERVED2_Msk (0x1UL << DIGITAL_INTPEND0_RESERVED2_Pos) /*!< Bit mask of RESERVED2 field.           */

/* THWARN @Bit 3 : Thermal warning */
  #define DIGITAL_INTPEND0_THWARN_Pos (3UL)          /*!< Position of THWARN field.                                            */
  #define DIGITAL_INTPEND0_THWARN_Msk (0x1UL << DIGITAL_INTPEND0_THWARN_Pos) /*!< Bit mask of THWARN field.                    */
  #define DIGITAL_INTPEND0_THWARN_Min (0x0UL)        /*!< Min enumerator value of THWARN field.                                */
  #define DIGITAL_INTPEND0_THWARN_Max (0x1UL)        /*!< Max enumerator value of THWARN field.                                */
  #define DIGITAL_INTPEND0_THWARN_NOTPENDING (0x0UL) /*!< Not Pending                                                          */
  #define DIGITAL_INTPEND0_THWARN_PENDING (0x1UL)    /*!< Pending                                                              */

/* BUCK0OC @Bit 4 : BUCK0 overcurrent */
  #define DIGITAL_INTPEND0_BUCK0OC_Pos (4UL)         /*!< Position of BUCK0OC field.                                           */
  #define DIGITAL_INTPEND0_BUCK0OC_Msk (0x1UL << DIGITAL_INTPEND0_BUCK0OC_Pos) /*!< Bit mask of BUCK0OC field.                 */
  #define DIGITAL_INTPEND0_BUCK0OC_Min (0x0UL)       /*!< Min enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTPEND0_BUCK0OC_Max (0x1UL)       /*!< Max enumerator value of BUCK0OC field.                               */
  #define DIGITAL_INTPEND0_BUCK0OC_NOTPENDING (0x0UL) /*!< Not Pending                                                         */
  #define DIGITAL_INTPEND0_BUCK0OC_PENDING (0x1UL)   /*!< Pending                                                              */

/* BUCK1OC @Bit 5 : BUCK1 overcurrent */
  #define DIGITAL_INTPEND0_BUCK1OC_Pos (5UL)         /*!< Position of BUCK1OC field.                                           */
  #define DIGITAL_INTPEND0_BUCK1OC_Msk (0x1UL << DIGITAL_INTPEND0_BUCK1OC_Pos) /*!< Bit mask of BUCK1OC field.                 */
  #define DIGITAL_INTPEND0_BUCK1OC_Min (0x0UL)       /*!< Min enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTPEND0_BUCK1OC_Max (0x1UL)       /*!< Max enumerator value of BUCK1OC field.                               */
  #define DIGITAL_INTPEND0_BUCK1OC_NOTPENDING (0x0UL) /*!< Not Pending                                                         */
  #define DIGITAL_INTPEND0_BUCK1OC_PENDING (0x1UL)   /*!< Pending                                                              */

/* BUCK2OC @Bit 6 : BUCK2 overcurrent */
  #define DIGITAL_INTPEND0_BUCK2OC_Pos (6UL)         /*!< Position of BUCK2OC field.                                           */
  #define DIGITAL_INTPEND0_BUCK2OC_Msk (0x1UL << DIGITAL_INTPEND0_BUCK2OC_Pos) /*!< Bit mask of BUCK2OC field.                 */
  #define DIGITAL_INTPEND0_BUCK2OC_Min (0x0UL)       /*!< Min enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTPEND0_BUCK2OC_Max (0x1UL)       /*!< Max enumerator value of BUCK2OC field.                               */
  #define DIGITAL_INTPEND0_BUCK2OC_NOTPENDING (0x0UL) /*!< Not Pending                                                         */
  #define DIGITAL_INTPEND0_BUCK2OC_PENDING (0x1UL)   /*!< Pending                                                              */

/* BUCK3OC @Bit 7 : BUCK3 overcurrent */
  #define DIGITAL_INTPEND0_BUCK3OC_Pos (7UL)         /*!< Position of BUCK3OC field.                                           */
  #define DIGITAL_INTPEND0_BUCK3OC_Msk (0x1UL << DIGITAL_INTPEND0_BUCK3OC_Pos) /*!< Bit mask of BUCK3OC field.                 */
  #define DIGITAL_INTPEND0_BUCK3OC_Min (0x0UL)       /*!< Min enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTPEND0_BUCK3OC_Max (0x1UL)       /*!< Max enumerator value of BUCK3OC field.                               */
  #define DIGITAL_INTPEND0_BUCK3OC_NOTPENDING (0x0UL) /*!< Not Pending                                                         */
  #define DIGITAL_INTPEND0_BUCK3OC_PENDING (0x1UL)   /*!< Pending                                                              */


/* DIGITAL_BUCK0VOUTULP: BUCK0 voltage setting (hysteretic mode) */
  #define DIGITAL_BUCK0VOUTULP_ResetValue (0x00UL)   /*!< Reset value of BUCK0VOUTULP register.                                */

/* VOLTAGE @Bits 0..3 : BUCK0 voltage setting for hysteretic mode. Setting must be identical with register BUCK0VOUTPWM. */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_Msk (0xFUL << DIGITAL_BUCK0VOUTULP_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.         */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_Min (0x0UL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_Max (0xFUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET1V8 (0x0UL) /*!< SET 1V8                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET1V9 (0x1UL) /*!< SET 1V9                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V0 (0x2UL) /*!< SET 2V0                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V1 (0x3UL) /*!< SET 2V1                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V2 (0x4UL) /*!< SET 2V2                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V3 (0x5UL) /*!< SET 2V3                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V4 (0x6UL) /*!< SET 2V4                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V5 (0x7UL) /*!< SET 2V5                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V6 (0x8UL) /*!< SET 2V6                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V7 (0x9UL) /*!< SET 2V7                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V8 (0xAUL) /*!< SET 2V8                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET2V9 (0xBUL) /*!< SET 2V9                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET3V0 (0xCUL) /*!< SET 3V0                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET3V1 (0xDUL) /*!< SET 3V1                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET3V2 (0xEUL) /*!< SET 3V2                                                             */
  #define DIGITAL_BUCK0VOUTULP_VOLTAGE_SET3V3 (0xFUL) /*!< SET 3V3                                                             */

/* RESERVED0 @Bit 4 : Reserved0 */
  #define DIGITAL_BUCK0VOUTULP_RESERVED0_Pos (4UL)   /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_BUCK0VOUTULP_RESERVED0_Msk (0x1UL << DIGITAL_BUCK0VOUTULP_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.   */


/* DIGITAL_BUCK0VOUTPWM: BUCK0 voltage setting (PWM mode) */
  #define DIGITAL_BUCK0VOUTPWM_ResetValue (0x00UL)   /*!< Reset value of BUCK0VOUTPWM register.                                */

/* VOLTAGE @Bits 0..3 : BUCK0 voltage setting for PWM mode. After updating this register, run TASK_UPDATE_VOUTPWM. */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_Msk (0xFUL << DIGITAL_BUCK0VOUTPWM_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.         */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_Min (0x0UL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_Max (0xFUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET1V8 (0x0UL) /*!< SET 1V8                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET1V9 (0x1UL) /*!< SET 1V9                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V0 (0x2UL) /*!< SET 2V0                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V1 (0x3UL) /*!< SET 2V1                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V2 (0x4UL) /*!< SET 2V2                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V3 (0x5UL) /*!< SET 2V3                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V4 (0x6UL) /*!< SET 2V4                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V5 (0x7UL) /*!< SET 2V5                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V6 (0x8UL) /*!< SET 2V6                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V7 (0x9UL) /*!< SET 2V7                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V8 (0xAUL) /*!< SET 2V8                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET2V9 (0xBUL) /*!< SET 2V9                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET3V0 (0xCUL) /*!< SET 3V0                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET3V1 (0xDUL) /*!< SET 3V1                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET3V2 (0xEUL) /*!< SET 3V2                                                             */
  #define DIGITAL_BUCK0VOUTPWM_VOLTAGE_SET3V3 (0xFUL) /*!< SET 3V3                                                             */

/* RESERVED0 @Bit 4 : Reserved0 */
  #define DIGITAL_BUCK0VOUTPWM_RESERVED0_Pos (4UL)   /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_BUCK0VOUTPWM_RESERVED0_Msk (0x1UL << DIGITAL_BUCK0VOUTPWM_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.   */


/* DIGITAL_BUCK1VOUTULP: BUCK1 voltage setting (hysteretic mode) */
  #define DIGITAL_BUCK1VOUTULP_ResetValue (0x02UL)   /*!< Reset value of BUCK1VOUTULP register.                                */

/* VOLTAGE @Bits 0..3 : BUCK1 voltage setting for hysteretic mode. Setting must be identical with register BUCK1VOUTPWM. */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_Msk (0xFUL << DIGITAL_BUCK1VOUTULP_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.         */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_Min (0x0UL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_Max (0xEUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V70 (0x0UL) /*!< SET 0V70                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V75 (0x1UL) /*!< SET 0V75                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V80 (0x2UL) /*!< SET 0V80                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V85 (0x3UL) /*!< SET 0V85                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V90 (0x4UL) /*!< SET 0V90                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET0V95 (0x5UL) /*!< SET 0V95                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V00 (0x6UL) /*!< SET 1V00                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V05 (0x7UL) /*!< SET 1V05                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V10 (0x8UL) /*!< SET 1V10                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V15 (0x9UL) /*!< SET 1V15                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V20 (0xAUL) /*!< SET 1V20                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V25 (0xBUL) /*!< SET 1V25                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V30 (0xCUL) /*!< SET 1V30                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V35 (0xDUL) /*!< SET 1V35                                                           */
  #define DIGITAL_BUCK1VOUTULP_VOLTAGE_SET1V40 (0xEUL) /*!< SET 1V40                                                           */

/* RESERVED0 @Bit 4 : Reserved0 */
  #define DIGITAL_BUCK1VOUTULP_RESERVED0_Pos (4UL)   /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_BUCK1VOUTULP_RESERVED0_Msk (0x1UL << DIGITAL_BUCK1VOUTULP_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.   */


/* DIGITAL_BUCK1VOUTPWM: BUCK1 voltage setting (PWM mode) */
  #define DIGITAL_BUCK1VOUTPWM_ResetValue (0x02UL)   /*!< Reset value of BUCK1VOUTPWM register.                                */

/* VOLTAGE @Bits 0..3 : BUCK1 voltage setting for PWM mode. After updating this register, run TASK_UPDATE_VOUTPWM */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_Msk (0xFUL << DIGITAL_BUCK1VOUTPWM_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.         */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_Min (0x0UL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_Max (0xEUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V70 (0x0UL) /*!< SET 0V70                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V75 (0x1UL) /*!< SET 0V75                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V80 (0x2UL) /*!< SET 0V80                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V85 (0x3UL) /*!< SET 0V85                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V90 (0x4UL) /*!< SET 0V90                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET0V95 (0x5UL) /*!< SET 0V95                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V00 (0x6UL) /*!< SET 1V00                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V05 (0x7UL) /*!< SET 1V05                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V10 (0x8UL) /*!< SET 1V10                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V15 (0x9UL) /*!< SET 1V15                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V20 (0xAUL) /*!< SET 1V20                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V25 (0xBUL) /*!< SET 1V25                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V30 (0xCUL) /*!< SET 1V30                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V35 (0xDUL) /*!< SET 1V35                                                           */
  #define DIGITAL_BUCK1VOUTPWM_VOLTAGE_SET1V40 (0xEUL) /*!< SET 1V40                                                           */

/* RESERVED0 @Bit 4 : Reserved0 */
  #define DIGITAL_BUCK1VOUTPWM_RESERVED0_Pos (4UL)   /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_BUCK1VOUTPWM_RESERVED0_Msk (0x1UL << DIGITAL_BUCK1VOUTPWM_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.   */


/* DIGITAL_BUCK2VOUTULP: BUCK2 voltage setting (hysteretic mode) */
  #define DIGITAL_BUCK2VOUTULP_ResetValue (0x0AUL)   /*!< Reset value of BUCK2VOUTULP register.                                */

/* VOLTAGE @Bits 0..4 : BUCK2 voltage setting for hysteretic mode. Setting must be identical with register BUCK2VOUTPWM. */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_Msk (0x1FUL << DIGITAL_BUCK2VOUTULP_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.        */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_Min (0xAUL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_Max (0xEUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_SET1V20 (0x0AUL) /*!< SET 1V20                                                          */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_SET1V25 (0x0BUL) /*!< SET 1V25                                                          */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_SET1V30 (0x0CUL) /*!< SET 1V30                                                          */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_SET1V35 (0x0DUL) /*!< SET 1V35                                                          */
  #define DIGITAL_BUCK2VOUTULP_VOLTAGE_SET1V40 (0x0EUL) /*!< SET 1V40                                                          */


/* DIGITAL_BUCK2VOUTPWM: BUCK2 voltage setting (PWM mode) */
  #define DIGITAL_BUCK2VOUTPWM_ResetValue (0x0AUL)   /*!< Reset value of BUCK2VOUTPWM register.                                */

/* VOLTAGE @Bits 0..4 : BUCK2 voltage setting for PWM-mode. After updating this register, run TASK_UPDATE_VOUTPWM. */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_Pos (0UL)     /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_Msk (0x1FUL << DIGITAL_BUCK2VOUTPWM_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.        */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_Min (0xAUL)   /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_Max (0xEUL)   /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_SET1V20 (0x0AUL) /*!< SET 1V20                                                          */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_SET1V25 (0x0BUL) /*!< SET 1V25                                                          */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_SET1V30 (0x0CUL) /*!< SET 1V30                                                          */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_SET1V35 (0x0DUL) /*!< SET 1V35                                                          */
  #define DIGITAL_BUCK2VOUTPWM_VOLTAGE_SET1V40 (0x0EUL) /*!< SET 1V40                                                          */


/* DIGITAL_BUCK3SELDAC: Internal DAC enable for BUCK3 */
  #define DIGITAL_BUCK3SELDAC_ResetValue (0x00UL)    /*!< Reset value of BUCK3SELDAC register.                                 */

/* SELECT @Bit 0 : BUCK3 internal DAC */
  #define DIGITAL_BUCK3SELDAC_SELECT_Pos (0UL)       /*!< Position of SELECT field.                                            */
  #define DIGITAL_BUCK3SELDAC_SELECT_Msk (0x1UL << DIGITAL_BUCK3SELDAC_SELECT_Pos) /*!< Bit mask of SELECT field.              */
  #define DIGITAL_BUCK3SELDAC_SELECT_Min (0x0UL)     /*!< Min enumerator value of SELECT field.                                */
  #define DIGITAL_BUCK3SELDAC_SELECT_Max (0x1UL)     /*!< Max enumerator value of SELECT field.                                */
  #define DIGITAL_BUCK3SELDAC_SELECT_DISABLE (0x0UL) /*!< Disable                                                              */
  #define DIGITAL_BUCK3SELDAC_SELECT_ENABLE (0x1UL)  /*!< Enable                                                               */


/* DIGITAL_BUCK3VOUT: BUCK3 voltage setting */
  #define DIGITAL_BUCK3VOUT_ResetValue (0x00UL)      /*!< Reset value of BUCK3VOUT register.                                   */

/* VOLTAGE @Bits 0..6 : BUCK3 voltage setting */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_Pos (0UL)        /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_Msk (0x7FUL << DIGITAL_BUCK3VOUT_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_Min (0x0UL)      /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_Max (0x70UL)     /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V5 (0x00UL)  /*!< SET 0V5                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V525 (0x01UL) /*!< SET 0V525                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V55 (0x02UL) /*!< SET 0V55                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V575 (0x03UL) /*!< SET 0V575                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V6 (0x04UL)  /*!< SET 0V6                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V625 (0x05UL) /*!< SET 0V625                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V65 (0x06UL) /*!< SET 0V65                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V675 (0x07UL) /*!< SET 0V675                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V7 (0x08UL)  /*!< SET 0V7                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V725 (0x09UL) /*!< SET 0V725                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V75 (0x0AUL) /*!< SET 0V75                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V775 (0x0BUL) /*!< SET 0V775                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V8 (0x0CUL)  /*!< SET 0V8                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V825 (0x0DUL) /*!< SET 0V825                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V85 (0x0EUL) /*!< SET 0V85                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V875 (0x0FUL) /*!< SET 0V875                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V9 (0x10UL)  /*!< SET 0V9                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V925 (0x11UL) /*!< SET 0V925                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V95 (0x12UL) /*!< SET 0V95                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET0V975 (0x13UL) /*!< SET 0V975                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V0 (0x14UL)  /*!< SET 1V0                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V025 (0x15UL) /*!< SET 1V025                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V05 (0x16UL) /*!< SET 1V05                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V075 (0x17UL) /*!< SET 1V075                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V1 (0x18UL)  /*!< SET 1V1                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V125 (0x19UL) /*!< SET 1V125                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V15 (0x1AUL) /*!< SET 1V15                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V175 (0x1BUL) /*!< SET 1V175                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V2 (0x1CUL)  /*!< SET 1V2                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V225 (0x1DUL) /*!< SET 1V225                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V25 (0x1EUL) /*!< SET 1V25                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V275 (0x1FUL) /*!< SET 1V275                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V3 (0x20UL)  /*!< SET 1V3                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V325 (0x21UL) /*!< SET 1V325                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V35 (0x22UL) /*!< SET 1V35                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V375 (0x23UL) /*!< SET 1V375                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V4 (0x24UL)  /*!< SET 1V4                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V425 (0x25UL) /*!< SET 1V425                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V45 (0x26UL) /*!< SET 1V45                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V475 (0x27UL) /*!< SET 1V475                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V5 (0x28UL)  /*!< SET 1V5                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V525 (0x29UL) /*!< SET 1V525                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V55 (0x2AUL) /*!< SET 1V55                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V575 (0x2BUL) /*!< SET 1V575                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V6 (0x2CUL)  /*!< SET 1V6                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V625 (0x2DUL) /*!< SET 1V625                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V65 (0x2EUL) /*!< SET 1V65                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V675 (0x2FUL) /*!< SET 1V675                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V7 (0x30UL)  /*!< SET 1V7                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V725 (0x31UL) /*!< SET 1V725                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V75 (0x32UL) /*!< SET 1V75                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V775 (0x33UL) /*!< SET 1V775                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V8 (0x34UL)  /*!< SET 1V8                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V825 (0x35UL) /*!< SET 1V825                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V85 (0x36UL) /*!< SET 1V85                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V875 (0x37UL) /*!< SET 1V875                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V9 (0x38UL)  /*!< SET 1V9                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V925 (0x39UL) /*!< SET 1V925                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V95 (0x3AUL) /*!< SET 1V95                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET1V975 (0x3BUL) /*!< SET 1V975                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V0 (0x3CUL)  /*!< SET 2V0                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V025 (0x3DUL) /*!< SET 2V025                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V05 (0x3EUL) /*!< SET 2V05                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V075 (0x3FUL) /*!< SET 2V075                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V1 (0x40UL)  /*!< SET 2V1                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V125 (0x41UL) /*!< SET 2V125                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V15 (0x42UL) /*!< SET 2V15                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V175 (0x43UL) /*!< SET 2V175                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V2 (0x44UL)  /*!< SET 2V2                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V225 (0x45UL) /*!< SET 2V225                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V25 (0x46UL) /*!< SET 2V25                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V275 (0x47UL) /*!< SET 2V275                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V3 (0x48UL)  /*!< SET 2V3                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V325 (0x49UL) /*!< SET 2V325                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V35 (0x4AUL) /*!< SET 2V35                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V375 (0x4BUL) /*!< SET 2V375                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V4 (0x4CUL)  /*!< SET 2V4                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V425 (0x4DUL) /*!< SET 2V425                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V45 (0x4EUL) /*!< SET 2V45                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V475 (0x4FUL) /*!< SET 2V475                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V5 (0x50UL)  /*!< SET 2V5                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V525 (0x51UL) /*!< SET 2V525                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V55 (0x52UL) /*!< SET 2V55                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V575 (0x53UL) /*!< SET 2V575                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V6 (0x54UL)  /*!< SET 2V6                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V625 (0x55UL) /*!< SET 2V625                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V65 (0x56UL) /*!< SET 2V65                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V675 (0x57UL) /*!< SET 2V675                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V7 (0x58UL)  /*!< SET 2V7                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V725 (0x59UL) /*!< SET 2V725                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V75 (0x5AUL) /*!< SET 2V75                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V775 (0x5BUL) /*!< SET 2V775                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V8 (0x5CUL)  /*!< SET 2V8                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V825 (0x5DUL) /*!< SET 2V825                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V85 (0x5EUL) /*!< SET 2V85                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V875 (0x5FUL) /*!< SET 2V875                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V9 (0x60UL)  /*!< SET 2V9                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V925 (0x61UL) /*!< SET 2V925                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V95 (0x62UL) /*!< SET 2V95                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET2V975 (0x63UL) /*!< SET 2V975                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V0 (0x64UL)  /*!< SET 3V0                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V025 (0x65UL) /*!< SET 3V025                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V05 (0x66UL) /*!< SET 3V05                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V075 (0x67UL) /*!< SET 3V075                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V1 (0x68UL)  /*!< SET 3V1                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V125 (0x69UL) /*!< SET 3V125                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V15 (0x6AUL) /*!< SET 3V15                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V175 (0x6BUL) /*!< SET 3V175                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V2 (0x6CUL)  /*!< SET 3V2                                                              */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V225 (0x6DUL) /*!< SET 3V225                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V25 (0x6EUL) /*!< SET 3V25                                                             */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V275 (0x6FUL) /*!< SET 3V275                                                           */
  #define DIGITAL_BUCK3VOUT_VOLTAGE_SET3V3 (0x70UL)  /*!< SET 3V3                                                              */


/* DIGITAL_LDO0VOUT: LDO0 voltage setting */
  #define DIGITAL_LDO0VOUT_ResetValue (0x1AUL)       /*!< Reset value of LDO0VOUT register.                                    */

/* VOLTAGE @Bits 0..4 : LDO0 voltage setting */
  #define DIGITAL_LDO0VOUT_VOLTAGE_Pos (0UL)         /*!< Position of VOLTAGE field.                                           */
  #define DIGITAL_LDO0VOUT_VOLTAGE_Msk (0x1FUL << DIGITAL_LDO0VOUT_VOLTAGE_Pos) /*!< Bit mask of VOLTAGE field.                */
  #define DIGITAL_LDO0VOUT_VOLTAGE_Min (0x6UL)       /*!< Min enumerator value of VOLTAGE field.                               */
  #define DIGITAL_LDO0VOUT_VOLTAGE_Max (0x1EUL)      /*!< Max enumerator value of VOLTAGE field.                               */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET1V8 (0x06UL)   /*!< SET 1.8V                                                             */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET2V1 (0x0BUL)   /*!< SET 2.1V                                                             */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET2V41 (0x10UL)  /*!< SET 2.41V                                                            */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET2V7 (0x15UL)   /*!< SET 2.7V                                                             */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET3V0 (0x1AUL)   /*!< SET 3.0V                                                             */
  #define DIGITAL_LDO0VOUT_VOLTAGE_SET3V3 (0x1EUL)   /*!< SET 3.3V                                                             */

/* RESERVED0 @Bit 5 : Reserved0 */
  #define DIGITAL_LDO0VOUT_RESERVED0_Pos (5UL)       /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_LDO0VOUT_RESERVED0_Msk (0x1UL << DIGITAL_LDO0VOUT_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.           */


/* DIGITAL_BUCK0CONFPWMMODE: BUCK0 PWM mode configuration */
  #define DIGITAL_BUCK0CONFPWMMODE_ResetValue (0x03UL) /*!< Reset value of BUCK0CONFPWMMODE register.                          */

/* PADBUCKMODE0 @Bit 0 : Configure pin BUCK_MODE0 for BUCK0 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Pos (0UL) /*!< Position of PADBUCKMODE0 field.                                 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Msk (0x1UL << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE0 field.*/                                 
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_NOPWM (0x0UL) /*!< BUCK0 is not set to PWM mode when pin BUCK_MODE0 is high    */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE0_PWM (0x1UL) /*!< BUCK0 is set to PWM mode when pin BUCK_MODE0 is high          */

/* PADBUCKMODE1 @Bit 1 : Configure pin BUCK_MODE1 for BUCK0 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Pos (1UL) /*!< Position of PADBUCKMODE1 field.                                 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Msk (0x1UL << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE1 field.*/                                 
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_NOPWM (0x0UL) /*!< BUCK0 is not set to PWM mode when pin BUCK_MODE1 is high    */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE1_PWM (0x1UL) /*!< BUCK0 is set to PWM mode when pin BUCK_MODE1 is high          */

/* PADBUCKMODE2 @Bit 2 : Configure pin BUCK_MODE2 for BUCK0 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Pos (2UL) /*!< Position of PADBUCKMODE2 field.                                 */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Msk (0x1UL << DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE2 field.*/                                 
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_NOPWM (0x0UL) /*!< BUCK0 is not set to PWM mode when pin BUCK_MODE2 is high    */
  #define DIGITAL_BUCK0CONFPWMMODE_PADBUCKMODE2_PWM (0x1UL) /*!< BUCK0 is set to PWM mode when pin BUCK_MODE2 is high          */

/* SETFORCEPWM @Bit 3 : Set BUCK0 to PWM mode */
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Pos (3UL) /*!< Position of SETFORCEPWM field.                                   */
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Msk (0x1UL << DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Pos) /*!< Bit mask of
                                                                            SETFORCEPWM field.*/                                  
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Min (0x0UL) /*!< Min enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_Max (0x1UL) /*!< Max enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_OFF (0x0UL) /*!< BUCK0 PWM mode is controlled by BUCK_MODE pins                 */
  #define DIGITAL_BUCK0CONFPWMMODE_SETFORCEPWM_ON (0x1UL) /*!< BUCK0 is set to PWM mode regardless of BUCK_MODE pin states     */


/* DIGITAL_BUCK1CONFPWMMODE: BUCK1 PWM mode configuration */
  #define DIGITAL_BUCK1CONFPWMMODE_ResetValue (0x02UL) /*!< Reset value of BUCK1CONFPWMMODE register.                          */

/* PADBUCKMODE0 @Bit 0 : Configure pin BUCK_MODE0 for BUCK1 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_Pos (0UL) /*!< Position of PADBUCKMODE0 field.                                 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_Msk (0x1UL << DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE0 field.*/                                 
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_NOPWM (0x0UL) /*!< BUCK1 is not set to PWM mode when pin BUCK_MODE0 is high    */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE0_PWM (0x1UL) /*!< BUCK1 is set to PWM mode when pin BUCK_MODE0 is high          */

/* PADBUCKMODE1 @Bit 1 : Configure pin BUCK_MODE1 for BUCK1 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_Pos (1UL) /*!< Position of PADBUCKMODE1 field.                                 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_Msk (0x1UL << DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE1 field.*/                                 
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_NOPWM (0x0UL) /*!< BUCK1 is not set to PWM mode when pin BUCK_MODE1 is high    */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE1_PWM (0x1UL) /*!< BUCK1 is set to PWM mode when pin BUCK_MODE1 is high          */

/* PADBUCKMODE2 @Bit 2 : Configure pin BUCK_MODE2 for BUCK1 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_Pos (2UL) /*!< Position of PADBUCKMODE2 field.                                 */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_Msk (0x1UL << DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE2 field.*/                                 
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_NOPWM (0x0UL) /*!< BUCK1 is not set to PWM mode when pin BUCK_MODE2 is high    */
  #define DIGITAL_BUCK1CONFPWMMODE_PADBUCKMODE2_PWM (0x1UL) /*!< BUCK1 is set to PWM mode when pin BUCK_MODE2 is high          */

/* SETFORCEPWM @Bit 3 : Set BUCK1 to PWM mode */
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_Pos (3UL) /*!< Position of SETFORCEPWM field.                                   */
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_Msk (0x1UL << DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_Pos) /*!< Bit mask of
                                                                            SETFORCEPWM field.*/                                  
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_Min (0x0UL) /*!< Min enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_Max (0x1UL) /*!< Max enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_OFF (0x0UL) /*!< BUCK1 PWM mode is controlled by BUCK_MODE pins                 */
  #define DIGITAL_BUCK1CONFPWMMODE_SETFORCEPWM_ON (0x1UL) /*!< BUCK1 is set to PWM mode regardless of BUCK_MODE pin states     */


/* DIGITAL_BUCK2CONFPWMMODE: BUCK2 PWM mode configuration */
  #define DIGITAL_BUCK2CONFPWMMODE_ResetValue (0x02UL) /*!< Reset value of BUCK2CONFPWMMODE register.                          */

/* PADBUCKMODE0 @Bit 0 : Configure pin BUCK_MODE0 for BUCK2 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_Pos (0UL) /*!< Position of PADBUCKMODE0 field.                                 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_Msk (0x1UL << DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE0 field.*/                                 
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_NOPWM (0x0UL) /*!< BUCK2 is not set to PWM mode when pin BUCK_MODE0 is high    */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE0_PWM (0x1UL) /*!< BUCK2 set to PWM mode when pin BUCK_MODE0 is high             */

/* PADBUCKMODE1 @Bit 1 : Configure pin BUCK_MODE1 for BUCK2 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_Pos (1UL) /*!< Position of PADBUCKMODE1 field.                                 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_Msk (0x1UL << DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE1 field.*/                                 
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_NOPWM (0x0UL) /*!< BUCK2 is not set to PWM mode when pin BUCK_MODE1 is high    */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE1_PWM (0x1UL) /*!< BUCK2 is set to PWM mode when pin BUCK_MODE1 is high          */

/* PADBUCKMODE2 @Bit 2 : Configure pin BUCK_MODE2 for BUCK2 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_Pos (2UL) /*!< Position of PADBUCKMODE2 field.                                 */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_Msk (0x1UL << DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE2 field.*/                                 
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_NOPWM (0x0UL) /*!< BUCK2 is not set to PWM mode when pin BUCK_MODE2 is high    */
  #define DIGITAL_BUCK2CONFPWMMODE_PADBUCKMODE2_PWM (0x1UL) /*!< BUCK2 is set to PWM mode when pin BUCK_MODE2 is high          */

/* SETFORCEPWM @Bit 3 : Set BUCK2 to PWM mode */
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_Pos (3UL) /*!< Position of SETFORCEPWM field.                                   */
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_Msk (0x1UL << DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_Pos) /*!< Bit mask of
                                                                            SETFORCEPWM field.*/                                  
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_Min (0x0UL) /*!< Min enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_Max (0x1UL) /*!< Max enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_OFF (0x0UL) /*!< BUCK2 PWM mode is controlled by BUCK_MODE pins                 */
  #define DIGITAL_BUCK2CONFPWMMODE_SETFORCEPWM_ON (0x1UL) /*!< BUCK2 is set to PWM mode regardless of BUCK_MODE pin states     */


/* DIGITAL_BUCK3CONFPWMMODE: BUCK3 PWM mode configuration */
  #define DIGITAL_BUCK3CONFPWMMODE_ResetValue (0x04UL) /*!< Reset value of BUCK3CONFPWMMODE register.                          */

/* PADBUCKMODE0 @Bit 0 : Configure pin BUCK_MODE0 for BUCK3 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_Pos (0UL) /*!< Position of PADBUCKMODE0 field.                                 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_Msk (0x1UL << DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE0 field.*/                                 
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE0 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_NOPWM (0x0UL) /*!< BUCK3 is not set to PWM mode when pin BUCK_MODE0 is high    */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE0_PWM (0x1UL) /*!< BUCK3 is set to PWM mode when pin BUCK_MODE0 is high          */

/* PADBUCKMODE1 @Bit 1 : Configure pin BUCK_MODE1 for BUCK3 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_Pos (1UL) /*!< Position of PADBUCKMODE1 field.                                 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_Msk (0x1UL << DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE1 field.*/                                 
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE1 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_NOPWM (0x0UL) /*!< BUCK3 is not set to PWM mode when pin BUCK_MODE1 is high    */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE1_PWM (0x1UL) /*!< BUCK3 is set to PWM mode when pin BUCK_MODE1 is high          */

/* PADBUCKMODE2 @Bit 2 : Configure pin BUCK_MODE2 for BUCK3 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_Pos (2UL) /*!< Position of PADBUCKMODE2 field.                                 */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_Msk (0x1UL << DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_Pos) /*!< Bit mask of
                                                                            PADBUCKMODE2 field.*/                                 
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_Min (0x0UL) /*!< Min enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_Max (0x1UL) /*!< Max enumerator value of PADBUCKMODE2 field.                   */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_NOPWM (0x0UL) /*!< BUCK3 is not set to PWM mode when pin BUCK_MODE2 is high    */
  #define DIGITAL_BUCK3CONFPWMMODE_PADBUCKMODE2_PWM (0x1UL) /*!< BUCK3 is set to PWM mode when pin BUCK_MODE2 is high          */

/* SETFORCEPWM @Bit 3 : Set BUCK3 to PWM mode */
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Pos (3UL) /*!< Position of SETFORCEPWM field.                                   */
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Msk (0x1UL << DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Pos) /*!< Bit mask of
                                                                            SETFORCEPWM field.*/                                  
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Min (0x0UL) /*!< Min enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_Max (0x1UL) /*!< Max enumerator value of SETFORCEPWM field.                     */
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_OFF (0x0UL) /*!< BUCK3 PWM mode is controlled by BUCK_MODE pins                 */
  #define DIGITAL_BUCK3CONFPWMMODE_SETFORCEPWM_ON (0x1UL) /*!< BUCK3 is set to PWM mode regardless of BUCK_MODE pin states     */


/* DIGITAL_BUCKMODEPADCONF: BUCK_MODE pin configuration */
  #define DIGITAL_BUCKMODEPADCONF_ResetValue (0x00UL) /*!< Reset value of BUCKMODEPADCONF register.                            */

/* BUCKMODE0PADTYPE @Bit 0 : BUCK_MODE0 input type */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Pos (0UL) /*!< Position of BUCKMODE0PADTYPE field.                          */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Pos) /*!< Bit mask of
                                                                            BUCKMODE0PADTYPE field.*/                             
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Min (0x0UL) /*!< Min enumerator value of BUCKMODE0PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_Max (0x1UL) /*!< Max enumerator value of BUCKMODE0PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_SCHMITT (0x0UL) /*!< Schmitt trigger input                                  */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PADTYPE_CMOS (0x1UL) /*!< CMOS input                                                */

/* BUCKMODE1PADTYPE @Bit 1 : BUCK_MODE1 input type */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Pos (1UL) /*!< Position of BUCKMODE1PADTYPE field.                          */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Pos) /*!< Bit mask of
                                                                            BUCKMODE1PADTYPE field.*/                             
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Min (0x0UL) /*!< Min enumerator value of BUCKMODE1PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_Max (0x1UL) /*!< Max enumerator value of BUCKMODE1PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_SCHMITT (0x0UL) /*!< Schmitt trigger input                                  */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PADTYPE_CMOS (0x1UL) /*!< CMOS input                                                */

/* BUCKMODE2PADTYPE @Bit 2 : BUCK_MODE2 input type */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Pos (2UL) /*!< Position of BUCKMODE2PADTYPE field.                          */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Pos) /*!< Bit mask of
                                                                            BUCKMODE2PADTYPE field.*/                             
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Min (0x0UL) /*!< Min enumerator value of BUCKMODE2PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_Max (0x1UL) /*!< Max enumerator value of BUCKMODE2PADTYPE field.            */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_SCHMITT (0x0UL) /*!< Schmitt trigger input                                  */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PADTYPE_CMOS (0x1UL) /*!< CMOS input                                                */

/* BUCKMODE0PULLD @Bit 4 : BUCK_MODE0 pulldown */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Pos (4UL) /*!< Position of BUCKMODE0PULLD field.                              */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Pos) /*!< Bit mask of
                                                                            BUCKMODE0PULLD field.*/                               
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Min (0x0UL) /*!< Min enumerator value of BUCKMODE0PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_Max (0x1UL) /*!< Max enumerator value of BUCKMODE0PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_DISABLED (0x0UL) /*!< Disabled                                                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE0PULLD_ENABLED (0x1UL) /*!< Enabled                                                  */

/* BUCKMODE1PULLD @Bit 5 : BUCK_MODE1 pulldown */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Pos (5UL) /*!< Position of BUCKMODE1PULLD field.                              */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Pos) /*!< Bit mask of
                                                                            BUCKMODE1PULLD field.*/                               
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Min (0x0UL) /*!< Min enumerator value of BUCKMODE1PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_Max (0x1UL) /*!< Max enumerator value of BUCKMODE1PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_DISABLED (0x0UL) /*!< Disabled                                                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE1PULLD_ENABLED (0x1UL) /*!< Enabled                                                  */

/* BUCKMODE2PULLD @Bit 6 : BUCK_MODE2 pulldown */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Pos (6UL) /*!< Position of BUCKMODE2PULLD field.                              */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Msk (0x1UL << DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Pos) /*!< Bit mask of
                                                                            BUCKMODE2PULLD field.*/                               
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Min (0x0UL) /*!< Min enumerator value of BUCKMODE2PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_Max (0x1UL) /*!< Max enumerator value of BUCKMODE2PULLD field.                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_DISABLED (0x0UL) /*!< Disabled                                                */
  #define DIGITAL_BUCKMODEPADCONF_BUCKMODE2PULLD_ENABLED (0x1UL) /*!< Enabled                                                  */


/* DIGITAL_THDYNPOWERUP: Thermal sensors' dynamic configuration */
  #define DIGITAL_THDYNPOWERUP_ResetValue (0x00UL)   /*!< Reset value of THDYNPOWERUP register.                                */

/* BUCK0PWM @Bit 0 : BUCK0 PWM mode controls thermal sensor */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_Pos (0UL)    /*!< Position of BUCK0PWM field.                                          */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_Msk (0x1UL << DIGITAL_THDYNPOWERUP_BUCK0PWM_Pos) /*!< Bit mask of BUCK0PWM field.      */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_Min (0x0UL)  /*!< Min enumerator value of BUCK0PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_Max (0x1UL)  /*!< Max enumerator value of BUCK0PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_NOEFFECT (0x0UL) /*!< Thermal sensor is not enabled while BUCK0 is in PWM mode         */
  #define DIGITAL_THDYNPOWERUP_BUCK0PWM_PWRUP (0x1UL) /*!< Thermal sensor is enabled while BUCK0 is in PWM mode                */

/* BUCK1PWM @Bit 1 : BUCK1 PWM mode controls thermal sensor */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_Pos (1UL)    /*!< Position of BUCK1PWM field.                                          */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_Msk (0x1UL << DIGITAL_THDYNPOWERUP_BUCK1PWM_Pos) /*!< Bit mask of BUCK1PWM field.      */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_Min (0x0UL)  /*!< Min enumerator value of BUCK1PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_Max (0x1UL)  /*!< Max enumerator value of BUCK1PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_NOEFFECT (0x0UL) /*!< Thermal sensor is not enabled while BUCK1 is in PWM mode         */
  #define DIGITAL_THDYNPOWERUP_BUCK1PWM_PWRUP (0x1UL) /*!< Thermal sensor is enabled while BUCK1 is in PWM mode                */

/* BUCK2PWM @Bit 2 : BUCK2 PWM mode controls thermal sensor */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_Pos (2UL)    /*!< Position of BUCK2PWM field.                                          */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_Msk (0x1UL << DIGITAL_THDYNPOWERUP_BUCK2PWM_Pos) /*!< Bit mask of BUCK2PWM field.      */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_Min (0x0UL)  /*!< Min enumerator value of BUCK2PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_Max (0x1UL)  /*!< Max enumerator value of BUCK2PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_NOEFFECT (0x0UL) /*!< Thermal sensor is not enabled while BUCK2 is in PWM mode         */
  #define DIGITAL_THDYNPOWERUP_BUCK2PWM_PWRUP (0x1UL) /*!< Thermal sensor is enabled while BUCK2 is in PWM mode                */

/* BUCK3PWM @Bit 3 : BUCK3 PWM mode controls thermal sensor */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_Pos (3UL)    /*!< Position of BUCK3PWM field.                                          */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_Msk (0x1UL << DIGITAL_THDYNPOWERUP_BUCK3PWM_Pos) /*!< Bit mask of BUCK3PWM field.      */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_Min (0x0UL)  /*!< Min enumerator value of BUCK3PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_Max (0x1UL)  /*!< Max enumerator value of BUCK3PWM field.                              */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_NOEFFECT (0x0UL) /*!< Thermal sensor is not enabled while BUCK3 is in PWM mode         */
  #define DIGITAL_THDYNPOWERUP_BUCK3PWM_PWRUP (0x1UL) /*!< Thermal sensor is enabled while BUCK3 is in PWM mode                */

/* WARNING @Bit 4 : Thermal warning sensor */
  #define DIGITAL_THDYNPOWERUP_WARNING_Pos (4UL)     /*!< Position of WARNING field.                                           */
  #define DIGITAL_THDYNPOWERUP_WARNING_Msk (0x1UL << DIGITAL_THDYNPOWERUP_WARNING_Pos) /*!< Bit mask of WARNING field.         */
  #define DIGITAL_THDYNPOWERUP_WARNING_Min (0x0UL)   /*!< Min enumerator value of WARNING field.                               */
  #define DIGITAL_THDYNPOWERUP_WARNING_Max (0x1UL)   /*!< Max enumerator value of WARNING field.                               */
  #define DIGITAL_THDYNPOWERUP_WARNING_NOTSELECTED (0x0UL) /*!< Thermal warning sensor is not enabled dynamically              */
  #define DIGITAL_THDYNPOWERUP_WARNING_SELECTED (0x1UL) /*!< Thermal warning sensor is enabled dynamically                     */

/* SHUTDWN @Bit 5 : Thermal shutdown sensor */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_Pos (5UL)     /*!< Position of SHUTDWN field.                                           */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_Msk (0x1UL << DIGITAL_THDYNPOWERUP_SHUTDWN_Pos) /*!< Bit mask of SHUTDWN field.         */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_Min (0x0UL)   /*!< Min enumerator value of SHUTDWN field.                               */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_Max (0x1UL)   /*!< Max enumerator value of SHUTDWN field.                               */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_NOTSELECTED (0x0UL) /*!< Thermal shutdown sensor is not enabled dynamically             */
  #define DIGITAL_THDYNPOWERUP_SHUTDWN_SELECTED (0x1UL) /*!< Thermal shutdown sensor is enabled dynamically                    */


/* DIGITAL_PADDRIVESTRENGTH: Drive strength control */
  #define DIGITAL_PADDRIVESTRENGTH_ResetValue (0x00UL) /*!< Reset value of PADDRIVESTRENGTH register.                          */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED0_Pos (0UL) /*!< Position of RESERVED0 field.                                       */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED0_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_RESERVED0_Pos) /*!< Bit mask of RESERVED0
                                                                            field.*/                                              

/* RESERVED1 @Bit 1 : Reserved1 */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED1_Pos (1UL) /*!< Position of RESERVED1 field.                                       */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED1_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_RESERVED1_Pos) /*!< Bit mask of RESERVED1
                                                                            field.*/                                              

/* READY @Bit 2 : Drive strength for pin READY */
  #define DIGITAL_PADDRIVESTRENGTH_READY_Pos (2UL)   /*!< Position of READY field.                                             */
  #define DIGITAL_PADDRIVESTRENGTH_READY_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_READY_Pos) /*!< Bit mask of READY field.       */
  #define DIGITAL_PADDRIVESTRENGTH_READY_Min (0x0UL) /*!< Min enumerator value of READY field.                                 */
  #define DIGITAL_PADDRIVESTRENGTH_READY_Max (0x1UL) /*!< Max enumerator value of READY field.                                 */
  #define DIGITAL_PADDRIVESTRENGTH_READY_NORMAL (0x0UL) /*!< Normal drive strength                                             */
  #define DIGITAL_PADDRIVESTRENGTH_READY_HIGH (0x1UL) /*!< High drive strength                                                 */

/* NINT @Bit 3 : Drive strength for pin nINT */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_Pos (3UL)    /*!< Position of NINT field.                                              */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_NINT_Pos) /*!< Bit mask of NINT field.          */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_Min (0x0UL)  /*!< Min enumerator value of NINT field.                                  */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_Max (0x1UL)  /*!< Max enumerator value of NINT field.                                  */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_NORMAL (0x0UL) /*!< Normal drive strength                                              */
  #define DIGITAL_PADDRIVESTRENGTH_NINT_HIGH (0x1UL) /*!< High drive strength                                                  */

/* RESERVED2 @Bit 4 : Reserved2 */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED2_Pos (4UL) /*!< Position of RESERVED2 field.                                       */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED2_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_RESERVED2_Pos) /*!< Bit mask of RESERVED2
                                                                            field.*/                                              

/* SDA @Bit 5 : Drive strength for pin SDA */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_Pos (5UL)     /*!< Position of SDA field.                                               */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_SDA_Pos) /*!< Bit mask of SDA field.             */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_Min (0x0UL)   /*!< Min enumerator value of SDA field.                                   */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_Max (0x1UL)   /*!< Max enumerator value of SDA field.                                   */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_NORMAL (0x0UL) /*!< Normal drive strength                                               */
  #define DIGITAL_PADDRIVESTRENGTH_SDA_HIGH (0x1UL)  /*!< High drive strength                                                  */

/* RESERVED3 @Bit 6 : Reserved3 */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED3_Pos (6UL) /*!< Position of RESERVED3 field.                                       */
  #define DIGITAL_PADDRIVESTRENGTH_RESERVED3_Msk (0x1UL << DIGITAL_PADDRIVESTRENGTH_RESERVED3_Pos) /*!< Bit mask of RESERVED3
                                                                            field.*/                                              


/* DIGITAL_WDARMEDVALUE: Arm watchdog or wake-up timer. Use strobe WDARMEDSTROBE. */
  #define DIGITAL_WDARMEDVALUE_ResetValue (0x00UL)   /*!< Reset value of WDARMEDVALUE register.                                */

/* VALUE @Bit 0 : Arm or disarm watchdog or wake-up timer */
  #define DIGITAL_WDARMEDVALUE_VALUE_Pos (0UL)       /*!< Position of VALUE field.                                             */
  #define DIGITAL_WDARMEDVALUE_VALUE_Msk (0x1UL << DIGITAL_WDARMEDVALUE_VALUE_Pos) /*!< Bit mask of VALUE field.               */
  #define DIGITAL_WDARMEDVALUE_VALUE_Min (0x0UL)     /*!< Min enumerator value of VALUE field.                                 */
  #define DIGITAL_WDARMEDVALUE_VALUE_Max (0x1UL)     /*!< Max enumerator value of VALUE field.                                 */
  #define DIGITAL_WDARMEDVALUE_VALUE_DISABLE (0x0UL) /*!< Disarmed                                                             */
  #define DIGITAL_WDARMEDVALUE_VALUE_ENABLE (0x1UL)  /*!< Armed                                                                */


/* DIGITAL_WDARMEDSTROBE: Strobe for register WDARMEDVALUE */
  #define DIGITAL_WDARMEDSTROBE_ResetValue (0x00UL)  /*!< Reset value of WDARMEDSTROBE register.                               */

/* STROBE @Bit 0 : Strobe for register WDARMEDVALUE */
  #define DIGITAL_WDARMEDSTROBE_STROBE_Pos (0UL)     /*!< Position of STROBE field.                                            */
  #define DIGITAL_WDARMEDSTROBE_STROBE_Msk (0x1UL << DIGITAL_WDARMEDSTROBE_STROBE_Pos) /*!< Bit mask of STROBE field.          */
  #define DIGITAL_WDARMEDSTROBE_STROBE_Min (0x0UL)   /*!< Min enumerator value of STROBE field.                                */
  #define DIGITAL_WDARMEDSTROBE_STROBE_Max (0x1UL)   /*!< Max enumerator value of STROBE field.                                */
  #define DIGITAL_WDARMEDSTROBE_STROBE_NOEFFECT (0x0UL) /*!< No effect                                                         */
  #define DIGITAL_WDARMEDSTROBE_STROBE_STROBE (0x1UL) /*!< Strobe                                                              */


/* DIGITAL_WDTRIGGERVALUE0: Watchdog and wake-up timer trigger value, lowest byte. Use strobe WDDATASTROBE */
  #define DIGITAL_WDTRIGGERVALUE0_ResetValue (0x00UL) /*!< Reset value of WDTRIGGERVALUE0 register.                            */

/* VALUE0 @Bits 0..7 : Watchdog and wakeup timer trigger value is a 24-bit value programmed into three registers. This is the
                       lowest byte. Lsb equals to 4 s. Do not use values 0 and 1. */                                              
                                                                                                                                  
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_Pos (0UL)   /*!< Position of VALUE0 field.                                            */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_Msk (0xFFUL << DIGITAL_WDTRIGGERVALUE0_VALUE0_Pos) /*!< Bit mask of VALUE0 field.     */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_Min (0x0UL) /*!< Min enumerator value of VALUE0 field.                                */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_Max (0x80UL) /*!< Max enumerator value of VALUE0 field.                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL0 (0x00UL) /*!< Sel0                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL2 (0x02UL) /*!< Sel2                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL3 (0x03UL) /*!< Sel3                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL4 (0x04UL) /*!< Sel4                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL5 (0x05UL) /*!< Sel5                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL6 (0x06UL) /*!< Sel6                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL7 (0x07UL) /*!< Sel7                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL8 (0x08UL) /*!< Sel8                                                               */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL16 (0x10UL) /*!< Sel16                                                             */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL32 (0x20UL) /*!< Sel32                                                             */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL64 (0x40UL) /*!< Sel64                                                             */
  #define DIGITAL_WDTRIGGERVALUE0_VALUE0_SEL128 (0x80UL) /*!< Sel128                                                           */


/* DIGITAL_WDTRIGGERVALUE1: Watchdog and wake-up timer trigger value, middle byte. Use strobe WDDATASTROBE */
  #define DIGITAL_WDTRIGGERVALUE1_ResetValue (0x00UL) /*!< Reset value of WDTRIGGERVALUE1 register.                            */

/* VALUE1 @Bits 0..7 : Watchdog and wakeup timer trigger value, middle byte. */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_Pos (0UL)   /*!< Position of VALUE1 field.                                            */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_Msk (0xFFUL << DIGITAL_WDTRIGGERVALUE1_VALUE1_Pos) /*!< Bit mask of VALUE1 field.     */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_Min (0x0UL) /*!< Min enumerator value of VALUE1 field.                                */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_Max (0x80UL) /*!< Max enumerator value of VALUE1 field.                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL0 (0x00UL) /*!< Sel0                                                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL1 (0x01UL) /*!< Sel1                                                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL2 (0x02UL) /*!< Sel2                                                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL4 (0x04UL) /*!< Sel4                                                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL8 (0x08UL) /*!< Sel8                                                               */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL16 (0x10UL) /*!< Sel16                                                             */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL32 (0x20UL) /*!< Sel32                                                             */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL64 (0x40UL) /*!< Sel64                                                             */
  #define DIGITAL_WDTRIGGERVALUE1_VALUE1_SEL128 (0x80UL) /*!< Sel128                                                           */


/* DIGITAL_WDTRIGGERVALUE2: Watchdog and wake-up timer trigger value, highest byte. Use strobe WDDATASTROBE */
  #define DIGITAL_WDTRIGGERVALUE2_ResetValue (0x00UL) /*!< Reset value of WDTRIGGERVALUE2 register.                            */

/* VALUE2 @Bits 0..7 : Watchdog and wakeup timer trigger value, highest byte. */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_Pos (0UL)   /*!< Position of VALUE2 field.                                            */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_Msk (0xFFUL << DIGITAL_WDTRIGGERVALUE2_VALUE2_Pos) /*!< Bit mask of VALUE2 field.     */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_Min (0x0UL) /*!< Min enumerator value of VALUE2 field.                                */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_Max (0x80UL) /*!< Max enumerator value of VALUE2 field.                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL0 (0x00UL) /*!< Sel0                                                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL1 (0x01UL) /*!< Sel1                                                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL2 (0x02UL) /*!< Sel2                                                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL4 (0x04UL) /*!< Sel4                                                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL8 (0x08UL) /*!< Sel8                                                               */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL16 (0x10UL) /*!< Sel16                                                             */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL32 (0x20UL) /*!< Sel32                                                             */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL64 (0x40UL) /*!< Sel64                                                             */
  #define DIGITAL_WDTRIGGERVALUE2_VALUE2_SEL128 (0x80UL) /*!< Sel128                                                           */


/* DIGITAL_WDDATASTROBE: Strobe for registers WDTRIGGERVALUE */
  #define DIGITAL_WDDATASTROBE_ResetValue (0x00UL)   /*!< Reset value of WDDATASTROBE register.                                */

/* STROBE @Bit 0 : Strobe for registers WDTRIGGERVALUEx */
  #define DIGITAL_WDDATASTROBE_STROBE_Pos (0UL)      /*!< Position of STROBE field.                                            */
  #define DIGITAL_WDDATASTROBE_STROBE_Msk (0x1UL << DIGITAL_WDDATASTROBE_STROBE_Pos) /*!< Bit mask of STROBE field.            */
  #define DIGITAL_WDDATASTROBE_STROBE_Min (0x0UL)    /*!< Min enumerator value of STROBE field.                                */
  #define DIGITAL_WDDATASTROBE_STROBE_Max (0x1UL)    /*!< Max enumerator value of STROBE field.                                */
  #define DIGITAL_WDDATASTROBE_STROBE_NOEFFECT (0x0UL) /*!< No effect                                                          */
  #define DIGITAL_WDDATASTROBE_STROBE_STROBE (0x1UL) /*!< Strobe                                                               */


/* DIGITAL_WDPWRUPVALUE: Watchdog and wake-up timer enable. Use Strobe WDPWRUPSTROBE */
  #define DIGITAL_WDPWRUPVALUE_ResetValue (0x00UL)   /*!< Reset value of WDPWRUPVALUE register.                                */

/* OSC @Bit 0 : Oscillator enable for watchdog and wake-up timer */
  #define DIGITAL_WDPWRUPVALUE_OSC_Pos (0UL)         /*!< Position of OSC field.                                               */
  #define DIGITAL_WDPWRUPVALUE_OSC_Msk (0x1UL << DIGITAL_WDPWRUPVALUE_OSC_Pos) /*!< Bit mask of OSC field.                     */
  #define DIGITAL_WDPWRUPVALUE_OSC_Min (0x0UL)       /*!< Min enumerator value of OSC field.                                   */
  #define DIGITAL_WDPWRUPVALUE_OSC_Max (0x1UL)       /*!< Max enumerator value of OSC field.                                   */
  #define DIGITAL_WDPWRUPVALUE_OSC_DISABLE (0x0UL)   /*!< Disable                                                              */
  #define DIGITAL_WDPWRUPVALUE_OSC_ENABLE (0x1UL)    /*!< Enable                                                               */

/* COUNTER @Bit 1 : Counter enable for watchdog and wake-up timer */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_Pos (1UL)     /*!< Position of COUNTER field.                                           */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_Msk (0x1UL << DIGITAL_WDPWRUPVALUE_COUNTER_Pos) /*!< Bit mask of COUNTER field.         */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_Min (0x0UL)   /*!< Min enumerator value of COUNTER field.                               */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_Max (0x1UL)   /*!< Max enumerator value of COUNTER field.                               */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_DISABLE (0x0UL) /*!< Disable                                                            */
  #define DIGITAL_WDPWRUPVALUE_COUNTER_ENABLE (0x1UL) /*!< Enable                                                              */

/* LS @Bit 2 : Clock levelshifter enable for watchdog and wake-up timer */
  #define DIGITAL_WDPWRUPVALUE_LS_Pos (2UL)          /*!< Position of LS field.                                                */
  #define DIGITAL_WDPWRUPVALUE_LS_Msk (0x1UL << DIGITAL_WDPWRUPVALUE_LS_Pos) /*!< Bit mask of LS field.                        */
  #define DIGITAL_WDPWRUPVALUE_LS_Min (0x0UL)        /*!< Min enumerator value of LS field.                                    */
  #define DIGITAL_WDPWRUPVALUE_LS_Max (0x1UL)        /*!< Max enumerator value of LS field.                                    */
  #define DIGITAL_WDPWRUPVALUE_LS_DISABLE (0x0UL)    /*!< Disable                                                              */
  #define DIGITAL_WDPWRUPVALUE_LS_ENABLE (0x1UL)     /*!< Enable                                                               */


/* DIGITAL_WDPWRUPSTROBE: Strobe for register WDPWRUPVALUE */
  #define DIGITAL_WDPWRUPSTROBE_ResetValue (0x00UL)  /*!< Reset value of WDPWRUPSTROBE register.                               */

/* STROBE @Bit 0 : Strobe for register WDPWRUPVALUE */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_Pos (0UL)     /*!< Position of STROBE field.                                            */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_Msk (0x1UL << DIGITAL_WDPWRUPSTROBE_STROBE_Pos) /*!< Bit mask of STROBE field.          */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_Min (0x0UL)   /*!< Min enumerator value of STROBE field.                                */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_Max (0x1UL)   /*!< Max enumerator value of STROBE field.                                */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_NOEFFECT (0x0UL) /*!< No effect                                                         */
  #define DIGITAL_WDPWRUPSTROBE_STROBE_STROBE (0x1UL) /*!< Strobe                                                              */


/* DIGITAL_WDKICK: Watchdog kick */
  #define DIGITAL_WDKICK_ResetValue (0x00UL)         /*!< Reset value of WDKICK register.                                      */

/* KICK @Bit 0 : Watchdog counter reset */
  #define DIGITAL_WDKICK_KICK_Pos (0UL)              /*!< Position of KICK field.                                              */
  #define DIGITAL_WDKICK_KICK_Msk (0x1UL << DIGITAL_WDKICK_KICK_Pos) /*!< Bit mask of KICK field.                              */
  #define DIGITAL_WDKICK_KICK_Min (0x0UL)            /*!< Min enumerator value of KICK field.                                  */
  #define DIGITAL_WDKICK_KICK_Max (0x1UL)            /*!< Max enumerator value of KICK field.                                  */
  #define DIGITAL_WDKICK_KICK_NOEFFECT (0x0UL)       /*!< No effect                                                            */
  #define DIGITAL_WDKICK_KICK_KICK (0x1UL)           /*!< Reset counter                                                        */


/* DIGITAL_WDREQPOWERDOWN: Enter hibernate mode */
  #define DIGITAL_WDREQPOWERDOWN_ResetValue (0x00UL) /*!< Reset value of WDREQPOWERDOWN register.                              */

/* HARDPOWERDOWN @Bit 0 : Enter hibernate mode */
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Pos (0UL) /*!< Position of HARDPOWERDOWN field.                                 */
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Msk (0x1UL << DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Pos) /*!< Bit mask of
                                                                            HARDPOWERDOWN field.*/                                
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Min (0x0UL) /*!< Min enumerator value of HARDPOWERDOWN field.                   */
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_Max (0x1UL) /*!< Max enumerator value of HARDPOWERDOWN field.                   */
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_NOREQUEST (0x0UL) /*!< No effect                                                */
  #define DIGITAL_WDREQPOWERDOWN_HARDPOWERDOWN_REQUEST (0x1UL) /*!< Enter hibernate mode                                       */

/* RESERVED0 @Bit 1 : Reserved0 */
  #define DIGITAL_WDREQPOWERDOWN_RESERVED0_Pos (1UL) /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_WDREQPOWERDOWN_RESERVED0_Msk (0x1UL << DIGITAL_WDREQPOWERDOWN_RESERVED0_Pos) /*!< Bit mask of RESERVED0
                                                                            field.*/                                              


/* DIGITAL_GPIOOUTSET: GPIO output value SET */
  #define DIGITAL_GPIOOUTSET_ResetValue (0x00UL)     /*!< Reset value of GPIOOUTSET register.                                  */

/* GPIO0OUTSET @Bit 0 : Set GPIO0 */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_Pos (0UL)   /*!< Position of GPIO0OUTSET field.                                       */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_Msk (0x1UL << DIGITAL_GPIOOUTSET_GPIO0OUTSET_Pos) /*!< Bit mask of GPIO0OUTSET field. */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_Min (0x0UL) /*!< Min enumerator value of GPIO0OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_Max (0x1UL) /*!< Max enumerator value of GPIO0OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTSET_GPIO0OUTSET_SET (0x1UL) /*!< Set output high                                                      */

/* GPIO1OUTSET @Bit 1 : Set GPIO1 */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_Pos (1UL)   /*!< Position of GPIO1OUTSET field.                                       */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_Msk (0x1UL << DIGITAL_GPIOOUTSET_GPIO1OUTSET_Pos) /*!< Bit mask of GPIO1OUTSET field. */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_Min (0x0UL) /*!< Min enumerator value of GPIO1OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_Max (0x1UL) /*!< Max enumerator value of GPIO1OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTSET_GPIO1OUTSET_SET (0x1UL) /*!< Set output high                                                      */

/* GPIO2OUTSET @Bit 2 : Set GPIO2 */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_Pos (2UL)   /*!< Position of GPIO2OUTSET field.                                       */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_Msk (0x1UL << DIGITAL_GPIOOUTSET_GPIO2OUTSET_Pos) /*!< Bit mask of GPIO2OUTSET field. */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_Min (0x0UL) /*!< Min enumerator value of GPIO2OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_Max (0x1UL) /*!< Max enumerator value of GPIO2OUTSET field.                           */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTSET_GPIO2OUTSET_SET (0x1UL) /*!< Set output high                                                      */


/* DIGITAL_GPIOOUTCLR: GPIO output value CLEAR */
  #define DIGITAL_GPIOOUTCLR_ResetValue (0x00UL)     /*!< Reset value of GPIOOUTCLR register.                                  */

/* GPIO0OUTCLR @Bit 0 : Clear GPIO0 */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Pos (0UL)   /*!< Position of GPIO0OUTCLR field.                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Msk (0x1UL << DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Pos) /*!< Bit mask of GPIO0OUTCLR field. */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Min (0x0UL) /*!< Min enumerator value of GPIO0OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_Max (0x1UL) /*!< Max enumerator value of GPIO0OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO0OUTCLR_CLR (0x1UL) /*!< Set output low                                                       */

/* GPIO1OUTCLR @Bit 1 : Clear GPIO1 */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Pos (1UL)   /*!< Position of GPIO1OUTCLR field.                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Msk (0x1UL << DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Pos) /*!< Bit mask of GPIO1OUTCLR field. */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Min (0x0UL) /*!< Min enumerator value of GPIO1OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_Max (0x1UL) /*!< Max enumerator value of GPIO1OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO1OUTCLR_CLR (0x1UL) /*!< Set output low                                                       */

/* GPIO2OUTCLR @Bit 2 : Clear GPIO2 */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Pos (2UL)   /*!< Position of GPIO2OUTCLR field.                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Msk (0x1UL << DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Pos) /*!< Bit mask of GPIO2OUTCLR field. */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Min (0x0UL) /*!< Min enumerator value of GPIO2OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_Max (0x1UL) /*!< Max enumerator value of GPIO2OUTCLR field.                           */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_NOEFFECT (0x0UL) /*!< No effect                                                       */
  #define DIGITAL_GPIOOUTCLR_GPIO2OUTCLR_CLR (0x1UL) /*!< Set output low                                                       */


/* DIGITAL_GPIOIN: GPIO input value */
  #define DIGITAL_GPIOIN_ResetValue (0x00UL)         /*!< Reset value of GPIOIN register.                                      */

/* GPIO0IN @Bit 0 : GPIO0 input value */
  #define DIGITAL_GPIOIN_GPIO0IN_Pos (0UL)           /*!< Position of GPIO0IN field.                                           */
  #define DIGITAL_GPIOIN_GPIO0IN_Msk (0x1UL << DIGITAL_GPIOIN_GPIO0IN_Pos) /*!< Bit mask of GPIO0IN field.                     */
  #define DIGITAL_GPIOIN_GPIO0IN_Min (0x0UL)         /*!< Min enumerator value of GPIO0IN field.                               */
  #define DIGITAL_GPIOIN_GPIO0IN_Max (0x1UL)         /*!< Max enumerator value of GPIO0IN field.                               */
  #define DIGITAL_GPIOIN_GPIO0IN_LOW (0x0UL)         /*!< Input is low                                                         */
  #define DIGITAL_GPIOIN_GPIO0IN_HIGH (0x1UL)        /*!< Input is high                                                        */

/* GPIO1IN @Bit 1 : GPIO1 input value */
  #define DIGITAL_GPIOIN_GPIO1IN_Pos (1UL)           /*!< Position of GPIO1IN field.                                           */
  #define DIGITAL_GPIOIN_GPIO1IN_Msk (0x1UL << DIGITAL_GPIOIN_GPIO1IN_Pos) /*!< Bit mask of GPIO1IN field.                     */
  #define DIGITAL_GPIOIN_GPIO1IN_Min (0x0UL)         /*!< Min enumerator value of GPIO1IN field.                               */
  #define DIGITAL_GPIOIN_GPIO1IN_Max (0x1UL)         /*!< Max enumerator value of GPIO1IN field.                               */
  #define DIGITAL_GPIOIN_GPIO1IN_LOW (0x0UL)         /*!< Input is low                                                         */
  #define DIGITAL_GPIOIN_GPIO1IN_HIGH (0x1UL)        /*!< Input is high                                                        */

/* GPIO2IN @Bit 2 : GPIO2 input value */
  #define DIGITAL_GPIOIN_GPIO2IN_Pos (2UL)           /*!< Position of GPIO2IN field.                                           */
  #define DIGITAL_GPIOIN_GPIO2IN_Msk (0x1UL << DIGITAL_GPIOIN_GPIO2IN_Pos) /*!< Bit mask of GPIO2IN field.                     */
  #define DIGITAL_GPIOIN_GPIO2IN_Min (0x0UL)         /*!< Min enumerator value of GPIO2IN field.                               */
  #define DIGITAL_GPIOIN_GPIO2IN_Max (0x1UL)         /*!< Max enumerator value of GPIO2IN field.                               */
  #define DIGITAL_GPIOIN_GPIO2IN_LOW (0x0UL)         /*!< Input is low                                                         */
  #define DIGITAL_GPIOIN_GPIO2IN_HIGH (0x1UL)        /*!< Input is high                                                        */


/* DIGITAL_GPIO0CONF: GPIO0 configuration */
  #define DIGITAL_GPIO0CONF_ResetValue (0x00UL)      /*!< Reset value of GPIO0CONF register.                                   */

/* DIRECTION @Bit 0 : Direction */
  #define DIGITAL_GPIO0CONF_DIRECTION_Pos (0UL)      /*!< Position of DIRECTION field.                                         */
  #define DIGITAL_GPIO0CONF_DIRECTION_Msk (0x1UL << DIGITAL_GPIO0CONF_DIRECTION_Pos) /*!< Bit mask of DIRECTION field.         */
  #define DIGITAL_GPIO0CONF_DIRECTION_Min (0x0UL)    /*!< Min enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO0CONF_DIRECTION_Max (0x1UL)    /*!< Max enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO0CONF_DIRECTION_INPUT (0x0UL)  /*!< Input                                                                */
  #define DIGITAL_GPIO0CONF_DIRECTION_OUTPUT (0x1UL) /*!< Output                                                               */

/* INPUT @Bit 1 : Input buffer */
  #define DIGITAL_GPIO0CONF_INPUT_Pos (1UL)          /*!< Position of INPUT field.                                             */
  #define DIGITAL_GPIO0CONF_INPUT_Msk (0x1UL << DIGITAL_GPIO0CONF_INPUT_Pos) /*!< Bit mask of INPUT field.                     */
  #define DIGITAL_GPIO0CONF_INPUT_Min (0x0UL)        /*!< Min enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO0CONF_INPUT_Max (0x1UL)        /*!< Max enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO0CONF_INPUT_DISABLED (0x0UL)   /*!< Input disabled. Input value is 0.                                    */
  #define DIGITAL_GPIO0CONF_INPUT_ENABLED (0x1UL)    /*!< Input enabled                                                        */

/* PULLDOWN @Bit 2 : Pulldown */
  #define DIGITAL_GPIO0CONF_PULLDOWN_Pos (2UL)       /*!< Position of PULLDOWN field.                                          */
  #define DIGITAL_GPIO0CONF_PULLDOWN_Msk (0x1UL << DIGITAL_GPIO0CONF_PULLDOWN_Pos) /*!< Bit mask of PULLDOWN field.            */
  #define DIGITAL_GPIO0CONF_PULLDOWN_Min (0x0UL)     /*!< Min enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO0CONF_PULLDOWN_Max (0x1UL)     /*!< Max enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO0CONF_PULLDOWN_DISABLED (0x0UL) /*!< Disabled                                                            */
  #define DIGITAL_GPIO0CONF_PULLDOWN_ENABLED (0x1UL) /*!< Enabled                                                              */

/* DRIVE @Bit 5 : Drive strength */
  #define DIGITAL_GPIO0CONF_DRIVE_Pos (5UL)          /*!< Position of DRIVE field.                                             */
  #define DIGITAL_GPIO0CONF_DRIVE_Msk (0x1UL << DIGITAL_GPIO0CONF_DRIVE_Pos) /*!< Bit mask of DRIVE field.                     */
  #define DIGITAL_GPIO0CONF_DRIVE_Min (0x0UL)        /*!< Min enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO0CONF_DRIVE_Max (0x1UL)        /*!< Max enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO0CONF_DRIVE_NORMAL (0x0UL)     /*!< Normal drive strength                                                */
  #define DIGITAL_GPIO0CONF_DRIVE_HIGH (0x1UL)       /*!< High drive strength                                                  */

/* SENSE @Bit 6 : Input type */
  #define DIGITAL_GPIO0CONF_SENSE_Pos (6UL)          /*!< Position of SENSE field.                                             */
  #define DIGITAL_GPIO0CONF_SENSE_Msk (0x1UL << DIGITAL_GPIO0CONF_SENSE_Pos) /*!< Bit mask of SENSE field.                     */
  #define DIGITAL_GPIO0CONF_SENSE_Min (0x0UL)        /*!< Min enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO0CONF_SENSE_Max (0x1UL)        /*!< Max enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO0CONF_SENSE_SCHMITT (0x0UL)    /*!< Schmitt trigger input                                                */
  #define DIGITAL_GPIO0CONF_SENSE_CMOS (0x1UL)       /*!< CMOS input                                                           */

/* RESERVED0 @Bit 7 : Reserved0 */
  #define DIGITAL_GPIO0CONF_RESERVED0_Pos (7UL)      /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_GPIO0CONF_RESERVED0_Msk (0x1UL << DIGITAL_GPIO0CONF_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.         */


/* DIGITAL_GPIO1CONF: GPIO1 configuration */
  #define DIGITAL_GPIO1CONF_ResetValue (0x00UL)      /*!< Reset value of GPIO1CONF register.                                   */

/* DIRECTION @Bit 0 : Direction */
  #define DIGITAL_GPIO1CONF_DIRECTION_Pos (0UL)      /*!< Position of DIRECTION field.                                         */
  #define DIGITAL_GPIO1CONF_DIRECTION_Msk (0x1UL << DIGITAL_GPIO1CONF_DIRECTION_Pos) /*!< Bit mask of DIRECTION field.         */
  #define DIGITAL_GPIO1CONF_DIRECTION_Min (0x0UL)    /*!< Min enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO1CONF_DIRECTION_Max (0x1UL)    /*!< Max enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO1CONF_DIRECTION_INPUT (0x0UL)  /*!< Input                                                                */
  #define DIGITAL_GPIO1CONF_DIRECTION_OUTPUT (0x1UL) /*!< Output                                                               */

/* INPUT @Bit 1 : Input buffer */
  #define DIGITAL_GPIO1CONF_INPUT_Pos (1UL)          /*!< Position of INPUT field.                                             */
  #define DIGITAL_GPIO1CONF_INPUT_Msk (0x1UL << DIGITAL_GPIO1CONF_INPUT_Pos) /*!< Bit mask of INPUT field.                     */
  #define DIGITAL_GPIO1CONF_INPUT_Min (0x0UL)        /*!< Min enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO1CONF_INPUT_Max (0x1UL)        /*!< Max enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO1CONF_INPUT_DISABLED (0x0UL)   /*!< Input disabled. Input data is 0.                                     */
  #define DIGITAL_GPIO1CONF_INPUT_ENABLED (0x1UL)    /*!< Input enabled                                                        */

/* PULLDOWN @Bit 2 : Pulldown */
  #define DIGITAL_GPIO1CONF_PULLDOWN_Pos (2UL)       /*!< Position of PULLDOWN field.                                          */
  #define DIGITAL_GPIO1CONF_PULLDOWN_Msk (0x1UL << DIGITAL_GPIO1CONF_PULLDOWN_Pos) /*!< Bit mask of PULLDOWN field.            */
  #define DIGITAL_GPIO1CONF_PULLDOWN_Min (0x0UL)     /*!< Min enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO1CONF_PULLDOWN_Max (0x1UL)     /*!< Max enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO1CONF_PULLDOWN_DISABLED (0x0UL) /*!< Disabled                                                            */
  #define DIGITAL_GPIO1CONF_PULLDOWN_ENABLED (0x1UL) /*!< Enabled                                                              */

/* DRIVE @Bit 5 : Drive strength */
  #define DIGITAL_GPIO1CONF_DRIVE_Pos (5UL)          /*!< Position of DRIVE field.                                             */
  #define DIGITAL_GPIO1CONF_DRIVE_Msk (0x1UL << DIGITAL_GPIO1CONF_DRIVE_Pos) /*!< Bit mask of DRIVE field.                     */
  #define DIGITAL_GPIO1CONF_DRIVE_Min (0x0UL)        /*!< Min enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO1CONF_DRIVE_Max (0x1UL)        /*!< Max enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO1CONF_DRIVE_NORMAL (0x0UL)     /*!< Normal drive strength                                                */
  #define DIGITAL_GPIO1CONF_DRIVE_HIGH (0x1UL)       /*!< High drive strength                                                  */

/* SENSE @Bit 6 : Input type */
  #define DIGITAL_GPIO1CONF_SENSE_Pos (6UL)          /*!< Position of SENSE field.                                             */
  #define DIGITAL_GPIO1CONF_SENSE_Msk (0x1UL << DIGITAL_GPIO1CONF_SENSE_Pos) /*!< Bit mask of SENSE field.                     */
  #define DIGITAL_GPIO1CONF_SENSE_Min (0x0UL)        /*!< Min enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO1CONF_SENSE_Max (0x1UL)        /*!< Max enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO1CONF_SENSE_SCHMITT (0x0UL)    /*!< Schmitt trigger input                                                */
  #define DIGITAL_GPIO1CONF_SENSE_CMOS (0x1UL)       /*!< CMOS input                                                           */

/* RESERVED0 @Bit 7 : Reserved0 */
  #define DIGITAL_GPIO1CONF_RESERVED0_Pos (7UL)      /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_GPIO1CONF_RESERVED0_Msk (0x1UL << DIGITAL_GPIO1CONF_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.         */


/* DIGITAL_GPIO2CONF: GPIO2 configuration */
  #define DIGITAL_GPIO2CONF_ResetValue (0x00UL)      /*!< Reset value of GPIO2CONF register.                                   */

/* DIRECTION @Bit 0 : Direction */
  #define DIGITAL_GPIO2CONF_DIRECTION_Pos (0UL)      /*!< Position of DIRECTION field.                                         */
  #define DIGITAL_GPIO2CONF_DIRECTION_Msk (0x1UL << DIGITAL_GPIO2CONF_DIRECTION_Pos) /*!< Bit mask of DIRECTION field.         */
  #define DIGITAL_GPIO2CONF_DIRECTION_Min (0x0UL)    /*!< Min enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO2CONF_DIRECTION_Max (0x1UL)    /*!< Max enumerator value of DIRECTION field.                             */
  #define DIGITAL_GPIO2CONF_DIRECTION_INPUT (0x0UL)  /*!< Input                                                                */
  #define DIGITAL_GPIO2CONF_DIRECTION_OUTPUT (0x1UL) /*!< Output                                                               */

/* INPUT @Bit 1 : Input buffer */
  #define DIGITAL_GPIO2CONF_INPUT_Pos (1UL)          /*!< Position of INPUT field.                                             */
  #define DIGITAL_GPIO2CONF_INPUT_Msk (0x1UL << DIGITAL_GPIO2CONF_INPUT_Pos) /*!< Bit mask of INPUT field.                     */
  #define DIGITAL_GPIO2CONF_INPUT_Min (0x0UL)        /*!< Min enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO2CONF_INPUT_Max (0x1UL)        /*!< Max enumerator value of INPUT field.                                 */
  #define DIGITAL_GPIO2CONF_INPUT_DISABLED (0x0UL)   /*!< Input disabled. Input data is 0.                                     */
  #define DIGITAL_GPIO2CONF_INPUT_ENABLED (0x1UL)    /*!< Input enabled                                                        */

/* PULLDOWN @Bit 2 : Pulldown */
  #define DIGITAL_GPIO2CONF_PULLDOWN_Pos (2UL)       /*!< Position of PULLDOWN field.                                          */
  #define DIGITAL_GPIO2CONF_PULLDOWN_Msk (0x1UL << DIGITAL_GPIO2CONF_PULLDOWN_Pos) /*!< Bit mask of PULLDOWN field.            */
  #define DIGITAL_GPIO2CONF_PULLDOWN_Min (0x0UL)     /*!< Min enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO2CONF_PULLDOWN_Max (0x1UL)     /*!< Max enumerator value of PULLDOWN field.                              */
  #define DIGITAL_GPIO2CONF_PULLDOWN_DISABLED (0x0UL) /*!< Disabled                                                            */
  #define DIGITAL_GPIO2CONF_PULLDOWN_ENABLED (0x1UL) /*!< Enabled                                                              */

/* DRIVE @Bit 5 : Drive strength */
  #define DIGITAL_GPIO2CONF_DRIVE_Pos (5UL)          /*!< Position of DRIVE field.                                             */
  #define DIGITAL_GPIO2CONF_DRIVE_Msk (0x1UL << DIGITAL_GPIO2CONF_DRIVE_Pos) /*!< Bit mask of DRIVE field.                     */
  #define DIGITAL_GPIO2CONF_DRIVE_Min (0x0UL)        /*!< Min enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO2CONF_DRIVE_Max (0x1UL)        /*!< Max enumerator value of DRIVE field.                                 */
  #define DIGITAL_GPIO2CONF_DRIVE_NORMAL (0x0UL)     /*!< Normal drive strength                                                */
  #define DIGITAL_GPIO2CONF_DRIVE_HIGH (0x1UL)       /*!< High drive strength                                                  */

/* SENSE @Bit 6 : Input type */
  #define DIGITAL_GPIO2CONF_SENSE_Pos (6UL)          /*!< Position of SENSE field.                                             */
  #define DIGITAL_GPIO2CONF_SENSE_Msk (0x1UL << DIGITAL_GPIO2CONF_SENSE_Pos) /*!< Bit mask of SENSE field.                     */
  #define DIGITAL_GPIO2CONF_SENSE_Min (0x0UL)        /*!< Min enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO2CONF_SENSE_Max (0x1UL)        /*!< Max enumerator value of SENSE field.                                 */
  #define DIGITAL_GPIO2CONF_SENSE_SCHMITT (0x0UL)    /*!< Schmitt trigger input                                                */
  #define DIGITAL_GPIO2CONF_SENSE_CMOS (0x1UL)       /*!< CMOS input                                                           */

/* RESERVED0 @Bit 7 : Reserved0 */
  #define DIGITAL_GPIO2CONF_RESERVED0_Pos (7UL)      /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_GPIO2CONF_RESERVED0_Msk (0x1UL << DIGITAL_GPIO2CONF_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.         */


/* DIGITAL_LDO0CTRL: LDO0 current limiter, output high-impedance and pulldown control */
  #define DIGITAL_LDO0CTRL_ResetValue (0x10UL)       /*!< Reset value of LDO0CTRL register.                                    */

/* ILIMITVALUE @Bits 0..1 : Current limiter level */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_Pos (0UL)     /*!< Position of ILIMITVALUE field.                                       */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_Msk (0x3UL << DIGITAL_LDO0CTRL_ILIMITVALUE_Pos) /*!< Bit mask of ILIMITVALUE field.     */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_Min (0x0UL)   /*!< Min enumerator value of ILIMITVALUE field.                           */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_Max (0x3UL)   /*!< Max enumerator value of ILIMITVALUE field.                           */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_50MA (0x0UL)  /*!< 50 mA                                                                */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_100MA (0x1UL) /*!< 100 mA                                                               */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_150MA (0x2UL) /*!< 150 mA                                                               */
  #define DIGITAL_LDO0CTRL_ILIMITVALUE_UNLIMITED (0x3UL) /*!< Unlimited                                                        */

/* HIGHZ @Bit 2 : Output high impedance */
  #define DIGITAL_LDO0CTRL_HIGHZ_Pos (2UL)           /*!< Position of HIGHZ field.                                             */
  #define DIGITAL_LDO0CTRL_HIGHZ_Msk (0x1UL << DIGITAL_LDO0CTRL_HIGHZ_Pos) /*!< Bit mask of HIGHZ field.                       */
  #define DIGITAL_LDO0CTRL_HIGHZ_Min (0x0UL)         /*!< Min enumerator value of HIGHZ field.                                 */
  #define DIGITAL_LDO0CTRL_HIGHZ_Max (0x1UL)         /*!< Max enumerator value of HIGHZ field.                                 */
  #define DIGITAL_LDO0CTRL_HIGHZ_DISABLED (0x0UL)    /*!< Disabled                                                             */
  #define DIGITAL_LDO0CTRL_HIGHZ_ENABLED (0x1UL)     /*!< Enabled                                                              */

/* PULLDOWN @Bit 3 : Output pulldown */
  #define DIGITAL_LDO0CTRL_PULLDOWN_Pos (3UL)        /*!< Position of PULLDOWN field.                                          */
  #define DIGITAL_LDO0CTRL_PULLDOWN_Msk (0x1UL << DIGITAL_LDO0CTRL_PULLDOWN_Pos) /*!< Bit mask of PULLDOWN field.              */
  #define DIGITAL_LDO0CTRL_PULLDOWN_Min (0x0UL)      /*!< Min enumerator value of PULLDOWN field.                              */
  #define DIGITAL_LDO0CTRL_PULLDOWN_Max (0x1UL)      /*!< Max enumerator value of PULLDOWN field.                              */
  #define DIGITAL_LDO0CTRL_PULLDOWN_DISABLED (0x0UL) /*!< Disabled                                                             */
  #define DIGITAL_LDO0CTRL_PULLDOWN_ENABLED (0x1UL)  /*!< Enabled                                                              */

/* ENAILIMITAFTERSTART @Bit 4 : Current limiter after LDO start */
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_Pos (4UL) /*!< Position of ENAILIMITAFTERSTART field.                           */
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_Msk (0x1UL << DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_Pos) /*!< Bit mask of
                                                                            ENAILIMITAFTERSTART field.*/                          
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_Min (0x0UL) /*!< Min enumerator value of ENAILIMITAFTERSTART field.             */
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_Max (0x1UL) /*!< Max enumerator value of ENAILIMITAFTERSTART field.             */
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_OFF (0x0UL) /*!< Off                                                            */
  #define DIGITAL_LDO0CTRL_ENAILIMITAFTERSTART_ON (0x1UL) /*!< On                                                              */

/* ENAILIMITDURINGSTART @Bit 5 : Current limiter during LDO start */
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_Pos (5UL) /*!< Position of ENAILIMITDURINGSTART field.                         */
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_Msk (0x1UL << DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_Pos) /*!< Bit mask of
                                                                            ENAILIMITDURINGSTART field.*/                         
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_Min (0x0UL) /*!< Min enumerator value of ENAILIMITDURINGSTART field.           */
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_Max (0x1UL) /*!< Max enumerator value of ENAILIMITDURINGSTART field.           */
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_OFF (0x0UL) /*!< Off                                                           */
  #define DIGITAL_LDO0CTRL_ENAILIMITDURINGSTART_ON (0x1UL) /*!< On                                                             */


/* DIGITAL_LDO1CTRL: LDO1 current limiter, output high-impedance and pulldown control */
  #define DIGITAL_LDO1CTRL_ResetValue (0x10UL)       /*!< Reset value of LDO1CTRL register.                                    */

/* ILIMITVALUE @Bit 0 : Current limiter level */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_Pos (0UL)     /*!< Position of ILIMITVALUE field.                                       */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_Msk (0x1UL << DIGITAL_LDO1CTRL_ILIMITVALUE_Pos) /*!< Bit mask of ILIMITVALUE field.     */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_Min (0x0UL)   /*!< Min enumerator value of ILIMITVALUE field.                           */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_Max (0x1UL)   /*!< Max enumerator value of ILIMITVALUE field.                           */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_150MA (0x0UL) /*!< 150mA                                                                */
  #define DIGITAL_LDO1CTRL_ILIMITVALUE_UNLIMITED (0x1UL) /*!< Unlimited                                                        */

/* RESERVED0 @Bit 1 : Reserved0 */
  #define DIGITAL_LDO1CTRL_RESERVED0_Pos (1UL)       /*!< Position of RESERVED0 field.                                         */
  #define DIGITAL_LDO1CTRL_RESERVED0_Msk (0x1UL << DIGITAL_LDO1CTRL_RESERVED0_Pos) /*!< Bit mask of RESERVED0 field.           */

/* HIGHZ @Bit 2 : Output high impedance */
  #define DIGITAL_LDO1CTRL_HIGHZ_Pos (2UL)           /*!< Position of HIGHZ field.                                             */
  #define DIGITAL_LDO1CTRL_HIGHZ_Msk (0x1UL << DIGITAL_LDO1CTRL_HIGHZ_Pos) /*!< Bit mask of HIGHZ field.                       */
  #define DIGITAL_LDO1CTRL_HIGHZ_Min (0x0UL)         /*!< Min enumerator value of HIGHZ field.                                 */
  #define DIGITAL_LDO1CTRL_HIGHZ_Max (0x1UL)         /*!< Max enumerator value of HIGHZ field.                                 */
  #define DIGITAL_LDO1CTRL_HIGHZ_DISABLED (0x0UL)    /*!< Disabled                                                             */
  #define DIGITAL_LDO1CTRL_HIGHZ_ENABLED (0x1UL)     /*!< Enabled                                                              */

/* PULLDOWN @Bit 3 : Output pulldown */
  #define DIGITAL_LDO1CTRL_PULLDOWN_Pos (3UL)        /*!< Position of PULLDOWN field.                                          */
  #define DIGITAL_LDO1CTRL_PULLDOWN_Msk (0x1UL << DIGITAL_LDO1CTRL_PULLDOWN_Pos) /*!< Bit mask of PULLDOWN field.              */
  #define DIGITAL_LDO1CTRL_PULLDOWN_Min (0x0UL)      /*!< Min enumerator value of PULLDOWN field.                              */
  #define DIGITAL_LDO1CTRL_PULLDOWN_Max (0x1UL)      /*!< Max enumerator value of PULLDOWN field.                              */
  #define DIGITAL_LDO1CTRL_PULLDOWN_DISABLED (0x0UL) /*!< Disabled                                                             */
  #define DIGITAL_LDO1CTRL_PULLDOWN_ENABLED (0x1UL)  /*!< Enabled                                                              */

/* ENAILIMITAFTERSTART @Bit 4 : Current limiter after LDO start */
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_Pos (4UL) /*!< Position of ENAILIMITAFTERSTART field.                           */
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_Msk (0x1UL << DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_Pos) /*!< Bit mask of
                                                                            ENAILIMITAFTERSTART field.*/                          
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_Min (0x0UL) /*!< Min enumerator value of ENAILIMITAFTERSTART field.             */
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_Max (0x1UL) /*!< Max enumerator value of ENAILIMITAFTERSTART field.             */
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_OFF (0x0UL) /*!< Off                                                            */
  #define DIGITAL_LDO1CTRL_ENAILIMITAFTERSTART_ON (0x1UL) /*!< On                                                              */

/* ENAILIMITDURINGSTART @Bit 5 : Current limiter during LDO start */
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_Pos (5UL) /*!< Position of ENAILIMITDURINGSTART field.                         */
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_Msk (0x1UL << DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_Pos) /*!< Bit mask of
                                                                            ENAILIMITDURINGSTART field.*/                         
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_Min (0x0UL) /*!< Min enumerator value of ENAILIMITDURINGSTART field.           */
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_Max (0x1UL) /*!< Max enumerator value of ENAILIMITDURINGSTART field.           */
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_OFF (0x0UL) /*!< Off                                                           */
  #define DIGITAL_LDO1CTRL_ENAILIMITDURINGSTART_ON (0x1UL) /*!< On                                                             */

/* RESERVED1 @Bit 6 : Reserved1 */
  #define DIGITAL_LDO1CTRL_RESERVED1_Pos (6UL)       /*!< Position of RESERVED1 field.                                         */
  #define DIGITAL_LDO1CTRL_RESERVED1_Msk (0x1UL << DIGITAL_LDO1CTRL_RESERVED1_Pos) /*!< Bit mask of RESERVED1 field.           */


/* DIGITAL_OVERRIDEPWRUPBUCK: Override for disabling BUCK1 or/and BUCK2 regulators */
  #define DIGITAL_OVERRIDEPWRUPBUCK_ResetValue (0x00UL) /*!< Reset value of OVERRIDEPWRUPBUCK register.                        */

/* RESERVED0 @Bit 0 : Reserved0 */
  #define DIGITAL_OVERRIDEPWRUPBUCK_RESERVED0_Pos (0UL) /*!< Position of RESERVED0 field.                                      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_RESERVED0_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_RESERVED0_Pos) /*!< Bit mask of RESERVED0
                                                                            field.*/                                              

/* BUCK1PWRUPOVERRIDE @Bit 1 : BUCK1 power up override */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Pos (1UL) /*!< Position of BUCK1PWRUPOVERRIDE field.                    */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Pos) /*!< Bit
                                                                            mask of BUCK1PWRUPOVERRIDE field.*/                   
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Min (0x0UL) /*!< Min enumerator value of BUCK1PWRUPOVERRIDE field.      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_Max (0x1UL) /*!< Max enumerator value of BUCK1PWRUPOVERRIDE field.      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_DISABLED (0x0UL) /*!< Disabled                                          */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDE_ENABLED (0x1UL) /*!< Enabled                                            */

/* BUCK2PWRUPOVERRIDE @Bit 2 : BUCK2 power up override */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Pos (2UL) /*!< Position of BUCK2PWRUPOVERRIDE field.                    */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Pos) /*!< Bit
                                                                            mask of BUCK2PWRUPOVERRIDE field.*/                   
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Min (0x0UL) /*!< Min enumerator value of BUCK2PWRUPOVERRIDE field.      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_Max (0x1UL) /*!< Max enumerator value of BUCK2PWRUPOVERRIDE field.      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_DISABLED (0x0UL) /*!< Disabled                                          */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDE_ENABLED (0x1UL) /*!< Enabled                                            */

/* SPARE0 @Bit 3 : Spare0 */
  #define DIGITAL_OVERRIDEPWRUPBUCK_SPARE0_Pos (3UL) /*!< Position of SPARE0 field.                                            */
  #define DIGITAL_OVERRIDEPWRUPBUCK_SPARE0_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_SPARE0_Pos) /*!< Bit mask of SPARE0 field.  */

/* RESERVED1 @Bit 4 : Reserved1 */
  #define DIGITAL_OVERRIDEPWRUPBUCK_RESERVED1_Pos (4UL) /*!< Position of RESERVED1 field.                                      */
  #define DIGITAL_OVERRIDEPWRUPBUCK_RESERVED1_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_RESERVED1_Pos) /*!< Bit mask of RESERVED1
                                                                            field.*/                                              

/* BUCK1PWRUPOVERRIDEVALUE @Bit 5 : BUCK1 power up override setting */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Pos (5UL) /*!< Position of BUCK1PWRUPOVERRIDEVALUE field.          */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Pos)
                                                                            /*!< Bit mask of BUCK1PWRUPOVERRIDEVALUE field.*/     
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Min (0x0UL) /*!< Min enumerator value of BUCK1PWRUPOVERRIDEVALUE
                                                                            field.*/                                              
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_Max (0x1UL) /*!< Max enumerator value of BUCK1PWRUPOVERRIDEVALUE
                                                                            field.*/                                              
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_DISABLED (0x0UL) /*!< Disable BUCK1                                */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK1PWRUPOVERRIDEVALUE_NOEFFECT (0x1UL) /*!< No effect                                    */

/* BUCK2PWRUPOVERRIDEVALUE @Bit 6 : BUCK2 power up override setting */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Pos (6UL) /*!< Position of BUCK2PWRUPOVERRIDEVALUE field.          */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Pos)
                                                                            /*!< Bit mask of BUCK2PWRUPOVERRIDEVALUE field.*/     
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Min (0x0UL) /*!< Min enumerator value of BUCK2PWRUPOVERRIDEVALUE
                                                                            field.*/                                              
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_Max (0x1UL) /*!< Max enumerator value of BUCK2PWRUPOVERRIDEVALUE
                                                                            field.*/                                              
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_DISABLED (0x0UL) /*!< Disable BUCK2                                */
  #define DIGITAL_OVERRIDEPWRUPBUCK_BUCK2PWRUPOVERRIDEVALUE_NOEFFECT (0x1UL) /*!< No effect                                    */

/* SPARE1 @Bit 7 : Spare1 */
  #define DIGITAL_OVERRIDEPWRUPBUCK_SPARE1_Pos (7UL) /*!< Position of SPARE1 field.                                            */
  #define DIGITAL_OVERRIDEPWRUPBUCK_SPARE1_Msk (0x1UL << DIGITAL_OVERRIDEPWRUPBUCK_SPARE1_Pos) /*!< Bit mask of SPARE1 field.  */



/* =========================================================================================================================== */
/* ================                                  Peripheral Address Map                                  ================ */
/* =========================================================================================================================== */

#define NRF_DIGITAL_BASE                  0x00000000UL

/* =========================================================================================================================== */
/* ================                                  Peripheral Declaration                                  ================ */
/* =========================================================================================================================== */

#define NRF_DIGITAL                       ((NRF_DIGITAL_Type*)                  NRF_DIGITAL_BASE)

/* ========================================== End of section using anonymous unions ========================================== */

#if defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#endif


#ifdef __cplusplus
}
#endif
#endif /* NPM6001_H */

