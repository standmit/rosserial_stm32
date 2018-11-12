/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Kenta Yonekura (a.k.a. yoneken)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote prducts derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROS_STM32_HARDWARE_H_
#define ROS_STM32_HARDWARE_H_

#include "STM32FXXX.h"

extern UART_HandleTypeDef huart1;

class STM32Hardware {
  protected:
    UART_HandleTypeDef *huart;
  
    uint8_t tick_period_ms;
    uint32_t systick_period_ns;

  public:
    STM32Hardware():
      huart(&huart1)
    { }
  
    void init(UART_HandleTypeDef* const uart_handle) {
        huart = uart_handle;

        tick_period_ms = 1000 / HAL_GetTickFreq(); // 1 / HAL_GetTickFreq() * 1000
        if (SysTick->CTRL && SYSTICK_CLKSOURCE_HCLK) {
            systick_period_ns = 1000000000 / HAL_RCC_GetHCLKFreq(); // 1 / ( HAL_RCC_GetHCLKFreq() / 1 ) * 1000000000
        } else {
            systick_period_ns = 8000000000 / HAL_RCC_GetHCLKFreq(); // 1 / ( HAL_RCC_GetHCLKFreq() / 8 ) * 1000000000
        }
    }

  
    void init()
    {
        this->init(&huart1);
    }

    int read() {
      uint8_t rxByte;
    	if(HAL_UART_Receive(huart, &rxByte, 1, 2) == HAL_OK)
    		return rxByte;
    	return -1;
    }

    void write(uint8_t* data, int length) {
      HAL_UART_Transmit(huart, data, length, 10);
    }

    unsigned long time() {
      return HAL_GetTick();
    }
  
    uint64_t time_ns(uint32_t ms, uint32_t ticks) {
        return (uint64_t)ms * 1000000 + ticks * systick_period_ns;
    }
    
    uint64_t time_ns() {
        return time_ns(this->time(), SysTick->VAL);
    }

  protected:
};

#endif

