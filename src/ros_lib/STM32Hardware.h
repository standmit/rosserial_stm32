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
#include "RingBuffer.h"

extern UART_HandleTypeDef huart1;

class STM32Hardware {
	public:
		static STM32Hardware* instance;
  protected:
    UART_HandleTypeDef *huart;
  
    uint8_t tick_period_ms;
    uint32_t systick_period_ns;

    bool use_dma;
    void (*uart_tx_ext_callback)(UART_HandleTypeDef*);
    void (*uart_rx_ext_callback)(UART_HandleTypeDef*);
    RingBuffer<512> buffer_tx;
    RingBuffer<512> buffer_rx;
    volatile bool uart_tx_busy;
    volatile bool uart_rx_busy;
    RB_SizeType count_to_receive;

    RB_SizeType GetDMAReceivedCount() {
    	RB_SizeType count = count_to_receive - huart->hdmarx->Instance->CNDTR;
    	count_to_receive -= count;
    	buffer_rx.OccupyFreeSpace(count);
    	return count;
    }

  public:
    struct Argument {
    	enum {
    		arg_huart,
			arg_use_dma,
			arg_uart_tx_callback,
			arg_uart_rx_callback
    	} type;

    	union {
    		UART_HandleTypeDef* huart;
    		bool use_dma;
    		void (*uart_callback)(UART_HandleTypeDef*);
    	} value;

    	Argument(UART_HandleTypeDef* const huart_): type(arg_huart) { value.huart = huart_; };
    	Argument(const bool use_dma_): type(arg_use_dma) { value.use_dma = use_dma_; };
    	Argument(void (*const uart_callback_)(UART_HandleTypeDef*), const bool tx) { type = tx ? arg_uart_tx_callback : arg_uart_rx_callback; value.uart_callback = uart_callback_; };
    };

    typedef uint8_t arg_count_type;


    STM32Hardware():
      huart(NULL),
	  use_dma(false),
	  uart_tx_ext_callback(NULL),
	  uart_rx_ext_callback(NULL),
	  buffer_tx(),
	  buffer_rx(),
	  uart_tx_busy(false),
	  uart_rx_busy(false),
	  count_to_receive(0)
    { }
  
    void init(char* const portName) {
    	instance = this;
    	const arg_count_type argc = *((arg_count_type*)portName);
    	Argument* argv = (Argument*)(portName + 1);
    	for (arg_count_type i = 0; i < argc; i++) {
    		switch (argv[i].type) {
    			case Argument::arg_huart:
    				huart = argv[i].value.huart;
    				break;
    			case Argument::arg_use_dma:
    				use_dma = argv[i].value.use_dma;
    				break;
    			case Argument::arg_uart_tx_callback:
    				uart_tx_ext_callback = argv[i].value.uart_callback;
    				break;
    			case Argument::arg_uart_rx_callback:
    				uart_rx_ext_callback = argv[i].value.uart_callback;
    				break;

    		}
    	}

        tick_period_ms = 1000 / HAL_GetTickFreq(); // 1 / HAL_GetTickFreq() * 1000
        if (SysTick->CTRL && SYSTICK_CLKSOURCE_HCLK) {
            systick_period_ns = 1000000000 / HAL_RCC_GetHCLKFreq(); // 1 / ( HAL_RCC_GetHCLKFreq() / 1 ) * 1000000000
        } else {
            systick_period_ns = 8000000000 / HAL_RCC_GetHCLKFreq(); // 1 / ( HAL_RCC_GetHCLKFreq() / 8 ) * 1000000000
        }

        receive_part();
    }

  
    void init()
    {
        huart = NULL;
        use_dma = false;
        uart_tx_ext_callback = NULL;
        uart_rx_ext_callback = NULL;
        uart_tx_busy = false;
        uart_rx_busy = false;
    }

    int read() {
    	uint8_t rxByte;
    	if (use_dma) {
    		GetDMAReceivedCount();
			if (buffer_rx.Read(rxByte)) {
				return rxByte;
			} else {
				return -1;
			}
    	} else {
    		if(HAL_UART_Receive(huart, &rxByte, 1, 2) == HAL_OK) {
				return rxByte;
    		} else {
    			return -1;
    		}
    	}
    }

    void write(uint8_t* data, int length, const bool wait = false) {
    	if (use_dma) {
    		if (wait) {
    			while (uart_tx_busy) { transmit_part(); }
    			HAL_UART_Transmit(huart, data, length, 10);
    		} else {
				while (!buffer_tx.Write(data, length)) { transmit_part(); }
			}

    		transmit_part();
    	} else {
    		HAL_UART_Transmit(huart, data, length, 10);
    	}
    }

    void transmit_part() {
    	if (!uart_tx_busy) {
			RB_DataType* buf;
			RB_SizeType count = buffer_tx.GetLastPart(buf);
			if (count)
				uart_tx_busy = (HAL_UART_Transmit_DMA(huart, buf, count) == HAL_OK);
    	}
    }

    void receive_part() {
    	if (!uart_rx_busy) {
    		RB_DataType* buf;
    		RB_SizeType count = buffer_rx.GetFreeSpace(buf);
    		if (count) {
    			count_to_receive = count;
    			uart_rx_busy = (HAL_UART_Receive_DMA(huart, buf, count) == HAL_OK);
    		}
    	}
    }

    void uart_tx_callback(UART_HandleTypeDef* const uart_handle) {
		if ((huart != NULL) && (uart_handle == huart) && use_dma && uart_tx_busy) {
			buffer_tx.DropLastPart();
			uart_tx_busy = false;
			transmit_part();
		}

		if (uart_tx_ext_callback != NULL)
			(*uart_tx_ext_callback)(uart_handle);
    }

    void uart_rx_callback(UART_HandleTypeDef* const uart_handle) {
    	if ((huart != NULL) && (uart_handle == huart) && use_dma && uart_rx_busy) {
    		GetDMAReceivedCount();
    		uart_rx_busy = false;
    		receive_part();
    	}

    	if (uart_rx_ext_callback != NULL)
    		(*uart_rx_ext_callback)(uart_handle);
    }

    void uart_error_callback(UART_HandleTypeDef* const uart_handle) {
    	uart_tx_callback(uart_handle);
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
};

STM32Hardware* STM32Hardware::instance = NULL;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (STM32Hardware::instance != NULL)
		STM32Hardware::instance->uart_tx_callback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (STM32Hardware::instance != NULL)
		STM32Hardware::instance->uart_rx_callback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (STM32Hardware::instance != NULL)
		STM32Hardware::instance->uart_error_callback(huart);
}

#endif

