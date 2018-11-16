# rosserial_stm32

## Note
This is a part of [rosserial](https://github.com/ros-drivers/rosserial) repository to communicate with ROS system through a USART for STM32 embedded system.

## Limitation
Currently, this code is focused on STM32F1xx & STM32F3xx series and it uses the [STM32CubeMX HAL](http://www.st.com/en/development-tools/stm32cubemx.html).  

## Generate code
$ rosrun rosserial_stm32 make_libraries.py F3 <output_path>  
