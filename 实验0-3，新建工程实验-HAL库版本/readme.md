# 目录结构

/**
*******************************************************************************
 * 实验简介
 * 实验名称：template 实验
 * 实验平台：正点原子 探索者 F407开发板
 * 实验目的：学习如何新建HAL库版本MDK工程
*******************************************************************************
 * 硬件资源及引脚分配
*******************************************************************************
 * 实验现象
 * 1 工程编译0错误，0警告
*******************************************************************************
 * 注意事项
 * 无
*******************************************************************************
 **/

`C++
├─Drivers
│  ├─BSP
│  │  └─LED
│  │          led.c
│  │          led.h 
│  ├─CMSIS
│  │  ├─Device
│  │  │  └─ST
│  │  │      └─STM32F4xx
│  │  │          ├─Include
│  │  │          │      stm32f407xx.h
│  │  │          │      stm32f4xx.h
│  │  │          │      system_stm32f4xx.h
│  │  │          └─Source
│  │  │              └─Templates
│  │  │                  │  system_stm32f4xx.c
│  │  │                  └─arm
│  │  │                          startup_stm32f407xx.s                 
│  │  └─Include
│  │          cmsis_armcc.h
│  │          cmsis_armclang.h
│  │          cmsis_compiler.h
│  │          cmsis_version.h
│  │          core_cm4.h
│  │          mpu_armv7.h
│  ├─STM32F4xx_HAL_Driver
│  │  ├─Inc
│  │  │  │  stm32f4xx_hal.h
│  │  │  │  stm32f4xx_hal_adc.h
│  │  │  │  stm32f4xx_hal_adc_ex.h
│  │  │  │  stm32f4xx_hal_can.h
│  │  │  │  stm32f4xx_hal_cec.h
│  │  │  │  stm32f4xx_hal_conf_template.h
│  │  │  │  stm32f4xx_hal_cortex.h
│  │  │  │  stm32f4xx_hal_crc.h
│  │  │  │  stm32f4xx_hal_cryp.h
│  │  │  │  stm32f4xx_hal_cryp_ex.h
│  │  │  │  stm32f4xx_hal_dac.h
│  │  │  │  stm32f4xx_hal_dac_ex.h
│  │  │  │  stm32f4xx_hal_dcmi.h
│  │  │  │  stm32f4xx_hal_dcmi_ex.h
│  │  │  │  stm32f4xx_hal_def.h
│  │  │  │  stm32f4xx_hal_dfsdm.h
│  │  │  │  stm32f4xx_hal_dma.h
│  │  │  │  stm32f4xx_hal_dma2d.h
│  │  │  │  stm32f4xx_hal_dma_ex.h
│  │  │  │  stm32f4xx_hal_dsi.h
│  │  │  │  stm32f4xx_hal_eth.h
│  │  │  │  stm32f4xx_hal_exti.h
│  │  │  │  stm32f4xx_hal_flash.h
│  │  │  │  stm32f4xx_hal_flash_ex.h
│  │  │  │  stm32f4xx_hal_flash_ramfunc.h
│  │  │  │  stm32f4xx_hal_fmpi2c.h
│  │  │  │  stm32f4xx_hal_fmpi2c_ex.h
│  │  │  │  stm32f4xx_hal_fmpsmbus.h
│  │  │  │  stm32f4xx_hal_fmpsmbus_ex.h
│  │  │  │  stm32f4xx_hal_gpio.h
│  │  │  │  stm32f4xx_hal_gpio_ex.h
│  │  │  │  stm32f4xx_hal_hash.h
│  │  │  │  stm32f4xx_hal_hash_ex.h
│  │  │  │  stm32f4xx_hal_hcd.h
│  │  │  │  stm32f4xx_hal_i2c.h
│  │  │  │  stm32f4xx_hal_i2c_ex.h
│  │  │  │  stm32f4xx_hal_i2s.h
│  │  │  │  stm32f4xx_hal_i2s_ex.h
│  │  │  │  stm32f4xx_hal_irda.h
│  │  │  │  stm32f4xx_hal_iwdg.h
│  │  │  │  stm32f4xx_hal_lptim.h
│  │  │  │  stm32f4xx_hal_ltdc.h
│  │  │  │  stm32f4xx_hal_ltdc_ex.h
│  │  │  │  stm32f4xx_hal_mmc.h
│  │  │  │  stm32f4xx_hal_nand.h
│  │  │  │  stm32f4xx_hal_nor.h
│  │  │  │  stm32f4xx_hal_pccard.h
│  │  │  │  stm32f4xx_hal_pcd.h
│  │  │  │  stm32f4xx_hal_pcd_ex.h
│  │  │  │  stm32f4xx_hal_pwr.h
│  │  │  │  stm32f4xx_hal_pwr_ex.h
│  │  │  │  stm32f4xx_hal_qspi.h
│  │  │  │  stm32f4xx_hal_rcc.h
│  │  │  │  stm32f4xx_hal_rcc_ex.h
│  │  │  │  stm32f4xx_hal_rng.h
│  │  │  │  stm32f4xx_hal_rtc.h
│  │  │  │  stm32f4xx_hal_rtc_ex.h
│  │  │  │  stm32f4xx_hal_sai.h
│  │  │  │  stm32f4xx_hal_sai_ex.h
│  │  │  │  stm32f4xx_hal_sd.h
│  │  │  │  stm32f4xx_hal_sdram.h
│  │  │  │  stm32f4xx_hal_smartcard.h
│  │  │  │  stm32f4xx_hal_smbus.h
│  │  │  │  stm32f4xx_hal_spdifrx.h
│  │  │  │  stm32f4xx_hal_spi.h
│  │  │  │  stm32f4xx_hal_sram.h
│  │  │  │  stm32f4xx_hal_tim.h
│  │  │  │  stm32f4xx_hal_tim_ex.h
│  │  │  │  stm32f4xx_hal_uart.h
│  │  │  │  stm32f4xx_hal_usart.h
│  │  │  │  stm32f4xx_hal_wwdg.h
│  │  │  │  stm32f4xx_ll_adc.h
│  │  │  │  stm32f4xx_ll_bus.h
│  │  │  │  stm32f4xx_ll_cortex.h
│  │  │  │  stm32f4xx_ll_crc.h
│  │  │  │  stm32f4xx_ll_dac.h
│  │  │  │  stm32f4xx_ll_dma.h
│  │  │  │  stm32f4xx_ll_dma2d.h
│  │  │  │  stm32f4xx_ll_exti.h
│  │  │  │  stm32f4xx_ll_fmc.h
│  │  │  │  stm32f4xx_ll_fmpi2c.h
│  │  │  │  stm32f4xx_ll_fsmc.h
│  │  │  │  stm32f4xx_ll_gpio.h
│  │  │  │  stm32f4xx_ll_i2c.h
│  │  │  │  stm32f4xx_ll_iwdg.h
│  │  │  │  stm32f4xx_ll_lptim.h
│  │  │  │  stm32f4xx_ll_pwr.h
│  │  │  │  stm32f4xx_ll_rcc.h
│  │  │  │  stm32f4xx_ll_rng.h
│  │  │  │  stm32f4xx_ll_rtc.h
│  │  │  │  stm32f4xx_ll_sdmmc.h
│  │  │  │  stm32f4xx_ll_spi.h
│  │  │  │  stm32f4xx_ll_system.h
│  │  │  │  stm32f4xx_ll_tim.h
│  │  │  │  stm32f4xx_ll_usart.h
│  │  │  │  stm32f4xx_ll_usb.h
│  │  │  │  stm32f4xx_ll_utils.h
│  │  │  │  stm32f4xx_ll_wwdg.h
│  │  │  │  stm32_assert_template.h
│  │  │  │  
│  │  │  └─Legacy
│  │  │          stm32f4xx_hal_can_legacy.h
│  │  │          stm32_hal_legacy.h 
│  │  └─Src
│  │      │  stm32f4xx_hal.c
│  │      │  stm32f4xx_hal_adc.c
│  │      │  stm32f4xx_hal_adc_ex.c
│  │      │  stm32f4xx_hal_can.c
│  │      │  stm32f4xx_hal_cec.c
│  │      │  stm32f4xx_hal_cortex.c
│  │      │  stm32f4xx_hal_crc.c
│  │      │  stm32f4xx_hal_cryp.c
│  │      │  stm32f4xx_hal_cryp_ex.c
│  │      │  stm32f4xx_hal_dac.c
│  │      │  stm32f4xx_hal_dac_ex.c
│  │      │  stm32f4xx_hal_dcmi.c
│  │      │  stm32f4xx_hal_dcmi_ex.c
│  │      │  stm32f4xx_hal_dfsdm.c
│  │      │  stm32f4xx_hal_dma.c
│  │      │  stm32f4xx_hal_dma2d.c
│  │      │  stm32f4xx_hal_dma_ex.c
│  │      │  stm32f4xx_hal_dsi.c
│  │      │  stm32f4xx_hal_eth.c
│  │      │  stm32f4xx_hal_exti.c
│  │      │  stm32f4xx_hal_flash.c
│  │      │  stm32f4xx_hal_flash_ex.c
│  │      │  stm32f4xx_hal_flash_ramfunc.c
│  │      │  stm32f4xx_hal_fmpi2c.c
│  │      │  stm32f4xx_hal_fmpi2c_ex.c
│  │      │  stm32f4xx_hal_fmpsmbus.c
│  │      │  stm32f4xx_hal_fmpsmbus_ex.c
│  │      │  stm32f4xx_hal_gpio.c
│  │      │  stm32f4xx_hal_hash.c
│  │      │  stm32f4xx_hal_hash_ex.c
│  │      │  stm32f4xx_hal_hcd.c
│  │      │  stm32f4xx_hal_i2c.c
│  │      │  stm32f4xx_hal_i2c_ex.c
│  │      │  stm32f4xx_hal_i2s.c
│  │      │  stm32f4xx_hal_i2s_ex.c
│  │      │  stm32f4xx_hal_irda.c
│  │      │  stm32f4xx_hal_iwdg.c
│  │      │  stm32f4xx_hal_lptim.c
│  │      │  stm32f4xx_hal_ltdc.c
│  │      │  stm32f4xx_hal_ltdc_ex.c
│  │      │  stm32f4xx_hal_mmc.c
│  │      │  stm32f4xx_hal_msp_template.c
│  │      │  stm32f4xx_hal_nand.c
│  │      │  stm32f4xx_hal_nor.c
│  │      │  stm32f4xx_hal_pccard.c
│  │      │  stm32f4xx_hal_pcd.c
│  │      │  stm32f4xx_hal_pcd_ex.c
│  │      │  stm32f4xx_hal_pwr.c
│  │      │  stm32f4xx_hal_pwr_ex.c
│  │      │  stm32f4xx_hal_qspi.c
│  │      │  stm32f4xx_hal_rcc.c
│  │      │  stm32f4xx_hal_rcc_ex.c
│  │      │  stm32f4xx_hal_rng.c
│  │      │  stm32f4xx_hal_rtc.c
│  │      │  stm32f4xx_hal_rtc_ex.c
│  │      │  stm32f4xx_hal_sai.c
│  │      │  stm32f4xx_hal_sai_ex.c
│  │      │  stm32f4xx_hal_sd.c
│  │      │  stm32f4xx_hal_sdram.c
│  │      │  stm32f4xx_hal_smartcard.c
│  │      │  stm32f4xx_hal_smbus.c
│  │      │  stm32f4xx_hal_spdifrx.c
│  │      │  stm32f4xx_hal_spi.c
│  │      │  stm32f4xx_hal_sram.c
│  │      │  stm32f4xx_hal_tim.c
│  │      │  stm32f4xx_hal_timebase_rtc_alarm_template.c
│  │      │  stm32f4xx_hal_timebase_rtc_wakeup_template.c
│  │      │  stm32f4xx_hal_timebase_tim_template.c
│  │      │  stm32f4xx_hal_tim_ex.c
│  │      │  stm32f4xx_hal_uart.c
│  │      │  stm32f4xx_hal_usart.c
│  │      │  stm32f4xx_hal_wwdg.c
│  │      │  stm32f4xx_ll_adc.c
│  │      │  stm32f4xx_ll_crc.c
│  │      │  stm32f4xx_ll_dac.c
│  │      │  stm32f4xx_ll_dma.c
│  │      │  stm32f4xx_ll_dma2d.c
│  │      │  stm32f4xx_ll_exti.c
│  │      │  stm32f4xx_ll_fmc.c
│  │      │  stm32f4xx_ll_fmpi2c.c
│  │      │  stm32f4xx_ll_fsmc.c
│  │      │  stm32f4xx_ll_gpio.c
│  │      │  stm32f4xx_ll_i2c.c
│  │      │  stm32f4xx_ll_lptim.c
│  │      │  stm32f4xx_ll_pwr.c
│  │      │  stm32f4xx_ll_rcc.c
│  │      │  stm32f4xx_ll_rng.c
│  │      │  stm32f4xx_ll_rtc.c
│  │      │  stm32f4xx_ll_sdmmc.c
│  │      │  stm32f4xx_ll_spi.c
│  │      │  stm32f4xx_ll_tim.c
│  │      │  stm32f4xx_ll_usart.c
│  │      │  stm32f4xx_ll_usb.c
│  │      │  stm32f4xx_ll_utils.c
│  │      └─Legacy
│  │              stm32f4xx_hal_can.c
│  └─SYSTEM
│      ├─delay
│      │      delay.c
│      │      delay.h
│      ├─sys
│      │      sys.c
│      │      sys.h
│      └─usart
│              usart.c
│              usart.h
├─Middlewares
│      readme.txt
├─Output
│      atk_f407.hex
├─Projects
│  └─MDK-ARM
│          atk_f407.uvoptx
│          atk_f407.uvprojx
└─User
        main.c
        stm32f4xx_hal_conf.h
        stm32f4xx_it.c
        stm32f4xx_it.h
`