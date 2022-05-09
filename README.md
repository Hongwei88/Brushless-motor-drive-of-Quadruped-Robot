# MIT_MiniDogKeilSource

MIT Mini Dog Source for KEIL  
2019.09.19

1. 修改 KEIL 中 Output 输入，勾选 Create HEX File，勾选 Browse Information;
2. 修改 KEIL 中 Define 内容格式空格用','替代；
3. 工程移除 stm32f4xx_hal_pcb.c, stm32f4xx_hal_hcd.c, stm32f4xx_hal_nand.c, stm32f4xx_hal_nor.c, stm32f4xx_hal_sdram.c, stm32f4xx_hal_sram.c, stm32f4xx_hal_dcmi.c, stm32f4xx_hal_dcmi_ex.c, stm32f4xx_hal_dfsdm.c, stm32f4xx_hal_eth.c, stm32f4xx_hal_dsi.c, stm32f4XX_hal_smartcard.c, stm32f4xx_hal_spdifrx.c, stm32f4xx_hal_sai.c, stm32f4xx_hal_sai_ex.c, stm32f4xx_hal_wwdg.c, stm32f4xx_hal_rng.c, stm32f4xx_hal_sd.c, stm32f4xx_hal_pccard.c, stm32f4xx_hal_i2s.c, stm32f4xx_hal_i2s_ex.c, stm32f4xx_hal_cec.c, stm32f4xx_hal_crc.c, stm32f4xx_hal_crpy.c, stm32f4xx_hal_crpy_ex.c, stm32f4xx_hal_fmpi2c.c, stm32f4xx_hal_fmpi2c_ex.c, stm32f4xx_hal_hash.c, stm32f4xx_hal_hal_hash_ex.c, stm32f4xx_hal_irda.c, stm32f4xx_hal_lptim.c, stm32f4xx_hal_ltdc.c, stm32f4xx_hal_ltdc_ex.c, stm32f4xx_hal_mmc.c,stm32f4xx_hal_pcd_ex.c,stm32f4xx_hal_msp_template.c, trng_api.c, stm32f4xx_hal_dma2d.c, stm32f4xx_hal_qspi.c， stm32f4xx_hal_iwdg.c
4. KEIL 而配置 ST-LINK 调试;
5. 工程移除所有的 LL 库文件;
6. 修改 stm32f4 配置文件;  
   2019.09.20
7. 删除程序中没有用到的 void Init_DAC(void)函数；
8. DRV.cpp 文件中写寄存器的函数，统一调用 write_register();
9. 修复读取 DRV8323 状态寄存器 bug;
10. 优化串口中断代码，添加 switch case 均添加 default 情况；
11. 删除 COMStruct 结构体;
12.
