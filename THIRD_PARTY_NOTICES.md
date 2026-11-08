# 第三方代码与来源说明

本仓库是学习和工程实践项目，包含不属于本人原创的代码。为避免许可与署名含糊，逐项说明如下。根目录 `LICENSE`（MIT）**仅覆盖本人编写的原创部分**。

| 路径 | 来源 | 说明 |
|---|---|---|
| `Drivers/STM32F1xx_HAL_Driver/`、`Drivers/CMSIS/` | STMicroelectronics | STM32CubeF1 固件库，随库附带的许可与文件头版权声明继续适用。本仓库未修改这些文件。 |
| `Core/Src/main.c`、`gpio.c`、`i2c.c`、`tim.c`、`usart.c`、`stm32f1xx_it.c`、`stm32f1xx_hal_msp.c`、`system_stm32f1xx.c` 等 | STM32CubeMX 生成 | 由 `balanceCar.ioc` 生成；`USER CODE BEGIN/END` 区段内的内容为本人的修改。重新生成 ioc 时请注意备份。 |
| `MyCode/inv_mpu.c`、`inv_mpu.h`、`inv_mpu_dmp_motion_driver.c/.h`、`dmpKey.h`、`dmpmap.h` | InvenSense / TDK Motion Driver 参考实现 | MPU6050 与 DMP 驱动，版权归原作者，遵循各文件头自带的使用条款。`IIC.c/.h`、`mpu6050.c/.h` 为针对本硬件的适配层。 |
| `Core/Src/oled.c`、`Core/Inc/oledfont.h` | SSD1306 常用开源驱动 | 字库与驱动来自公开参考实现。 |
| `startup_stm32f103xb.s`、`STM32F103C8Tx_FLASH.ld`、`cmake/` | ARM / ST / STM32CubeMX 模板 | 启动文件与链接脚本由工具链和 CubeMX 提供。 |

## 项目自身的来源边界

直立环、速度环的基础控制框架，以及硬件驱动与 MPU6050 库，来自公开开源参考实现。本人的工作是在这块具体硬件上完成移植与实测标定、诊断并修复三个有数据支撑的稳定性问题、重新设计速度环限幅结构与转向环、补齐超声波避障与蓝牙遥测链路，以及完整的实测记录。详见 `README.md` 的"来源与边界"和 `docs/技术深潜.md`。

如果某个文件的原始版权持有人认为此处的说明不准确或需要补充署名，欢迎提 issue，会及时修正。
