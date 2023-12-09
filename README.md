
# 计算机性能监视器程序 v1.0

本程序用于监控计算机性能，运行在特定硬件上

## 串口通信定义

概述：本程序使用的串口每次通信发送或读取的数据量为32bit，前16bit固定为0x5aa5，下8bit表示命令，最后8bit表示具体携带的数值。

|命令|命令代码|值定义|备注|
| -- | -- |-- |-- |
|握手|ff|01：尝试连接 <br/> 10：连接确认 |无|
|CPU温度|01|ff：接收方发送，表示接收成功 <br/> 其他值：具体温度数值，单位摄氏度|无|
|CPU利用率|02|ff：接收方发送，表示接收成功 <br/> 其他值：具体利用率数值，单位百分比|无|
|RAM利用率|03|ff：接收方发送，表示接收成功 <br/> 其他值：具体利用率数值，单位百分比|无|
|RAM已用|04|ff：接收方发送，表示接收成功 <br/> 其他值：具体已使用数值，单位GB|无|
|RAM总记|05|ff：接收方发送，表示接收成功 <br/> 其他值：具体总量数值，单位GB|无|
|GPU温度|06|ff：接收方发送，表示接收成功 <br/> 其他值：具体温度数值，单位摄氏度|无|
|GPU利用率|07|ff：接收方发送，表示接收成功 <br/> 其他值：具体利用率数值，单位百分比|无|
|GPU内存利用率|08|ff：接收方发送，表示接收成功 <br/> 其他值：具体利用率数值，单位百分比|无|
|GPU内存已用|09|ff：接收方发送，表示接收成功 <br/> 其他值：具体已使用数值，单位GB|无|
|GPU内存总量|0A|ff：接收方发送，表示接收成功 <br/> 其他值：具体内存总量数值，单位GB|无|

## 搭建开发和编译环境

本程序使用RP2040官方 C++ SDK 在 Windows 系统下开发。使用的开发工具为 visual studio code 编辑器开发

想要编译该程序，需要先安装工具链，这部分请参照文档 Getting started with Raspberry Pi Pico: C/C++ development 中的 9.2. Building on MS Windows 章节，有详细的安装工具链指南。安装完成后能够编译 example 中的程序即可。

想要调试该程序也可参照 Getting started with Raspberry Pi Pico: C/C++ development 文档操作，需要的调试仿真器可以使用文档中 Appendix A: Using Picoprobe 中的硬件实现

Getting started with Raspberry Pi Pico: C/C++ development 文档在这里获取： https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf

在编译本项目前，需要先下载 lvgl-8.3.10 源代码放在本项目根目录下，将目录名称改为 lvgl-8.3.10，链接：https://github.com/lvgl/lvgl/releases/tag/v8.3.10

## 相关开源资料

- Windows端程序：https://github.com/push-0x57df/computer_performance_monitor
- 硬件设计资料：https://oshwhub.com/push_a/xing-neng-jian-shi-qi