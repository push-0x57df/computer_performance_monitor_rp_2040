
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