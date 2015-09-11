#USB Wireless LAN for STM32F4xx

本软件使用uC/OS-III作为嵌入式操作系统，lwip作为网络协议栈，STM32F4xx作为
微处理器，移植了联发科的USB无线网卡驱动DPO_RT5572_LinuxSTA_2.6.1.3_20121022，
并针对嵌入式系统做了优化。测试过程可以达到2.97Mbits的上行速度,2.04Mbits的下
行速度。

本软件特性：
1.支持热插拔

2.支持WEP、WPAPSK-AES、WPAPSK-TKIP、WPA2PSK-AES、WPA2PSK-TKIP等认证和加密方式

3.支持802.11b/g/n

4.使用iwpriv工具对无线网卡进行配置

5.测试过的USB无线网卡型号RT3070


有关演示例子和更多帮助见 Doc\USB Wireless LAN for STM32F4xx Doc.pdf
