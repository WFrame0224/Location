# Location
* 为多天线同步控制，实现利用2.4G射频芯片读取其RSSI的值，实现无线节点的定位，此处展示的是**锚节点**（Anchor）的部分
****

## 项目介绍
### 总体描述：
* 项目实现 **Semtech**公司的**sx1280**的*2.4G Lora*射频芯片，进行通信，读取数据包对应的**RSSI**的值，利用433MHz *Lora*芯片实现控制帧和数据帧的传输。
### 硬件平台：
* **STC8A8K64sA12**(新型51单片机)
### 软件平台：
* **keilV4**
* 采用C语言完成实现

### 附加说明：

1. 本项目中**sx1280-driver文件**为*sx1280*的51驱动文件，您可以使用其进行您的单片机方面的开发
2. **Semtech**公司的说明与介绍文档地址：[www.semtech.com/products/wireless-rf/24-ghz-transceivers/SX1280](https://www.semtech.com/products/wireless-rf/24-ghz-transceivers/SX1280)
3. **Semtech**公司提供的*STM32*的实例程序地址：[https://os.mbed.com/teams/Semtech/code/SX1280DevKit/](https://os.mbed.com/teams/Semtech/code/SX1280DevKit/)
