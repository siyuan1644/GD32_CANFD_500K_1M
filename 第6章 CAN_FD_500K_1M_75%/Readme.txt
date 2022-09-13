
硬件接口
led   PB12
CAN  PB8 PB9

USB
PA2 PA3


LIN   
uart1  PA2 PA3    RX必须要接上拉电阻
控制引脚 PB11


J1850
输入 
PWM_IN   PA0   TIMER1_CH0
VPW_IN    PA1   TIMER1_CH1

输出
PWM_OUT  PB6  TIMER3_CH0
VPM_OUT   PB7  TIMER3_CH1

例子:
USB 虚拟串口测试，速度500k 左右，不丢帧
帧格式:
55 AA LEN(2 BYTE)+DATA....

硬件地址
https://shop100592026.taobao.com/

