# ReadMe

## 程序基本架构

### main.c

- 进行电机的初始化（reset后自动发送一条报文进行电机搜索）
- TIM中断回调函数
  - 呼吸灯初始化
  - 警示灯检测



### Go_Driver.c

- Go电机报文解算以及控制报文的发送（函数名如同功能）
- AS曲线的解算
- Go电机参数FLASH写入



### usart_cmd.c

- 实现串口命令行控制



### CAN_Handler.c

- 实现can消息控制

---

## 常见问题及可能解决方式

1. 警告灯亮起此时重启电机导致疯转：
   1. 这是由于自动重连导致，可以注释掉TIM中断中的`motorOn`函数代码
   2. 注意安全
2. 发送命令导致疯转：
   1. 可能是曲线计算出现问题，可以尝试在AS曲线最后的else中加新的约束条件（前提是知道发生了什么）
3. 发送曲线命令卡死：
   1. 大概率是二分法求解出现问题，根据具体情况调整deadbond以及新增约束条件
   2. 也有可能是计算区间出现问题，可以加入新的约束条件进行重新计算（同上条）

4. 其他问题尚未遇见，祝你好运。