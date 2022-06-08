# <center>STM32F4飞控（凌霄MCU）

> 基于[匿名科创](http://www.anotc.com/wiki/%E5%8C%BF%E5%90%8D%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E8%B5%84%E6%96%99%E4%B8%8B%E8%BD%BD%E9%93%BE%E6%8E%A5%E6%B1%87%E6%80%BB)官方代码

## 文件结构

* DriversBsp - 通用驱动
* DriversMcu/STM32F407 - 板级驱动
* FcSrc - 凌霄飞控相关通讯模块和飞控任务调度模块
* python_sdk - 二级下位机（树莓派）python通讯接口

## TODO

1. 从旧飞控移植树莓派通讯代码过来
2. 更改python sdk适配凌霄的命令树
3. 添加绕圈等连续操作的sdk
4. 完善飞控状态数据下发到树莓派的流程
5. 尝试把激光雷达解算打包发给凌霄
6. more

