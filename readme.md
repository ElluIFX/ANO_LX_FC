# STM32F4飞控（凌霄MCU）

> 基于[匿名科创](http://www.anotc.com/wiki/%E5%8C%BF%E5%90%8D%E4%BA%A7%E5%93%81%E8%B5%84%E6%96%99/%E8%B5%84%E6%96%99%E4%B8%8B%E8%BD%BD%E9%93%BE%E6%8E%A5%E6%B1%87%E6%80%BB)官方代码

## 文件结构

* DriversBsp - 通用驱动
* DriversMcu/STM32F407 - 板级驱动
* FcSrc - 凌霄飞控相关通讯模块和飞控任务调度模块
  * ANO* 凌霄通讯相关
  * \*FC* 飞控相关
  * User* sdk交互相关
* python_sdk - 二级下位机（树莓派）python通讯接口
  * FlightController 飞控控制类
  * camRecv / camTrans 图传相关
  * fc_gui 图形化上位机
  * mission* 任务相关
  * server 飞控服务网络服务器

## TODO

***ALL DONE***
