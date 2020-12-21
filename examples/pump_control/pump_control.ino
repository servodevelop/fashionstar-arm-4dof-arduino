/*
 * 气泵控制 
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/13
 */
#include "FashionStar_Arm4DoF.h"
FSARM_ARM4DoF arm; //机械臂对象

void setup(){
    arm.init();
    arm.home();
}

void loop(){
    arm.pumpOn();  // 气泵打开
    delay(1000);    // 等待1s
    arm.pumpOff(); // 气泵关闭
    delay(1000);    // 等待1s
}