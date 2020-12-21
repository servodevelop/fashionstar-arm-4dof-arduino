/*
 * 测试机械臂直线插补测试
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/05/14
 */
#include "FashionStar_Arm4DoF.h"

FSARM_ARM4DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
}

void loop(){
    arm.line(12, 0, -5);
    delay(1000);
    arm.line(9, -13, -5);
    delay(1000);
    arm.line(9, 9, -5);
    delay(1000);
}