/*
 * 机械臂物块抓取
 * 将物块从一个位置移动到另外一个位置
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/14
 */
#include "FashionStar_Arm4DoF.h"

FSARM_ARM4DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
    arm.home(); // 机械臂回归到Home的位置
}

void loop(){
    // 测试物块抓取
    // 将物块从起始点(x1=9, y1=-13, z1=-5)
    // 抓取到目标点点(x2=9, y2=9, z2=-5)
    arm.grab(9, -13, -5, 9, 9, -5);
    // 停顿1s
    delay(1000);
    arm.grab(15, 0, -5, 9, 9, -5);
    // 停顿1s
    delay(1000);
    arm.grab(22, 1, -3, 9, 11, -3);
    // 停顿1s
    delay(1000);
    arm.grab(1, 22, -3, 11, 9, -3);
    // 停顿1s
    delay(1000);
}