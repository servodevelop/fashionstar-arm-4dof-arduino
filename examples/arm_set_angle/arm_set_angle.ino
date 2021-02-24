/*
 * 设置机械臂关节的角度
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/04/23
 */
#include "FashionStar_Arm4DoF.h"

FSARM_ARM4DoF arm; //机械臂对象

void setup(){
    arm.init(); //机械臂初始化
}

void loop(){
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    thetas.theta1 = 0.0;
    thetas.theta2 = -60.0;
    thetas.theta3 = 105.0;
    thetas.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
                           // theta4 = -(theta2 + theta3)
    arm.setAngle(thetas);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    
    delay(1000); // 等待1s

    thetas.theta1 = 90.0;
    arm.setAngle(thetas);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    delay(1000);
}