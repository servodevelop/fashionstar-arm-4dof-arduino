/*
 * 测试机械臂正向运动学
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/04/23
 */
#include <SoftwareSerial.h>
#include "FashionStar_Arm4DoF.h"
// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 创建软串口
FSARM_ARM4DoF arm; //机械臂对象

void testForwardKinematics(){
    FSARM_JOINTS_STATE_T thetas; // 关节角度
    thetas.theta1 = 0.0;
    thetas.theta2 = -60.0;
    thetas.theta3 = 105.0;
    thetas.theta4 = -45.0; // 注: 因为theta4会被自动算出来,有一个末端水平的约束
                           // theta4 = -(theta2 + theta3)
    arm.setAngle(thetas);  // 设置舵机旋转到特定的角度
    arm.wait();            // 等待舵机旋转到目标位置
    
    FSARM_POINT3D_T toolPosi; // 末端的位置
    arm.forwardKinematics(thetas, &toolPosi); // 正向运动学
    // 打印正向运动学的结果
    softSerial.println("Tool Posi: X= " + String(toolPosi.x, 1) +\
         ", Y= " + String(toolPosi.y, 1) + \
         ", Z= " + String(toolPosi.z, 1));

}
void setup(){
    softSerial.begin(SOFT_SERIAL_BAUDRATE); // 初始化软串口
    arm.init(); //机械臂初始化
    arm.setDamping(); //设置舵机为阻尼模式

    softSerial.println("Test Forward Kinematics");
    testForwardKinematics();
}

void loop(){

}