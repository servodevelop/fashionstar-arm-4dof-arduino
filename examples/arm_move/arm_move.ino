/*
 * 测试机械臂点控, 从一个点运动到另外一个点
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
    // 机械臂末端移动到 x1=10, y1=0, z1=5
    // 并且阻塞式等待, 直到机械臂运动到目标角度, 才结束该语句
    arm.move(10, 0, 5, true);
    delay(1000); // 停顿1s
    
    arm.move(8, 8, -4, true);
    delay(1000); // 停顿1s

    arm.move(6, -6, 5, true);
    delay(1000); // 停顿1s
}