/*
 * 机械臂末端移动到Home的位置
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/04/23
 */

#include "FashionStar_Arm4DoF.h"

FSARM_ARM4DoF arm; //机械臂对象


void setup(){
    arm.init(); //机械臂初始化
    arm.home();
    // 手动调整值
    // arm.move(12, 0, 10, true);
}

void loop(){
    
}