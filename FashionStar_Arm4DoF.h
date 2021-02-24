/*
 * FashionStar四自由度机械臂SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/13
 */
#ifndef _FASHION_STAR_ARM4DOF_H
#define _FASHION_STAR_ARM4DOF_H
#include "Arduino.h"
#include "FashionStar_UartServoProtocol.h"
#include "FashionStar_UartServo.h"

// 状态码
#define FSARM_STATUS uint8_t
#define FSARM_STATUS_SUCCESS 0 // 成功
#define FSARM_STATUS_FAIL 1 // 失败
#define FSARM_STATUS_JOINT1_OUTRANGE 2 // 关节1超出范围
#define FSARM_STATUS_JOINT2_OUTRANGE 3 // 关节2超出范围
#define FSARM_STATUS_JOINT3_OUTRANGE 4 // 关节3超出范围
#define FSARM_STATUS_JOINT4_OUTRANGE 5 // 关节4超出范围
#define FSARM_STATUS_TOOLPOSI_TOO_FAR 6 // 工具坐标目标点距离机械臂太遥远

// 机械臂常量
#define FSARM_SERVO_NUM 4 // 机械臂舵机的个数
#define FSARM_JOINT1 0 // 关节1对应的舵机ID
#define FSARM_JOINT2 1 // 关节2对应的舵机ID
#define FSARM_JOINT3 2 // 关节3对应的舵机ID
#define FSARM_JOINT4 3 // 关节4对应的舵机ID
#define FSARM_LINK1 0    // 连杆1的长度 单位cm
#define FSARM_LINK2 8    // 连杆2的长度 单位cm
#define FSARM_LINK3 7.6  // 连杆3的长度 单位cm
#define FSARM_LINK4 6.5  // 连杆4的长度 单位cm
#define FSARM_TOOL_LENGTH 4.0 // 工具的长度

// 舵机标定参数
#define FSARM_JOINT1_P90 -85.10  	//关节1为90°时的舵机原始角度
#define FSARM_JOINT1_N90 90.23   	//关节1为-90°时的舵机原始角度
#define FSARM_JOINT2_P0  90.80   	//关节2为0°时的舵机原始角度
#define FSARM_JOINT2_N90 0.90    	//关节2为-90°时的舵机原始角度
#define FSARM_JOINT3_P90 -45.5   	//关节3为90°时的舵机原始角度
#define FSARM_JOINT3_N90 130.90  	//关节3为-90°时的舵机原始角度
#define FSARM_JOINT4_P90 -93.4   	//关节4为90°时的舵机原始角度
#define FSARM_JOINT4_N90 84.3    	//关节4为-90°时的舵机原始角度

// 设置关节角度的约束
#define FSARM_JOINT1_MIN -135.0
#define FSARM_JOINT1_MAX 135.0
#define FSARM_JOINT2_MIN -135.0
#define FSARM_JOINT2_MAX 0.0
#define FSARM_JOINT3_MIN -90.0
#define FSARM_JOINT3_MAX 160.0 
#define FSARM_JOINT4_MIN -135.0
#define FSARM_JOINT4_MAX 135.0

// HOME 
// 摄像头处于工作台中心的正上方, 
// 工作台的四个角点刚刚填满整个画面.
#define FSARM_HOME_X 12
#define FSARM_HOME_Y 0
#define FSARM_HOME_Z 10

// 气泵的配置
#define PUMP_SERVO_ID 0xFE

// // 轨迹模式
// #define FSARM_PLAN_T uint8_t //轨迹规划的模式
// #define FSARM_PLAN_P2P 0 // 点到点, 对轨迹无要求
// #define FSARM_PLAN_LINE 1 // 直线轨迹模式
// #define FSARM_PLAN_DOOR 2 // 门式轨迹模式-抬起,直线,落下
// #define FSARM_LINE_STEP 1.0 //每一步的步长cm
// #define FSARM_DOOR_LIFT 3.0 //抬起的高度cm

// 机械臂的模式
// #define FSARM_STATE_T  uint8_t // 机械臂的状态
// #define FSARM_STATE_IDLE 0 // 空闲状态,等待新的指令传入
// #define FSARM_STATE_BUSY 1 // 机械臂正在执行动作
// #define FSARM_STATE_PAUSE 2 // 机械臂被暂停执行动作

// 笛卡尔空间下的点
typedef struct{
    float x;
    float y;
    float z;
}FSARM_POINT3D_T;

typedef struct{
    float theta1;
    float theta2;
    float theta3;
    float theta4;
}FSARM_JOINTS_STATE_T;

class FSARM_ARM4DoF{
public:
    FSARM_ARM4DoF();
    // 初始化
    void init();
    // 初始化舵机
    void initServos();
    // 关节标定
    void calibration();
    // 设置舵机角度范围
    void setAngleRange();
    // 开启扭矩
    void setTorque(bool enable);
    // 设置舵机为阻尼模式
    void setDamping();
    // 设置所有舵机的转速
    void setSpeed(FSUS_SERVO_SPEED_T speed);
    // 读取舵机原始角度
    void queryRawAngle(FSARM_JOINTS_STATE_T* thetas);
    // 读取角度
    void queryAngle(FSARM_JOINTS_STATE_T* thetas);
    // 设置舵机的原始角度
    void setRawAngle(FSARM_JOINTS_STATE_T thetas);
    // 设置舵机的角度
    void setAngle(FSARM_JOINTS_STATE_T thetas);
    // 设置舵机的角度(附带Link45平行与桌面的约束)
    void setAngle2(FSARM_JOINTS_STATE_T thetas);
    // 设置舵机的角度
    void setAngle(FSARM_JOINTS_STATE_T thetas, uint16_t interval);
    // 机械臂正向运动学
    void forwardKinematics(FSARM_JOINTS_STATE_T thetas, FSARM_POINT3D_T* toolPosi);
    // 机械臂逆向运动学
    FSARM_STATUS inverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* thetas);
    // 腕关节转换为末端的坐标
    void transWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi);
    // 末端的坐标转换为腕关节的坐标
    void transTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi);
    // 机械臂末端移动, 点对点
    FSARM_STATUS move(FSARM_POINT3D_T toolPosi);
    FSARM_STATUS move(float tx, float ty, float tz);
    FSARM_STATUS move(float tx, float ty, float tz, bool isWait);
    // 直线插补(简易)
    FSARM_STATUS line(float tx, float ty, float tz);
    FSARM_STATUS line(FSARM_POINT3D_T goalPosi);

    // home: 回归机械零点, 初始化机械臂的姿态
    void home();
    // 返回机械臂是否空闲
    bool isIdle(); 
    // 等待舵机停止
    void wait();
    // 更新末端的位置
    void getToolPosi(FSARM_POINT3D_T *toolPosi);
    // 初始化气泵
    void initPump();
    // 气泵开启
    void pumpOn();
    // 气泵关闭
    void pumpOff();
    // 物块抓取
    void grab(float x1, float y1, float z1, float x2, float y2, float z2);
    // FSARM_POINT3D_T curEndPosi; // 末端当前在机械臂基坐标系下的位置
    FSUS_Protocol protocol; // 舵机串口通信协议
    FSUS_Servo servos[FSARM_SERVO_NUM]; // 舵机列表
    bool _isIdle; // 机械臂是否空闲
    FSUS_Servo pump;
private:
};

#endif
