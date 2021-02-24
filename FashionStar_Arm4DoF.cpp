/*
 * FashionStar四自由度机械臂SDK
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: kyle.xing@fashionstar.com.hk
 * 更新时间: 2020/05/13
 */
#include "FashionStar_Arm4DoF.h"

FSARM_ARM4DoF::FSARM_ARM4DoF(){

}

// 初始化
void FSARM_ARM4DoF::init(){
    this->protocol.init(115200); // 初始化通信协议
    initServos();   // 初始化所有的舵机
    initPump();     // 初始化气泵
    calibration();  // 机械臂标定
    setAngleRange();// 设置舵机角度范围
    setSpeed(100);  // 初始化舵机的转速
}

// 初始化舵机
void FSARM_ARM4DoF::initServos(){
    // 舵机初始化
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].init(sidx, &this->protocol);
    }   
}



// 关节标定
void FSARM_ARM4DoF::calibration(){
    this->servos[FSARM_JOINT1].calibration(FSARM_JOINT1_P90, 90.0, FSARM_JOINT1_N90, -90.0);
    this->servos[FSARM_JOINT2].calibration(FSARM_JOINT2_P0, 0.0, FSARM_JOINT2_N90, -90.0);
    this->servos[FSARM_JOINT3].calibration(FSARM_JOINT3_P90, 90.0, FSARM_JOINT3_N90, -90.0);
    this->servos[FSARM_JOINT4].calibration(FSARM_JOINT4_P90, 90.0, FSARM_JOINT4_N90, -90.0);
}

// 设置舵机的角度范围
void FSARM_ARM4DoF::setAngleRange(){
    this->servos[FSARM_JOINT1].setAngleRange(FSARM_JOINT1_MIN, FSARM_JOINT1_MAX);
    this->servos[FSARM_JOINT2].setAngleRange(FSARM_JOINT2_MIN, FSARM_JOINT2_MAX);
    this->servos[FSARM_JOINT3].setAngleRange(FSARM_JOINT3_MIN, FSARM_JOINT3_MAX);
    this->servos[FSARM_JOINT4].setAngleRange(FSARM_JOINT4_MIN, FSARM_JOINT4_MAX);
}

// 开启扭矩
void FSARM_ARM4DoF::setTorque(bool enable){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].setTorque(enable);
    }
}
// 设置阻尼模式
void FSARM_ARM4DoF::setDamping(){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].setDamping(1000);
    }
}

// 设置所有舵机的转速
void FSARM_ARM4DoF::setSpeed(FSUS_SERVO_SPEED_T speed){
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].setSpeed(speed);
    }
    // this->servos[FSARM_JOINT1].setSpeed(1.0*speed);
    // this->servos[FSARM_JOINT2].setSpeed(1*speed);
    // this->servos[FSARM_JOINT3].setSpeed(1.5*speed);
    // this->servos[FSARM_JOINT4].setSpeed(2*speed);
    /*
    for(uint8_t sidx=0; sidx < FSARM_SERVO_NUM; sidx++){
        if (sidx == FSARM_JOINT4){
            this->servos[sidx].setSpeed(2*speed);
        }else{
            this->servos[sidx].setSpeed(speed);
        }
    }
    */
}

// 读取舵机原始角度
void FSARM_ARM4DoF::queryRawAngle(FSARM_JOINTS_STATE_T* thetas){
    thetas->theta1 = this->servos[FSARM_JOINT1].queryRawAngle();
    thetas->theta2 = this->servos[FSARM_JOINT2].queryRawAngle();
    thetas->theta3 = this->servos[FSARM_JOINT3].queryRawAngle();
    thetas->theta4 = this->servos[FSARM_JOINT4].queryRawAngle();
}

// 读取角度
// 查询所有舵机的角度并填充在thetas里面
void FSARM_ARM4DoF::queryAngle(FSARM_JOINTS_STATE_T* thetas){
    thetas->theta1 = this->servos[FSARM_JOINT1].queryAngle();
    thetas->theta2 = this->servos[FSARM_JOINT2].queryAngle();
    thetas->theta3 = this->servos[FSARM_JOINT3].queryAngle();
    thetas->theta4 = this->servos[FSARM_JOINT4].queryAngle();
}

// 设置舵机的原始角度
void FSARM_ARM4DoF::setRawAngle(FSARM_JOINTS_STATE_T thetas){
    this->servos[FSARM_JOINT1].setRawAngle(thetas.theta1);
    this->servos[FSARM_JOINT2].setRawAngle(thetas.theta2);
    this->servos[FSARM_JOINT3].setRawAngle(thetas.theta3);
    this->servos[FSARM_JOINT4].setRawAngle(thetas.theta4);
}

// 设置舵机的角度
void FSARM_ARM4DoF::setAngle(FSARM_JOINTS_STATE_T thetas){
    this->servos[FSARM_JOINT1].setAngle(thetas.theta1);
    this->servos[FSARM_JOINT2].setAngle(thetas.theta2);
    this->servos[FSARM_JOINT3].setAngle(thetas.theta3);
    this->servos[FSARM_JOINT4].setAngle(thetas.theta4);
}

// 设置舵机的角度
void FSARM_ARM4DoF::setAngle(FSARM_JOINTS_STATE_T thetas, uint16_t interval){
    this->servos[FSARM_JOINT1].setAngle(thetas.theta1, interval);
    this->servos[FSARM_JOINT2].setAngle(thetas.theta2, interval);
    this->servos[FSARM_JOINT3].setAngle(thetas.theta3, interval);
    this->servos[FSARM_JOINT4].setAngle(thetas.theta4, interval);
}

// 机械臂正向运动学
void FSARM_ARM4DoF::forwardKinematics(FSARM_JOINTS_STATE_T thetas, FSARM_POINT3D_T* toolPosi){
    FSARM_POINT3D_T wristPosi; //腕关节原点的坐标
    // 角度转弧度
    float theta1 = radians(thetas.theta1);
    float theta2 = radians(thetas.theta2);
    float theta3 = radians(thetas.theta3);
    float theta4 = radians(thetas.theta4); // 暂时没用到
    // 计算腕关节的坐标
    wristPosi.x = cos(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.y = sin(theta1) * (FSARM_LINK2*cos(theta2)+FSARM_LINK3*cos(theta2+theta3));
    wristPosi.z = -FSARM_LINK2*sin(theta2)-FSARM_LINK3*sin(theta2+theta3);
    // 计算末端的坐标
    transWrist2Tool(wristPosi, toolPosi);
}



// 机械臂逆向运动学
FSARM_STATUS FSARM_ARM4DoF::inverseKinematics(FSARM_POINT3D_T toolPosi, FSARM_JOINTS_STATE_T* thetas){
    // 关节弧度
    float theta1 = 0.0;
    float theta2 = 0.0;
    float theta3 = 0.0;
    float theta4 = 0.0;
    FSARM_POINT3D_T wristPosi; // 腕关节坐标
    transTool2Wrist(toolPosi, &wristPosi); //工具坐标转换为腕关节坐标
    // 根据腕关节原点距离机械臂基坐标系的直线距离
    float disO2WristXY = sqrt(pow(wristPosi.x,2) + pow(wristPosi.y, 2) + pow(wristPosi.z, 2));
    if (disO2WristXY > (FSARM_LINK2+FSARM_LINK3)){
        return FSARM_STATUS_TOOLPOSI_TOO_FAR;
    }
    // 判断腕关节的原点是否在机械臂坐标系的Z轴上
    if (toolPosi.x == 0 && toolPosi.y == 0){
        // 让theta1保持跟原来相同
        theta1 = radians(this->servos[FSARM_JOINT1].queryRawAngle());
    }else{
        // 求解theta1
        theta1 = atan2(toolPosi.y, toolPosi.x);
        thetas->theta1 = degrees(theta1);
        // 判断theta1是否合法
        if (!servos[FSARM_JOINT1].isAngleLegal(thetas->theta1)){
            return FSARM_STATUS_JOINT1_OUTRANGE;
        }
    }
    // 计算theta3
    float b;
    if(cos(theta1) !=0){
        b = wristPosi.x / cos(theta1);
    }else{
        b = wristPosi.y / sin(theta1);
    }
    float cos_theta3 = (pow(wristPosi.z, 2)+pow(b,2) - pow(FSARM_LINK2,2) - pow(FSARM_LINK3, 2))/(2*FSARM_LINK2*FSARM_LINK3);
    float sin_theta3 = sqrt(1 - pow(cos_theta3, 2));
    theta3 = atan2(sin_theta3, cos_theta3);
    thetas->theta3 = degrees(theta3);
    if(!servos[FSARM_JOINT3].isAngleLegal(thetas->theta3)){
        return FSARM_STATUS_JOINT3_OUTRANGE;
    }
    // 计算theta2
    float k1 = FSARM_LINK2 + FSARM_LINK3*cos(theta3);
    float k2 = FSARM_LINK3 * sin(theta3);
    float r = sqrt(pow(k1, 2) + pow(k2, 2));
    theta2 = atan2(-wristPosi.z/r, b/r) - atan2(k2/r, k1/r);
    thetas->theta2 = degrees(theta2);
    if(!servos[FSARM_JOINT2].isAngleLegal(thetas->theta2)){
        return FSARM_STATUS_JOINT2_OUTRANGE;
    }
    // 计算theta4
    theta4 = -(theta2 + theta3);
    thetas->theta4 = degrees(theta4);
    if(!servos[FSARM_JOINT4].isAngleLegal(thetas->theta4)){
        return FSARM_STATUS_JOINT4_OUTRANGE;
    }

    // 成功完成求解
    return FSARM_STATUS_SUCCESS;
}

// 腕关节转换为末端的坐标
void FSARM_ARM4DoF::transWrist2Tool(FSARM_POINT3D_T endPosi, FSARM_POINT3D_T* toolPosi){
   float theta1_rad = atan2(endPosi.y, endPosi.x);
   toolPosi->x = endPosi.x + FSARM_LINK4*cos(theta1_rad);
   toolPosi->y = endPosi.y + FSARM_LINK4*sin(theta1_rad);
   toolPosi->z = endPosi.z - FSARM_TOOL_LENGTH;
}

// 末端的坐标转换为腕关节的坐标
void FSARM_ARM4DoF::transTool2Wrist(FSARM_POINT3D_T toolPosi, FSARM_POINT3D_T* endPosi){
    float theta1_rad = atan2(toolPosi.y, toolPosi.x); 
    endPosi->x = toolPosi.x - FSARM_LINK4*cos(theta1_rad);
    endPosi->y = toolPosi.y - FSARM_LINK4*sin(theta1_rad);
    endPosi->z = toolPosi.z + FSARM_TOOL_LENGTH;
}

// 机械臂末端移动, 点对点
FSARM_STATUS FSARM_ARM4DoF::move(FSARM_POINT3D_T toolPosi){
    FSARM_JOINTS_STATE_T thetas;
    FSARM_STATUS status = inverseKinematics(toolPosi, &thetas); // 逆向运动学
    if(status == FSARM_STATUS_SUCCESS){
        // 设置舵机的角度
        setAngle2(thetas);
        wait();
    }
    return status;
}

// 设置关节1234的角度, 期间要求Link45保持水平
// 不断的发送角度控制指令
void FSARM_ARM4DoF::setAngle2(FSARM_JOINTS_STATE_T thetas){
    float theta1 = thetas.theta1;
    float theta2 = thetas.theta2;
    float theta3 = thetas.theta3;
    float theta4 = thetas.theta4;

    // 查询当前的舵机角度
    FSARM_JOINTS_STATE_T curThetas;
    queryAngle(&curThetas); 

    float dTheta1 = theta1 - curThetas.theta1;
    float dTheta2 = theta2 - curThetas.theta2;
    float dTheta3 = theta3 - curThetas.theta3;
    float dTheta4 = theta4 - curThetas.theta4;

    float nextTheta1;
    float nextTheta2;
    float nextTheta3;
    float nextTheta4;
    float ratio;

    // 先简单的做笛卡尔距离
    float maxDis = max(max(fabs(dTheta1), fabs(dTheta2)), max(fabs(dTheta3), fabs(dTheta4))); // 找到角度差值的最大值
    long nStep = (long)(maxDis / 1.0); // 获取 步数
    long tStart,tPass;
    for (long i=1; i<=nStep; i++){
        tStart = millis();
        ratio = ((float)i / (float)nStep); // ratio的取值范围 [0, 1]
        // 通过S曲线对ratio进行平滑
        ratio = 0.5*(1-sin(radians(90+ 180 *ratio)));
        nextTheta1 = dTheta1*ratio + curThetas.theta1;
        nextTheta2 = dTheta2*ratio + curThetas.theta2;
        nextTheta3 = dTheta3*ratio + curThetas.theta3;
        nextTheta4 = -1.0 * (nextTheta2 + nextTheta3);
        // 设置舵机角度
        this->servos[FSARM_JOINT1].setAngle(nextTheta1, 0);
        this->servos[FSARM_JOINT2].setAngle(nextTheta2, 0);
        this->servos[FSARM_JOINT3].setAngle(nextTheta3, 0);
        this->servos[FSARM_JOINT4].setAngle(nextTheta4, 0);
        tPass = millis() - tStart;
        if(tPass < 20){
            delay(20 - tPass);
        }
        
    }
    // wait();
}

// 直线插补(简单实现)
FSARM_STATUS FSARM_ARM4DoF::line(FSARM_POINT3D_T goalPosi){
    FSARM_POINT3D_T curPosi;
    FSARM_POINT3D_T nextPosi;
    FSARM_JOINTS_STATE_T thetas;
    FSARM_STATUS status;
    // 获取工具的位置
    getToolPosi(&curPosi);
    // 计算偏移量
    float dx = goalPosi.x - curPosi.x;
    float dy = goalPosi.y - curPosi.y;
    float dz = goalPosi.z - curPosi.z;
    float distance = sqrt(dx*dx + dy*dy +dz*dz);
    float nx, ny, nz;

    float ratio;
    long nStep = (long)(distance / 0.2);
    long tStart, tPass;
    for (long i=1; i<=nStep; i++){
        tStart = millis();
        ratio = (i*1.0 / nStep); // ratio的取值范围 [0, 1]
        // 通过S曲线对ratio进行平滑
        ratio = 0.5*(1-sin(radians(90+ 180 *ratio)));
        // 填写
        nx = curPosi.x + dx*ratio;
        ny = curPosi.y + dy*ratio;
        nz = curPosi.z + dz*ratio;
        nextPosi.x = nx;
        nextPosi.y = ny;
        nextPosi.z = nz;

        // 逆向运动学求解
        status = inverseKinematics(nextPosi, &thetas); // 逆向运动学
        if(status == FSARM_STATUS_SUCCESS){
            // 设置舵机的角度
            // setAngle(thetas);
            this->servos[FSARM_JOINT1].setAngle(thetas.theta1, 0);
            this->servos[FSARM_JOINT2].setAngle(thetas.theta2, 0);
            this->servos[FSARM_JOINT3].setAngle(thetas.theta3, 0);
            this->servos[FSARM_JOINT4].setAngle(thetas.theta4, 0);
        }

        tPass = millis() - tStart;
        if(tPass < 20){
            delay(20 -tPass);
        }
        
        // move(nx, ny, nz);
        // delay(15);
    }
    // 等待到达目的地
    wait();
    
    return status;
}

FSARM_STATUS FSARM_ARM4DoF::line(float tx, float ty, float tz){
    FSARM_POINT3D_T goalPosi;
    goalPosi.x = tx;
    goalPosi.y = ty;
    goalPosi.z = tz;
    return line(goalPosi);
}

FSARM_STATUS FSARM_ARM4DoF::move(float tx, float ty, float tz){
    FSARM_POINT3D_T toolPosi;
    toolPosi.x = tx;
    toolPosi.y = ty;
    toolPosi.z = tz;
    return move(toolPosi);
}

FSARM_STATUS FSARM_ARM4DoF::move(float tx, float ty, float tz, bool isWait){
    move(tx, ty, tz);
    if(isWait){
        wait();
    }
}

// home: 回归机械零点, 初始化机械臂的姿态
void FSARM_ARM4DoF::home(){
    move(FSARM_HOME_X, FSARM_HOME_Y, FSARM_HOME_Z);
    wait();
}

// 返回机械臂是否空闲
bool FSARM_ARM4DoF::isIdle(){
    bool is_stop = true;
    for(int sidx=0; sidx<FSARM_SERVO_NUM; sidx++){
        is_stop &= this->servos[sidx].isStop();
    }
    return is_stop;
}

// 等待舵机停止
void FSARM_ARM4DoF::wait(){
    for(int sidx=0; sidx<FSARM_SERVO_NUM; sidx++){
        this->servos[sidx].wait();
    }
}

// 更新末端工具的坐标
void FSARM_ARM4DoF::getToolPosi(FSARM_POINT3D_T *toolPosi){
    FSARM_JOINTS_STATE_T thetas;
    queryAngle(&thetas);
    forwardKinematics(thetas, toolPosi);
}

void FSARM_ARM4DoF::initPump(){
    // 气泵初始化
    this->pump.init(PUMP_SERVO_ID, &this->protocol);
}

// 开启气泵(抽气)
void FSARM_ARM4DoF::pumpOn(){
    this->pump.setAngle(-90, 0); // 打开电机
}

// 关闭气泵(放气)
void FSARM_ARM4DoF::pumpOff(){
    this->pump.setAngle(0, 0); // 关闭电机
    delay(100);
    this->pump.setAngle(90, 0); // 打开电磁阀
    delay(500);
    this->pump.setAngle(0, 0); // 关闭电磁阀
}

void FSARM_ARM4DoF::grab(float x1, float y1, float z1, float x2, float y2, float z2){
    line(x1, y1, z1+4); // 运动到目标点上方
    delay(200);
    line(x1, y1, z1); // 运动到目标点
    delay(200);
    pumpOn(); // 开启气泵
    delay(200);
    setSpeed(50);
    line(x1, y1, z1-1); // 气泵下压
    delay(200);
    line(x1, y1, z1+4); // 抬起
    delay(200);
    setSpeed(100);
    line(x2, y2, z2+4); // 运动到目标点上方
    delay(200);
    setSpeed(50);
    line(x2, y2, z2); // 运动到目标点
    delay(200);
    pumpOff(); // 关闭气泵
    delay(200);
    line(x2, y2, z2+4); // 运动到目标点上方
    setSpeed(100);
}
