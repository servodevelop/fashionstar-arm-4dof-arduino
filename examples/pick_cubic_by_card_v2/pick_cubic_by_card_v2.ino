/**
 * 机械臂通过Pixy2进行物块抓取
 * 通过卡片来选择要抓取的色块的颜色
 * 使用高斯分布进行卡片颜色的判别
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/08/13
 */
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <SoftwareSerial.h>
#include "FashionStar_Arm4DoF.h"
#include <Wire.h> // I2C库
#include "Adafruit_TCS34725.h" // 颜色传感器库

#define E 2.718281828459

// 颜色的代码
#define COLOR_NUM 5     // 颜色的个数
#define COLOR_NONE 0    // 什么颜色都不是
#define COLOR_RED 1     // 红色
#define COLOR_GREEN 2   // 绿色
#define COLOR_BLUE 3    // 蓝色
#define COLOR_YELLOW 4  // 黄色

#define BUTTON_GPIO 2 // 按键的管脚
#define BUZZER_GPIO 8 // 蜂鸣器的管脚

// RGB模块的GPIO分配,注意管脚都必须是模拟管脚
#define LED_RED_GPIO 9
#define LED_GREEN_GPIO 10
#define LED_BLUE_GPIO 11

// 软串口的配置
#define SOFT_SERIAL_RX 6
#define SOFT_SERIAL_TX 7
#define SOFT_SERIAL_BAUDRATE 4800

// 画面尺寸(单位:像素)
#define PIXY_IMG_WIDTH 317.0
#define PIXY_IMG_HEIGHT 209.0
// 工作台尺寸(单位:cm)
#define WORKSPACE_W 21.0
#define WORKSPACE_H 15.0
// 工作台中心在机械臂坐标系下的位置(单位:cm)
#define WORKSPACE_OX 15.0
#define WORKSPACE_OY 1.5
#define WORKSPACE_OZ -7.0
// 物块的尺寸(单位cm)
#define CUBIC_SIZE 2.5
// 颜色签名
#define BLOCK_SIGNATURE_RED 1
#define BLOCK_SIGNATURE_GREEN 2
#define BLOCK_SIGNATURE_BLUE 3

// 物块稳定性判断条件
#define BLOCK_STATIC_MAX_DIS 1.0 // 物块距离上次最大的距离, 若小于这个值则判断静止 单位cm

// RGB颜色的均值
const float RGB_MEAN[COLOR_NUM][3] = {
  {143.72619048, 111.57142857, 103.36904762},// NONE
  {192.34328358,  49.55223881,  44.3880597}, // RED
  {72.19491525, 129.61016949,  54.1779661},  // GREEN
  {56.09090909, 102.66666667, 110.06060606}, // BLUE
  {112.81632653, 102.18367347,  34.97959184},// YELLOW
};
// RGB颜色的标准差
const float RGB_SIGMA[COLOR_NUM][3] = {
  {4.76183036, 1.29362645, 2.10317086},   // NONE
  {7.91664077, 10.49418258, 10.31873419}, // RED
  {7.76809912, 3.0282117 , 3.32843771},   // GREEN
  {11.3654544 ,  2.95077459,  2.56360771},// BLUE
  {10.22536556,  4.11901251,  2.80298526},// YELLOW
};

typedef struct{
  bool isValid; // 是否有效
  uint8_t signature; // 颜色签名
  float wx; // 工作台坐标系x坐标
  float wy; // 工作台坐标系y坐标
  uint8_t staticFrameCnt; // 物块存在且静止的帧数
  uint8_t trackIdx; // 物块跟踪的ID
}BlockStatus_T;

SoftwareSerial softSerial(SOFT_SERIAL_RX, SOFT_SERIAL_TX); // 软串口
FSARM_ARM4DoF arm; //机械臂对象
Pixy2 pixy; // Pixy2对象-视觉模块
BlockStatus_T maxBlock; // 最大的物块的状态

// 颜色名称
String colorNames[] = {"", "RED", "GREEN", "BLUE", "YELLOW"};
// 物块的目标位置
float blockTargePosi[4][3] = {
  {8, 8, -3.5},  // 红色物块的目标位置
  {8, 12, -3.5}, // 绿色物块的目标位置
  {8, -8, -3.5}, // 蓝色物块的目标位置
  {10, -12, -3.5}, // 黄色物块的目标位置
};

// 创建RGB颜色传感器对象
Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

volatile bool btnPressed = false; // 记录按键是否被按下
float channelRed;   // R通道的颜色
float channelGreen; // G通道的颜色
float channelBlue;  // B通道的颜色 
char logMsgBuffer[100]; // 日志缓冲区
int colorIdx = 0;  // 颜色的ID号

// 正态分布
float normalDistribution(float mean, float sigma, float x){
  return (1.0/sigma*sqrt(2*PI))*pow(E, -(x-mean)*(x-mean)/(2*sigma*sigma));
}

// RGB数值属于改颜色的概率
float probRGBColor(uint8_t colorId, float ch_r, float ch_g, float ch_b){
  // 通道R的概率密度分布
  float pdf_r = normalDistribution(RGB_MEAN[colorId][0], RGB_SIGMA[colorId][0], ch_r);
  // 通道G的概率密度分布
  float pdf_g = normalDistribution(RGB_MEAN[colorId][1], RGB_SIGMA[colorId][1], ch_g);
  // 通道B的概率密度分布
  float pdf_b = normalDistribution(RGB_MEAN[colorId][2], RGB_SIGMA[colorId][2], ch_b);
  // 近似计算概率
  return (pdf_r*1.0) * (pdf_g*1.0) * (pdf_b*1.0);
}

// 预测颜色
uint8_t predictRGBColor(float ch_r, float ch_g, float ch_b){
  // 预测的颜色的ID
  uint8_t predictColorIdx = 0;
  // 概率的最大值
  float maxProb = probRGBColor(colorIdx, ch_r, ch_g, ch_b);
  // 寻找最大概率的颜色
  for(int i_color=1; i_color<COLOR_NUM; i_color+=1){
    float curProb = probRGBColor(i_color, ch_r, ch_g, ch_b);
    if(curProb > maxProb){
      predictColorIdx = i_color;
      maxProb = curProb;
    }
  }
  return predictColorIdx;
}

void printCCCBlockInfo(Block block, float wx, float wy){
  String message = "";
  message += "color: " + colorNames[block.m_signature]+ " > ";
  message += "ws:(" + String(wx, 2) + " , " + String(wy, 2) +") | ";
  message += "cx:" + String(block.m_x, DEC) + " cy:" + String(block.m_y, DEC);
  message += " w:" + String(block.m_width, DEC) + " h:" + String(block.m_height, DEC);
  message += " idx:" + String(block.m_index);
  // 打印日志
  softSerial.println(message);
}

/* 判断物块是否合法 */
bool isBlockLegal(Block block){
  // 根据图像的宽高进行筛选
  if(block.m_width < 20 || block.m_width > 50 || block.m_height < 20 || block.m_height > 50){
      return false;
  }
  if(block.m_signature != colorIdx){
      return false;  
  }
  
  return true;
}

// 补偿色块侧边对识别的影响
void compensateBlock(Block* block){
  // 判断宽高差是否大于一个特定的范围
  if(abs(block->m_width - block->m_height) < 5){
    return;
  }
  if(block->m_width > block->m_height){
    // 水平方向上补偿 
    if(block->m_x < PIXY_IMG_WIDTH/2){
      block->m_x -= (block->m_width - block->m_height);  // 在画面左侧
    }else{
      block->m_x += (block->m_width - block->m_height);  // 在画面右侧
    }
    block->m_width = block->m_height;
  }else{
    // 垂直方向上补偿
    if(block->m_y < PIXY_IMG_HEIGHT/2){
      block->m_y -= (block->m_height - block->m_width); // 在画面上方
    }else{
      block->m_y += (block->m_height - block->m_width); // 在画面下方
    }
    block->m_height = block->m_width;
  }  
}

/* 将色块从像素坐标系转换为工作台坐标系 */
void blockPixel2Workspace(Block block, float* wx, float* wy){
  *wx = (0.5 - (block.m_y / PIXY_IMG_HEIGHT)) * WORKSPACE_H;
  *wy = (0.5 - (block.m_x / PIXY_IMG_WIDTH)) * WORKSPACE_W;
}


// 获取最大的色块
bool getMaxBlock(uint8_t* blockIdx){
    float maxArea = 0;
    bool hasCubic = false;
    if(!pixy.ccc.numBlocks){
      // 画面中没有物块
      return false;
    }
    // 找到面积最大的物块
    for (uint8_t i=0; i<pixy.ccc.numBlocks; i++){
      // 找到面积最大且符合尺寸要求的
      if(pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height > maxArea && isBlockLegal(pixy.ccc.blocks[i])){
        hasCubic = true;
        maxArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height; // 更新最大的面积  
        *blockIdx = i; // 更新索引
      }
    }
    return hasCubic;
}

// 根据跟踪的索引号获取
bool getBlockByTrackIdx(uint8_t trackIdx, uint8_t* blockIdx, BlockStatus_T* maxBlock){
    if(!pixy.ccc.numBlocks){
      return false; // 画面中没有物块
    }
    // 遍历所有的pixy, 根据m_index进行查找
    for (uint8_t i=0; i<pixy.ccc.numBlocks; i++){
        if(pixy.ccc.blocks[i].m_index == trackIdx){
          *(blockIdx) = i;
          return true;
        }
    }
    if(maxBlock->isValid){
      // 根据m_index没找到, 则根据相对位置进行查找
      // 如果距离小于特定得范围, 就认为跟上一个是同一个
      float maxDis = 20; // 20个像素
      float wx;
      float wy;

      for (uint8_t i=0; i<pixy.ccc.numBlocks; i++){
          blockPixel2Workspace(pixy.ccc.blocks[i], &wx, &wy);
          maxDis = sqrt(pow(wx - maxBlock->wx, 2) + pow(wy - maxBlock->wy, 2));
          if(maxDis < maxDis){
            *(blockIdx) = i;
            return true;
          }
      }
    }
    
    return false;
}

// 更新物块的状态
void updateBlockStatus(BlockStatus_T* maxBlock){
  // pixy.ccc.getBlocks(); // 获取色块的信息
  float wx, wy; //物块在工作台上的坐标
  uint8_t blockIdx; //物块的序号

  if(maxBlock->isValid){
    softSerial.println("Old Cubic, trackIdx =" + String(maxBlock->trackIdx, 0));
    // 物块存在
    if(!getBlockByTrackIdx(maxBlock->trackIdx, &blockIdx, maxBlock)){
      // 目标对象不存在
      softSerial.println("no cubic found");
      maxBlock->isValid = false;
      return;
    }
    softSerial.println("Cubic Exist");
    softSerial.println("blockIdx = "+String(blockIdx, DEC));
    compensateBlock(&(pixy.ccc.blocks[blockIdx])); // 物块补偿
    blockPixel2Workspace(pixy.ccc.blocks[blockIdx], &wx, &wy); // 图像坐标转换为工作台坐标系
    softSerial.println("wx = " + String(wx, 2) + ", wy =" + String(wy, 2));
    // 判断是否发生运动
    if(sqrt(pow(wx - maxBlock->wx, 2) + pow(wy - maxBlock->wy, 2)) > BLOCK_STATIC_MAX_DIS){
      softSerial.println("Cubic Moving");
      maxBlock->wx = wx;
      maxBlock->wy = wy;
      maxBlock->staticFrameCnt = 1;
    }else{
      softSerial.println("Cublic Static");
      // 高通滤波
      maxBlock->wx = 0.4*maxBlock->wx + 0.6*wx;
      maxBlock->wy = 0.4*maxBlock->wy + 0.6*wy;
      // 计数+1
      maxBlock->staticFrameCnt += 1;
    }

    // 打印色块信息
    printCCCBlockInfo(pixy.ccc.blocks[blockIdx], maxBlock->wx, maxBlock->wy);
    return;
  }else{
    softSerial.println("New Cubic");
    if(getMaxBlock(&blockIdx)){
      // 存在最大的物块
      maxBlock->isValid = true;
      maxBlock->signature = pixy.ccc.blocks[blockIdx].m_signature;
      maxBlock->staticFrameCnt = 1;
      compensateBlock(&(pixy.ccc.blocks[blockIdx])); // 物块补偿
      blockPixel2Workspace(pixy.ccc.blocks[blockIdx], &wx, &wy); // 图像坐标转换为工作台坐标系
      maxBlock->wx = wx;
      maxBlock->wy = wy;
      maxBlock->trackIdx = pixy.ccc.blocks[blockIdx].m_index;
    }
    return;
  }
}

// 工作台坐标转换为机械臂基坐标系
void transWorkspace2Arm(float wx, float wy, float *ax, float *ay, float *az){
  *ax = wx + WORKSPACE_OX;
  *ay = wy + WORKSPACE_OY;
  *az = WORKSPACE_OZ + CUBIC_SIZE;
}

/* 更新RGB三个通道的颜色 */
void updateRGBValue(){
  colorSensor.setInterrupt(false);  // 开启补光的LED
  delay(60); // 等待60ms
  colorSensor.getRGB(&channelRed, &channelGreen, &channelBlue);// 更新RGB三个通道的值
  colorSensor.setInterrupt(true);  // 关闭补光的LED
}

/* 打印一下RGB数值 */
void logRGBValue(){
  sprintf(logMsgBuffer, "ColorType: %d RGB Value: %d,%d,%d\n", colorIdx, int(channelRed), int(channelGreen), int(channelBlue));
  softSerial.print(logMsgBuffer);
}

/* 初始化RGB灯LED的管脚 */
void initLED(){
  // 设置RGB灯的GPIO
  pinMode(LED_RED_GPIO, OUTPUT);
  pinMode(LED_GREEN_GPIO, OUTPUT);
  pinMode(LED_BLUE_GPIO, OUTPUT);
}

/* 设置LED灯的颜色 */
void setLEDColor(uint8_t colorIdx){
  switch(colorIdx){
    case COLOR_RED:
      analogWrite(LED_RED_GPIO, 255);
      analogWrite(LED_GREEN_GPIO, 0);
      analogWrite(LED_BLUE_GPIO, 0);
      break;
    case COLOR_GREEN:
      analogWrite(LED_RED_GPIO, 0);
      analogWrite(LED_GREEN_GPIO, 255);
      analogWrite(LED_BLUE_GPIO, 0);
      break;
    case COLOR_BLUE:
      analogWrite(LED_RED_GPIO, 0);
      analogWrite(LED_GREEN_GPIO, 0);
      analogWrite(LED_BLUE_GPIO, 255);
      break;
    case COLOR_YELLOW:
      analogWrite(LED_RED_GPIO, 255);
      analogWrite(LED_GREEN_GPIO, 255);
      analogWrite(LED_BLUE_GPIO, 0);
      break;
    default:
      analogWrite(LED_RED_GPIO, 0);
      analogWrite(LED_GREEN_GPIO, 0);
      analogWrite(LED_BLUE_GPIO, 0);
      break;
  }
}

// 处理按键按下的事件
void handleBtnPressed() {
  btnPressed = true;
}





void setup()
{
    softSerial.begin(4800);
    pinMode(BUTTON_GPIO, INPUT_PULLUP); // 按键GPIO上拉输入 
    initLED(); // LED灯GPIO初始化
    // 设置中断捕获条件
    attachInterrupt(digitalPinToInterrupt(BUTTON_GPIO), handleBtnPressed, FALLING);

    // 开启颜色传感器
    if (colorSensor.begin()){
        Serial.println("Found RGB Color Sensor"); // 发现颜色传感器
    }else{
        Serial.println("No RGB Color Sensor Found"); // 没有发现颜色传感器
        while(1){
          tone(BUZZER_GPIO, 400, 100); // 蜂鸣器频率为600HZ
          delay(900);
        } //死循环
    }

    arm.init(); // 机械臂初始化
    arm.home(); // 机械臂回归机械零点
    pixy.init(); // Pixy初始化
    
    maxBlock.isValid = false;
    delay(1000);
    softSerial.print("Arm Start To Grasp Color Cubic\n");
    

}

void loop()
{ 
  if(btnPressed){
    tone(BUZZER_GPIO, 600, 100); // 蜂鸣器频率为600HZ
    // 更新RGB的数值
    updateRGBValue();      
    // 预测颜色ID
    colorIdx = predictRGBColor((float)channelRed, (float)channelGreen, (float)channelBlue);
    logRGBValue();          // 打印RGB的数值
    setLEDColor(colorIdx);    // 修改RGB彩灯的颜色
    delay(200);             // 额外延时200ms (消抖)
    btnPressed = false;  
  }

  softSerial.println("Update Cubic Status");
  pixy.ccc.getBlocks(); // 获取色块的信息
  updateBlockStatus(&maxBlock); // 更新最大物块的信息
  
  // 判断是否要抓取物块
  if (maxBlock.isValid &&  maxBlock.staticFrameCnt >= 10){
    float ax, ay, az;
    
    // 计算物块在机械臂坐标系下的位置
    transWorkspace2Arm(maxBlock.wx, maxBlock.wy, &ax, &ay, &az);
    softSerial.println("Grab Cubic : (" + String(ax, 1)+", " + String(ay, 1) + ", " + String(az, 1)+")");
    
    // 物块抓取
    arm.grab(ax, ay, az, blockTargePosi[maxBlock.signature-1][0], blockTargePosi[maxBlock.signature-1][1], blockTargePosi[maxBlock.signature-1][2]);
    arm.home();
    // 将colorIdx变为None
    colorIdx = COLOR_NONE;
    setLEDColor(colorIdx);
  }
  // 调试需要查看日志, 帧率太高了不好调试
  // delay(1000);
}