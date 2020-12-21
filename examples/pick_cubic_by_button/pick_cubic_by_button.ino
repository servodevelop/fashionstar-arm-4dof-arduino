/**
 * 机械臂通过Pixy2进行物块抓取
 * 通过AD按键模块来选择要抓取的色块的颜色
 * 注: 
 * 1. 白色代表None， 即不抓取
 * 2. 颜色选择, 单次有效
 * 3. AD按键扫描只在机械臂空闲的时候有效.
 * --------------------------
 * 作者: 阿凯|Kyle
 * 邮箱: xingshunkai@qq.com
 * 更新时间: 2020/06/11
 */
#include <Pixy2.h>
#include <Pixy2CCC.h>
#include <SoftwareSerial.h>
#include "FashionStar_Arm4DoF.h"


#define IS_DEBUG 0

// 色块的尺寸
#define CUBIC_SIZE_MIN 30
#define CUBIC_SIZE_MAX 60
 
// 颜色的代码
#define COLOR_NONE 0    // 什么颜色都不是
#define COLOR_RED 1     // 红色
#define COLOR_GREEN 2   // 绿色
#define COLOR_BLUE 3    // 蓝色
#define COLOR_YELLOW 4  // 黄色
#define COLOR_WHITE 5   // 白色
#define COLOR_UNKOWN 6  // 未知

#define KEYBOARD_GPIO_ANALOG 0 // 按键模组的管脚
// 按键模组的ADC采样数值
#define KEYBOARD_ADC_RELEASE 1023  // 没有按键按下的ADC采样值(按键抬起)
#define KEYBOARD_ADC_RED 735    // 红色按键按下时的ADC采样值
#define KEYBOARD_ADC_GREEN 483  // 绿色按键的ADC采样值
#define KEYBOARD_ADC_BLUE 142   // 蓝色按键的ADC采样值
#define KEYBOARD_ADC_YELLOW 0   // 黄色按键的ADC采样值
#define KEYBOARD_ADC_WHITE 290  // 白色按键的ADC采样值 -> 对应NONE
#define KEYBOARD_ADC_CMP_THRESHOLD 20 // ADC采样的数值误差范围

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
#define WORKSPACE_OY 1.0
#define WORKSPACE_OZ -7.0
// 物块的尺寸(单位cm)
#define CUBIC_SIZE 2.5
// 颜色签名
#define BLOCK_SIGNATURE_RED 1
#define BLOCK_SIGNATURE_GREEN 2
#define BLOCK_SIGNATURE_BLUE 3

// 物块稳定性判断条件
#define BLOCK_STATIC_MAX_DIS 1.0 // 物块距离上次最大的距离, 若小于这个值则判断静止 单位cm

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
String colorNames[] = {"NONE", "RED", "GREEN", "BLUE", "YELLOW","WHITE", "UNKOWN"};

// 物块的目标位置
float blockTargePosi[4][3] = {
  {8, 8, -3.5},  // 红色物块的目标位置
  {8, 12, -3.5}, // 绿色物块的目标位置
  {8, -8, -3.5}, // 蓝色物块的目标位置
  {10, -12, -3.5}, // 黄色物块的目标位置
};



char logMsgBuffer[100]; // 日志缓冲区
uint16_t lastAdcValue = KEYBOARD_ADC_RELEASE; // 上一次的ADC采样数值
int colorIdx = COLOR_NONE; // 当前的颜色ID


/*将按键的ADC数值转换为颜色ID号(或者说按键ID)*/
int Keyboard_ADC2ColorId(uint16_t adcValue){
  if(abs(adcValue - KEYBOARD_ADC_WHITE) <= KEYBOARD_ADC_CMP_THRESHOLD){
    return COLOR_NONE;
  }else if (abs(adcValue - KEYBOARD_ADC_RED) <= KEYBOARD_ADC_CMP_THRESHOLD){
    return COLOR_RED;
  }else if (abs(adcValue - KEYBOARD_ADC_GREEN) <= KEYBOARD_ADC_CMP_THRESHOLD){
    return COLOR_GREEN;
  }else if (abs(adcValue - KEYBOARD_ADC_BLUE) <= KEYBOARD_ADC_CMP_THRESHOLD){
    return COLOR_BLUE;
  }else if (abs(adcValue - KEYBOARD_ADC_YELLOW) <= KEYBOARD_ADC_CMP_THRESHOLD){
    return COLOR_YELLOW;
  }else{
    return COLOR_UNKOWN; // 不在已知的ADC数值点位内
  }
      
}


/*按键ADC采样*/
bool Keyboard_Update(){
  uint16_t adcValue; // 采样的ADC数值
  bool btnPressed=false; // 按键切换
  adcValue = analogRead(KEYBOARD_GPIO_ANALOG);

  if(adcValue  != lastAdcValue){
    delay(50); // 等待一会儿软件消抖
    adcValue = analogRead(KEYBOARD_GPIO_ANALOG); // 再次重新采样
  }
  
  if(abs(adcValue - lastAdcValue) > KEYBOARD_ADC_CMP_THRESHOLD){
    // 打印ADC数值
    Serial.println("ADC " + String(adcValue) + " LAST ADC "+String(lastAdcValue));
    // 按键状态发生变动
    if(abs(adcValue - KEYBOARD_ADC_RELEASE) <= KEYBOARD_ADC_CMP_THRESHOLD){
      // 按键抬起
      digitalWrite(13,LOW);  // LED灭
    }else{
      int tmpColorIdx =  Keyboard_ADC2ColorId(adcValue);
      if(tmpColorIdx != COLOR_UNKOWN && tmpColorIdx != COLOR_NONE){
        colorIdx = tmpColorIdx;
        maxBlock.staticFrameCnt = 0;
        btnPressed = true;
      }     
      digitalWrite(13, HIGH); // LED亮
      
    }
  }
  lastAdcValue = adcValue; // 更新最近的ADC采样值
  return btnPressed; 
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
  if(block.m_width < CUBIC_SIZE_MIN || block.m_width > CUBIC_SIZE_MAX || block.m_height < CUBIC_SIZE_MIN || block.m_height > CUBIC_SIZE_MAX){
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
      if(IS_DEBUG){
        // 打印候选的block的信息
        Block *block = &(pixy.ccc.blocks[i]);
        String message = "candi ["  + String(i, DEC) +"] ";
        message += "color: " + colorNames[block->m_signature]+ " > ";
        message += "cx:" + String(block->m_x, DEC) + " cy:" + String(block->m_y, DEC);
        message += " w:" + String(block->m_width, DEC) + " h:" + String(block->m_height, DEC);
        message += " idx:" + String(block->m_index);
        // 打印日志
        softSerial.println(message);
      }
      

      // 找到面积最大且符合尺寸要求的
      if(pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height > maxArea && isBlockLegal(pixy.ccc.blocks[i])){
        hasCubic = true;
        maxArea = pixy.ccc.blocks[i].m_width * pixy.ccc.blocks[i].m_height; // 更新最大的面积  
        *blockIdx = i; // 更新索引
      }
    }
    if(IS_DEBUG){
      if(hasCubic){
        String message = "Has Max Cubic, Index = ["  + String(*blockIdx) +"] ";
        softSerial.println(message);
      }else{
        String message = "No Valid Cubic Found";
        softSerial.println(message);
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
    if(IS_DEBUG){
      softSerial.println("Old Cubic, trackIdx =" + String(maxBlock->trackIdx, 0));
    }
    
    // 物块存在
    if(!getBlockByTrackIdx(maxBlock->trackIdx, &blockIdx, maxBlock)){
      // 目标对象不存在
      if(IS_DEBUG){
        softSerial.println("no cubic found");
      }
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
      if(IS_DEBUG){
        softSerial.println("Cubic Moving");
      }
      
      maxBlock->wx = wx;
      maxBlock->wy = wy;
      maxBlock->staticFrameCnt = 1;
    }else{
      if(IS_DEBUG){
        softSerial.println("Cublic Static");
      }
      
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
    if(IS_DEBUG){
      softSerial.println("New Cubic");
    }
    
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

void setup()
{
    softSerial.begin(4800);
    arm.init(); // 机械臂初始化
    arm.home(); // 机械臂回归机械零点
    pixy.init(); // Pixy初始化
    
    maxBlock.isValid = false;
    delay(1000);
    softSerial.print("Arm Start To Grasp Color Cubic\n");

    // 配置Timer0的Compare 0
    // OCR0A = 0xAF;
    // TIMSK0 |= _BV(OCIE0A);
}

void loop()
{ 
  for(int i=0; i<10; i++){
    if(Keyboard_Update()){
      // 打印日志
      // if(IS_DEBUG){
      softSerial.println("Btn Pressed, color Id = "+String(colorIdx) + " COLOR NAME " + colorNames[colorIdx]);
      // }
      break;
    }
  }
  
  if(IS_DEBUG){
    softSerial.println("Update Cubic Status");
  }
  
  pixy.ccc.getBlocks(); // 获取色块的信息
  updateBlockStatus(&maxBlock); // 更新最大物块的信息
  
  // 判断是否要抓取物块
  if (maxBlock.isValid &&  maxBlock.staticFrameCnt >= 2){
    float ax, ay, az;
    
    // 计算物块在机械臂坐标系下的位置
    transWorkspace2Arm(maxBlock.wx, maxBlock.wy, &ax, &ay, &az);
    // if(IS_DEBUG){
    softSerial.println("Grab Cubic : (" + String(ax, 1)+", " + String(ay, 1) + ", " + String(az, 1)+")");
    // }
    // 物块抓取
    arm.grab(ax, ay, az, blockTargePosi[maxBlock.signature-1][0], blockTargePosi[maxBlock.signature-1][1], blockTargePosi[maxBlock.signature-1][2]);
    arm.home();
    // 将colorIdx变为None
    colorIdx = COLOR_NONE;
  }
  // 调试需要查看日志, 帧率太高了不好调试
  // delay(1000);
}

// SIGNAL(TIMER0_COMPA_vect)
// {
// }