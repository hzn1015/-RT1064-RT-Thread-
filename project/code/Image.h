#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "zf_common_headfile.h"

#define RESULT_ROW 100//高
#define RESULT_COL 130//宽
#define Image_W RESULT_COL
#define Image_H RESULT_ROW
#define x_max RESULT_COL
#define y_max RESULT_ROW
#define PER_IMG     mt9v03x_image//SimBinImage:用于透视变换的图像

#define PointsSize x_max+20 //边线点最大数量
#define PointsDist 5//重采样间距
#define TrackDeviation 26//赛道偏移值，赛道宽度像素值/2

#define CardWidth 15//卡片宽度
extern uint8 Imagecopy[Image_H][Image_W];//备份数据
extern uint8 Imagecopy2[Image_H][Image_W];//备份数据
extern uint8 *ImageUSE[RESULT_ROW][RESULT_COL];//逆透视变换结果图
extern uint16 LeftPoints[PointsSize][2];//左边线坐标
extern uint16 AvargeLeftPoints[PointsSize][2];//滤波后左边线坐标
extern uint16 MiddlePoints[PointsSize][2];//中线坐标

extern RightPointsNum;
extern LeftPointsNum;
extern int RasampleLeftPointsLen;//等距采样后坐标长度
extern int RasampleRightPointsLen;//等距采样后坐标长度
extern uint16 RasampleMiddlePointsLen;//重采样后中线长度

extern uint16 RasampleLeftPoints[PointsSize][2];//等距采样后点集
extern uint16 RasampleRightPoints[PointsSize][2];//等距采样后点集
extern uint16 RightPoints[PointsSize][2];
extern uint16 ResampleMiddlePoints[PointsSize][2];//重采样后中线坐标

extern uint16 LeftAngleMaxPoint[2];//左边线角度最大的点
extern uint16 RightAngleMaxPoint[2];//右边线角度最大的点
extern uint16 LeftAngleMaxPosition;//边线中的第几个点
extern uint16 RightAngleMaxPosition;//边线中的第几个点
extern float LeftAngleMax;
extern float RightAngleMax;
extern uint16 LeftMax_Y_Position;//左边最大y坐标
extern uint16 RightMax_Y_Position;//右边最大y坐标

extern int AvargeLeftPointsNum;//滤波后左边线长度
extern int AvargeRightPointsNum;//滤波后右边线长度

extern uint16 LeftAngleMaxPoint2[2];//左拐点
extern uint16 RightAngleMaxPoint2[2];//右拐点
extern uint16 MiddlePointslen2;//原始左边线坐标
extern uint16 RasampleMiddlePointslen2;//原始右边线坐标
extern uint16 MiddlePoints2[PointsSize][2];//原始左边线坐标
extern uint16 RasampleMiddlePoints2[PointsSize][2];//原始右边线坐标

extern uint16 UpAngularPostion[2];//上角点

extern uint16 LeftUpPoints[PointsSize][2];//原始左上边线坐标
extern uint16 RightUpPoints[PointsSize][2];//原始右上边线坐标

extern uint16 AvargeLeftUpPoints[PointsSize][2];//滤波后左上边线坐标
extern uint16 AvargeRightUpPoints[PointsSize][2];//滤波后右上边线坐标


extern int LeftUpPointsNum;//左边线长度
extern int RightUpPointsNum;//右边线长度

extern int AvargeLeftUpPointsNum;//滤波后左边线长度
extern int AvargeRightUpPointsNum;//滤波后右边线长度

extern float LeftUpAngleMax;
extern float RightUpAngleMax;

extern uint16 LeftUpAngleMaxPoint[2];//左边线角度最大的点
extern uint16 RightUpAngleMaxPoint[2];//右边线角度最大的点
//小车运动状态
extern uint8 MotionStateFlag;//小车巡线状态，分为0常规巡线，1入十字路口，2入圆环

extern uint16 LeftCrossFlag;//左十字标志
extern uint16 RightCrossFlag;//右十字标志
extern uint16 CrossCardFlag;//十字有无卡片标志
int clip(int x, int min, int max);
void dilation(uint8 img0[][RESULT_COL],uint8 img1[][RESULT_COL]);
void ImageBinaryzation(uint8 ** image,uint16 width,uint16 height,uint8 threshold);
void ImageBinaryzation2(uint8 * image,uint16 width,uint16 height,uint8 threshold);
uint8 GetOSTU(uint8 *image,uint16 width,uint16 height);
void ImagePerspective_Init(void);
uint16 FindLetfSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y);
uint16 FindRightSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y);
uint16 LeftGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size);
uint16 RightGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size);
uint16 AverageFilter(uint16 Points[][2],uint16 len,uint16 Out_Points[][2],uint16 *Max_Y,uint8 Kernel);
uint16 ResamplePoints(uint16 InPoints[][2],uint16 len,uint16 OutPoints[][2],uint16 len2,float dist);
void FindAngle(uint16 InPoints[][2],uint16 len ,uint16 OutPoint[2],float *OutAngle,uint16 *OutPosition,uint16 pointsLen);
void FindUpinflexion(uint8 *image,uint16 width,uint16 height);
void LineTreatment(void);
uint8 RampJudge(uint16 LeftPoints[][2],uint16 LeftNum,uint16 RightPoints[][2],uint8 RightNum,uint8 dis);
void ElementJudge();//元素判断

uint8 FindUpPoints(uint8 *image,uint16 width,uint16 height,uint16 In_X,uint16 In_Y,uint16 * out_x,uint16 *out_y,uint8 step);
uint16 FindLeftUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y);
void ImageDrawline(uint8 *image,uint16 width,uint16 height,uint16 Start_X,uint16 Start_Y,uint16 End_X,uint16 End_Y);
uint16 UPLineGainMiddle(uint16 InPoints[][2],uint16 Inlen,uint16 OutPoints[][2],uint16 Dis);
#endif