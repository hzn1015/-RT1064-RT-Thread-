#ifndef _IMAGE_H_
#define _IMAGE_H_

#include "zf_common_headfile.h"

#define RESULT_ROW 100//��
#define RESULT_COL 130//��
#define Image_W RESULT_COL
#define Image_H RESULT_ROW
#define x_max RESULT_COL
#define y_max RESULT_ROW
#define PER_IMG     mt9v03x_image//SimBinImage:����͸�ӱ任��ͼ��

#define PointsSize x_max+20 //���ߵ��������
#define PointsDist 5//�ز������
#define TrackDeviation 26//����ƫ��ֵ�������������ֵ/2

#define CardWidth 15//��Ƭ���
extern uint8 Imagecopy[Image_H][Image_W];//��������
extern uint8 Imagecopy2[Image_H][Image_W];//��������
extern uint8 *ImageUSE[RESULT_ROW][RESULT_COL];//��͸�ӱ任���ͼ
extern uint16 LeftPoints[PointsSize][2];//���������
extern uint16 AvargeLeftPoints[PointsSize][2];//�˲������������
extern uint16 MiddlePoints[PointsSize][2];//��������

extern RightPointsNum;
extern LeftPointsNum;
extern int RasampleLeftPointsLen;//�Ⱦ���������곤��
extern int RasampleRightPointsLen;//�Ⱦ���������곤��
extern uint16 RasampleMiddlePointsLen;//�ز��������߳���

extern uint16 RasampleLeftPoints[PointsSize][2];//�Ⱦ������㼯
extern uint16 RasampleRightPoints[PointsSize][2];//�Ⱦ������㼯
extern uint16 RightPoints[PointsSize][2];
extern uint16 ResampleMiddlePoints[PointsSize][2];//�ز�������������

extern uint16 LeftAngleMaxPoint[2];//����߽Ƕ����ĵ�
extern uint16 RightAngleMaxPoint[2];//�ұ��߽Ƕ����ĵ�
extern uint16 LeftAngleMaxPosition;//�����еĵڼ�����
extern uint16 RightAngleMaxPosition;//�����еĵڼ�����
extern float LeftAngleMax;
extern float RightAngleMax;
extern uint16 LeftMax_Y_Position;//������y����
extern uint16 RightMax_Y_Position;//�ұ����y����

extern int AvargeLeftPointsNum;//�˲�������߳���
extern int AvargeRightPointsNum;//�˲����ұ��߳���

extern uint16 LeftAngleMaxPoint2[2];//��յ�
extern uint16 RightAngleMaxPoint2[2];//�ҹյ�
extern uint16 MiddlePointslen2;//ԭʼ���������
extern uint16 RasampleMiddlePointslen2;//ԭʼ�ұ�������
extern uint16 MiddlePoints2[PointsSize][2];//ԭʼ���������
extern uint16 RasampleMiddlePoints2[PointsSize][2];//ԭʼ�ұ�������

extern uint16 UpAngularPostion[2];//�Ͻǵ�

extern uint16 LeftUpPoints[PointsSize][2];//ԭʼ���ϱ�������
extern uint16 RightUpPoints[PointsSize][2];//ԭʼ���ϱ�������

extern uint16 AvargeLeftUpPoints[PointsSize][2];//�˲������ϱ�������
extern uint16 AvargeRightUpPoints[PointsSize][2];//�˲������ϱ�������


extern int LeftUpPointsNum;//����߳���
extern int RightUpPointsNum;//�ұ��߳���

extern int AvargeLeftUpPointsNum;//�˲�������߳���
extern int AvargeRightUpPointsNum;//�˲����ұ��߳���

extern float LeftUpAngleMax;
extern float RightUpAngleMax;

extern uint16 LeftUpAngleMaxPoint[2];//����߽Ƕ����ĵ�
extern uint16 RightUpAngleMaxPoint[2];//�ұ��߽Ƕ����ĵ�
//С���˶�״̬
extern uint8 MotionStateFlag;//С��Ѳ��״̬����Ϊ0����Ѳ�ߣ�1��ʮ��·�ڣ�2��Բ��

extern uint16 LeftCrossFlag;//��ʮ�ֱ�־
extern uint16 RightCrossFlag;//��ʮ�ֱ�־
extern uint16 CrossCardFlag;//ʮ�����޿�Ƭ��־
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
void ElementJudge();//Ԫ���ж�

uint8 FindUpPoints(uint8 *image,uint16 width,uint16 height,uint16 In_X,uint16 In_Y,uint16 * out_x,uint16 *out_y,uint8 step);
uint16 FindLeftUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y);
void ImageDrawline(uint8 *image,uint16 width,uint16 height,uint16 Start_X,uint16 Start_Y,uint16 End_X,uint16 End_Y);
uint16 UPLineGainMiddle(uint16 InPoints[][2],uint16 Inlen,uint16 OutPoints[][2],uint16 Dis);
#endif