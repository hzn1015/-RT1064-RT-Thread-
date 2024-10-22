#include "Image.h"
#include "math.h"
//��ʱ�޸Ļ���ͼ���С������дѰ�����㷨��ԭ������uint8 Imagecopy[RESULT_ROW][RESULT_COL]
uint8 Imagecopy2[RESULT_ROW][RESULT_COL];//Ԥ����ǰ��ͼƬ
uint8 Imagecopy[RESULT_ROW][RESULT_COL];//��������,��������ͷ��ȡ���ݸ��ǵ�ԭ������
uint8 *ImageUSE[RESULT_ROW][RESULT_COL];//��͸�ӱ任���ͼ


uint16 LeftPoints[PointsSize][2];//ԭʼ���������
uint16 RightPoints[PointsSize][2];//ԭʼ�ұ�������

uint16 AvargeLeftPoints[PointsSize][2];//�˲������������
uint16 AvargeRightPoints[PointsSize][2];//�˲����ұ�������

uint16 RasampleLeftPoints[PointsSize][2];//�Ⱦ������㼯
uint16 RasampleRightPoints[PointsSize][2];//�Ⱦ������㼯
float LeftAngleMax=0;
float RightAngleMax=0;

uint16 LeftAngleMaxPoint[2];//����߽Ƕ����ĵ�
uint16 RightAngleMaxPoint[2];//�ұ��߽Ƕ����ĵ�

uint16 LeftMax_Y_Position=0;//������y�����ֵ
uint16 RightMax_Y_Position=0;//�ұ����y�����ֵ

uint16 LeftAngleMaxPosition=0;//�����еĵڼ�����
uint16 RightAngleMaxPosition=0;//�����еĵڼ�����




int LeftPointsNum=0;//����߳���
int RightPointsNum=0;//�ұ��߳���

int AvargeLeftPointsNum=0;//�˲�������߳���
int AvargeRightPointsNum=0;//�˲����ұ��߳���

int RasampleLeftPointsLen=0;//�Ⱦ���������곤��
int RasampleRightPointsLen=0;//�Ⱦ���������곤��


uint8 MiddleLineFlag=1;//���߱�־�������жϴ��������ȡ���߻��Ǵ��ұ�����ȡ����
uint16 MiddlePoints[PointsSize][2];//��������
uint16 ResampleMiddlePoints[PointsSize][2];//�ز�������������
uint16 RasampleMiddlePointsLen=0;//�ز��������߳���
uint16 MiddlePointsNum=0;//���߳���

/************ʮ��Ԫ�ز���*************************/

uint16 MiddlePoints2[PointsSize][2];//ԭʼ���������
uint16 RasampleMiddlePoints2[PointsSize][2];//ԭʼ�ұ�������

uint16 MiddlePointslen2=0;//ԭʼ���������
uint16 RasampleMiddlePointslen2=0;//ԭʼ�ұ�������

uint16 LeftAngleMaxPoint2[2];//��յ�
uint16 RightAngleMaxPoint2[2];//�ҹյ�

uint16 LeftAngleMaxPointPosition2;//��յ�λ��
uint16 RightAngleMaxPointPosition2;//�ҹյ�λ��

uint16 LeftCrossFlag=0;//��ʮ�ֱ�־
uint16 RightCrossFlag=0;//��ʮ�ֱ�־
uint16 CrossCardFlag=0;//ʮ�����޿�Ƭ��־
/************************************************/

/************Բ��Ԫ��*************************/
uint16 UpAngularPostion[2];

uint16 LeftUpPoints[PointsSize][2];//ԭʼ���ϱ�������
uint16 RightUpPoints[PointsSize][2];//ԭʼ���ϱ�������

uint16 AvargeLeftUpPoints[PointsSize][2];//�˲������ϱ�������
uint16 AvargeRightUpPoints[PointsSize][2];//�˲������ϱ�������

int LeftUpPointsNum=0;//����߳���
int RightUpPointsNum=0;//�ұ��߳���

int AvargeLeftUpPointsNum=0;//�˲�������߳���
int AvargeRightUpPointsNum=0;//�˲����ұ��߳���

float LeftUpAngleMax=0;
float RightUpAngleMax=0;

uint16 LeftUpAngleMaxPoint[2];//����߽Ƕ����ĵ�
uint16 RightUpAngleMaxPoint[2];//�ұ��߽Ƕ����ĵ�

uint16 LeftUpMax_Y_Position=0;//������y�����ֵ
uint16 RightUpMax_Y_Position=0;//�ұ����y�����ֵ

uint16 LeftUpAngleMaxPosition=0;//�����еĵڼ�����
uint16 RightUpAngleMaxPosition=0;//�����еĵڼ�����

uint16 Last_StartAnglePoint[2];//��һ�ν����Ľǵ�
/**********************************************/
//���Ʋ�����Χ����,
int clip(int x, int min, int max) {
	
	if(x>=max)
		x=max;
	else if(x<=min)
		x=min;
    return x;
}
/********3x3ͼ��ʴ**************



*/
void dilation(uint8 img0[][RESULT_COL],uint8 img1[][RESULT_COL])
{
	int dy=-1,dx=-1;
	for(uint16 i=0;i<RESULT_ROW;i++)
		for(uint16 j=0;j<RESULT_COL;j++)
		{
			uint8 max_value=0;
			for(dy=-1;dy<=1;dy++)
				for(dx=-1;dx<=1;dx++)
				{
					if(img0[clip(i+dy,0,y_max-1)][clip(j+dx,0,x_max-1)]>max_value)
					{	
						max_value=img0[clip(i+dy,0,y_max-1)][clip(j+dx,0,x_max-1)];
						//dy=1;//�����˳��ⲿѭ��	
					}
					
				}
			img1[clip(i,0,y_max-1)][clip(j,0,x_max-1)]=max_value;
		
		}


}
//double  ReversePerspectiveMatrix[3][3]={};//��͸�Ӿ���
//ͼ����Ϊһά���飬������Ҫ��Ѱַ��ʽ��ȡ����
//ͼ���ֵ��
//����͸�ӱ任���ͼ��
//��ԭʼ����ת�浽Imagecopy���������ݷ��ʷ�����ͻ
void ImageBinaryzation(uint8 ** image,uint16 width,uint16 height,uint8 threshold)
{
	  uint32 i = 0,j = 0;
		uint8 temp=0;
		for(j = 0; j < height; j ++)
    {
        for(i = 0; i < width; i ++)
        {
					 temp = **(image + j * width + i);
                        // ��ȡ���ص�
            if(temp < threshold)
            {
								Imagecopy[j][i]=0;
               // *(image + j * width + i)=0;
            }
            else
            {
               // *(image + j * width + i)=255;
								Imagecopy[j][i]=255;
            }
        }
    }

}
/*��ֵ��ͼ�񣬴���͸�ӱ任ǰ��ͼ��


*/
void ImageBinaryzation2(uint8 * image,uint16 width,uint16 height,uint8 threshold)
{
	  uint32 i = 0,j = 0;
    uint32 width_index = 0, height_index = 0;
		uint8 temp=0;
		for(j = 0; j < height; j ++)
    {
        for(i = 0; i < width; i ++)
        {
					 temp = *(image + j * width + i);
                        // ��ȡ���ص�
            if(temp <=threshold)
            {
								Imagecopy[j][i]=0;
               // *(image + j * width + i)=0;
            }
            else
            {
               // *(image + j * width + i)=255;
								Imagecopy[j][i]=255;
            }
        }
    }

}
//��򷨶�ֵ��ͼ��
//uint8 GetOSTU(uint8 Image[160][128])
uint8 GetOSTU(uint8 *image,uint16 width,uint16 height)
{ 
  int16 i,j; 
  uint32 Amount = 0; 
  uint32 PixelBack = 0; 
  uint32 PixelIntegralBack = 0; 
  uint32 PixelIntegral = 0; 
  int32 PixelIntegralFore = 0; 
  int32 PixelFore = 0; 
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; //����??��?2?
  int16 MinValue, MaxValue; // ��?D??��?��?��  ��?�䨮?��?��?��
  uint8 Threshold = 0;      // ?D?��
  uint8 HistoGram[256]={0};     // ?��?��0-255?����?��?        
 
//  for (j = 0; j < 256; j++)
//	HistoGram[j] = 0;
  
  for (j = 0; j < height; j++) 
  { 
    for (i = 0; i <width; i++) 
    { 
      HistoGram[*(image + j * width + i)]++; //��3???��?��???D????��???��y
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ;
      
  if (MaxValue == MinValue)     return MaxValue;         //��????D??��D��?????��? ?������??��?�䨮?��?��?��
  if (MinValue + 1 == MaxValue)  return MinValue;        //��????D??��D��?????��? ?������??��?D??��?��?��
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ????������y
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];   //?��?��????��?��y
    PixelFore = Amount - PixelBack;         //��3?��????��?��y
    OmegaBack = (double)PixelBack / Amount;//?��?��????�㨴��?����
    OmegaFore = (double)PixelFore / Amount;//��3?��????�㨴��?����
    PixelIntegralBack += HistoGram[j] * j;  //?��?��?��?��?��
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//��3?��?��?��?��
    MicroBack = (double)PixelIntegralBack / PixelBack;   //?��?��?��?���㨴��?����
    MicroFore = (double)PixelIntegralFore / PixelFore;   //��3?��?��?���㨴��?����
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//????����??��?2?
    if (Sigma > SigmaB)                    //?
		{
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return (Threshold);                        //����??��????D?��
}

//͸�ӱ任��ʼ������������͸�ӱ任�����������ӳ��
void ImagePerspective_Init(void) {
 
    static uint8_t BlackColor = 0;
	double ReversePerspectiveMatrix[3][3] ={{0.481187,-0.328070,28.580165},{-0.010185,0.182906,10.880093},{-0.000377,-0.003722,0.717862}};
		//double ReversePerspectiveMatrix[3][3] ={{0.561486,-0.432066,29.643302},{-0.016025,0.209152,5.277625},{-0.000594,-0.004894,0.802194}};
	//double ReversePerspectiveMatrix[3][3] ={{0.532986,-0.381048,23.690318},{-0.021102,0.172902,8.655215},{-0.000314,-0.004475,0.700313}};
//float ReversePerspectiveMatrix[3][3] ={{0.467189,-0.341648,23.704500},{-0.038637,0.170273,7.251533},{-0.000597,-0.004012,0.681166}};
//    double ReversePerspectiveMatrix[3][3] = {          //114w*100h
//            { 0.431597, -0.363123, 27.140875},
//						{0.018130, 0.075770,14.398831 },
//						{ 0.000235,-0.003548,0.526885 },};
    for (int i = 0; i < RESULT_COL ;i++) {
        for (int j = 0; j < RESULT_ROW ;j++) {
            int local_x = (int) ((ReversePerspectiveMatrix[0][0] * i
                    + ReversePerspectiveMatrix[0][1] * j + ReversePerspectiveMatrix[0][2])
                    / (ReversePerspectiveMatrix[2][0] * i + ReversePerspectiveMatrix[2][1] * j
                            + ReversePerspectiveMatrix[2][2]));
            int local_y = (int) ((ReversePerspectiveMatrix[1][0] * i
                    + ReversePerspectiveMatrix[1][1] * j + ReversePerspectiveMatrix[1][2])
                    / (ReversePerspectiveMatrix[2][0] * i + ReversePerspectiveMatrix[2][1] * j
                            + ReversePerspectiveMatrix[2][2]));
            if (local_x>= 0&& local_y >= 0 && local_y < MT9V03X_H && local_x < MT9V03X_W)
						{
                ImageUSE[j][i] = &PER_IMG[local_y][local_x];
            }
            else {
                *ImageUSE[j][i] = 255;          //&PER_IMG[0][0];
            }
 
        }
    }
 
}

/*****��Ȩƽ���˲�****
����������㼯�����ȣ��˲��˴�С
�����˲���㼯����

***/

uint16 AverageFilter(uint16 Points[][2],uint16 len,uint16 Out_Points[][2],uint16 *Max_Y,uint8 Kernel)
{
	uint8 half_size=0;
	uint16 x=0,y=0;
	uint16 size=0;//�����˲���㳤��
	uint16 min_y=y_max;
	if(Kernel%2==0)
		Kernel=Kernel+1;
	half_size=Kernel/2;
	for(uint16 i=0;i<len;i++)
	{
		x=0;
		y=0;
		for(int j=-half_size;j<half_size+1;j++)
		{
			x=x+Points[clip(i+j,0,len-1)][0]*(half_size+1-abs(j));
			y=y+Points[clip(i+j,0,len-1)][1]*(half_size+1-abs(j));
		}
		x=x/((half_size+1)*(half_size+1));
		y=y/((half_size+1)*(half_size+1));
		x=clip(x,0,x_max-1);
		y=clip(y,0,y_max-1);	
		
		if(y<min_y)
		{
			min_y=y;//������Сֵ
		}
		Out_Points[size][0]=x;
		Out_Points[size][1]=y;
		size++;
	}
	*Max_Y=y_max-min_y;
	return size;
}

//Ѱ�������
//ʵ��Ѱ��һ�α���Ϊ5us
uint16 FindLetfSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//��ʼ�����꣬��ͼ�����һ���е㿪ʼ
	uint16 Initiation_Y=height-5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 Point[2]={0};//��ʼ������
	uint16 LeftPointsLen=0;//����߳���
	//����ʼ���Ȳ���
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//���ڱ�����ʼ������ֲ���ɫ���򣨿�Ƭ��
	if(*(image + Initiation_Y * width + Initiation_X)==0||((*(image + Initiation_Y * width + Initiation_X)==255)&&((*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0||*(image + Initiation_Y * width +  clip(Initiation_X-CardWidth,0,x_max-1))==0))))//����ײ������Ǻ�ɫ����������Ѱ������
	{
		Initiation_X=Initiation_X+width/4;//�����ұ�Ѱ���е�
		if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width +  clip(Initiation_X+CardWidth,0,x_max-1))==0))//����ұ��ķ�֮����Ҳ�Ǻ�ɫ���������Ѱ��
		{
			Initiation_X=Initiation_X-width/2;//�����Ѱ���е�
			if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width +  clip(Initiation_X-CardWidth,0,x_max-1))==0))//�������ķ�֮һ��Ҳ�Ǻ�ɫ��������
		{
			return 0;	
		}
		}
	}
	
	for(x=Initiation_X;x>0;x--)
	{
		temp=*(image + Initiation_Y * width + x);//Ѱ�ұ���λ��
		if(temp==0)
		{
			Point[0]=x+1;
			Point[1]=Initiation_Y;//��ȡ������ʼ��
			break;
		}
	}
	x=Point[0];
	y=Point[1];
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//�Ѿ���ͼ��߽��˳�
		if(*(image + (y-1)* width + x)!=0)
			{
				if(*(image + (y-1)* width + x-1)!=0)
				{
					if(*(image + (y)* width + x-1)==0)
					{
						x=x-1;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
					else
					{
						x=x-1;
						y=y+1;
						Point[0]=x;
						Point[1]=y;
					}
				}
				else
				{
						y=y-1;
						Point[0]=x;
						Point[1]=y;
			
				}
			}
		else
		{
			if(*(image + (y)* width + x+1)!=0)
			{
				if(*(image + (y-1)* width + x+1)!=0)
				{
					x=x+1;
					y=y-1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					x=x+1;
					Point[0]=x;
					Point[1]=y;
				}
			}
			else
			{
				if(*(image + (y+1)* width + x+1)!=0)
				{
					x=x+1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					*(image + (y)* width + x)=0;
					y=y+1;
				}
			
			}
		}
	Point[0]=clip(Point[0],0,x_max-1);
	Point[1]=clip(Point[1],0,y_max-1);	
	LeftPoints[LeftPointsLen][0]=Point[0];
	LeftPoints[LeftPointsLen][1]=Point[1];	
	LeftPointsLen++;
		if(LeftPointsLen>=PointsSize)//���������������Խ��
			return LeftPointsLen-1;
	}
	return LeftPointsLen;
}

/*������߸���б�ʻ������
���������ߵ㼯�ϣ�������߳���,OffetValueƫ��ֵ,����
��������߳���


*/
uint16 LeftGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size)
{
	float dy=0,dx=0;
	int x=0,y=0;
	uint16 MiddlePointsLen=0;//���߳���
		for(uint16 i=0;i<len;i++)
		{
			dy=(Points[clip(i+Step_size,0,len-1)][1]-Points[clip(i-Step_size,0,len-1)][1]);
			dx=(Points[clip(i+Step_size,0,len-1)][0]-Points[clip(i-Step_size,0,len-1)][0]);
			float dn=sqrt(dx*dx+dy*dy);
			dx=dx/dn;
			dy=dy/dn;

				x=(Points[i][0])-OffetValue*dy;
				y=(Points[i][1])+OffetValue*dx;	
				x=clip(x,0,x_max-1);
				y=clip(y,0,y_max-1);;
			MiddlePoints[MiddlePointsLen][0]=x;
			MiddlePoints[MiddlePointsLen][1]=y;
			MiddlePointsLen++;
		}
		
		return MiddlePointsLen;

}

/*
���ұ��߻�ȡ����


*/
uint16 RightGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size)
{
	float dy=0,dx=0;
	int x=0,y=0;
	uint16 MiddlePointsLen=0;//���߳���
		for(uint16 i=0;i<len;i++)
		{
			dy=(Points[clip(i+Step_size,0,len-1)][1]-Points[clip(i-Step_size,0,len-1)][1]);
			dx=(Points[clip(i+Step_size,0,len-1)][0]-Points[clip(i-Step_size,0,len-1)][0]);
			float dn=sqrt(dx*dx+dy*dy);
			dx=dx/dn;
			dy=dy/dn;

				x=(Points[i][0])+OffetValue*dy;
				y=(Points[i][1])-OffetValue*dx;	
				x=clip(x,0,x_max-1);
				y=clip(y,0,y_max-1);
			MiddlePoints[MiddlePointsLen][0]=x;
			MiddlePoints[MiddlePointsLen][1]=y;
			MiddlePointsLen++;
		}
		
		return MiddlePointsLen;

}
/*�ز���ʹ���ڵ����߾������
����������㼯������㼯���ȣ�����㼯������㼯���ȣ����ڵ����

����ֵ������㼯����
*/
uint16 ResamplePoints(uint16 InPoints[][2],uint16 len,uint16 OutPoints[][2],uint16 len2,float dist)
{
	int remain=0,num=0;//remainΪ���뻺��
	for(int i=0;i<len;i++)
	{
		float x0=InPoints[clip(i,0,len-1)][0];
		float y0=InPoints[clip(i,0,len-1)][1];
		float dx=InPoints[clip(i+1,0,len-1)][0]-x0;
		float dy=InPoints[clip(i+1,0,len-1)][1]-y0;
		float dn=sqrt(dx*dx+dy*dy);
		dx=dx/dn;
		dy=dy/dn;
		while(remain<dn &&num<len2)
		{
			x0+=dx*remain;
			y0+=dy*remain;
			x0=clip(x0,0,x_max-1);
			y0=clip(y0,0,y_max-1);
			OutPoints[num][0]=x0;
			OutPoints[num][1]=y0;
			num++;
			dn-=remain;
			remain=dist;
		}
		remain-=dn;
	}

	return num;
}

/*Ѱ�ҽǶ�ֵ���ĵ�



ȡǰ������pointsLen/2�����������Ƕ�
*/
void FindAngle(uint16 InPoints[][2],uint16 len ,uint16 OutPoint[2],float *OutAngle,uint16 *OutPosition,uint16 pointsLen)
{
	float angle_max=0;
	float angle=0;
	uint16 x=0,y=0;
	if (len<5)//ǰ��ĵ���Ե�
		return;
	for(uint16 i=0;i<len-1;i++)
	{
		if(InPoints[clip(i+pointsLen,0,len-1)][0]>=x_max-5||InPoints[clip(i+pointsLen,0,len-1)][0]<=5)
			continue;//���Թ��ڿ������ߵĵ�
		float dy1=InPoints[clip(i+pointsLen,0,len-1)][1]-InPoints[clip(i,0,len-1)][1];
		float dx1=InPoints[clip(i+pointsLen,0,len-1)][0]-InPoints[clip(i,0,len-1)][0];
		float dy2=InPoints[clip(i,0,len-1)][1]-InPoints[clip(i-pointsLen,0,len-1)][1];
		float dx2=InPoints[clip(i,0,len-1)][0]-InPoints[clip(i-pointsLen,0,len-1)][0];
		
		float dn1=sqrt(dx1*dx1+dy1*dy1);
		float dn2=sqrt(dx2*dx2+dy2*dy2);
		if(dn2==0)
			continue;
		float c1 = dx1 / dn1;
    float s1 = dy1 / dn1;
    float c2 = dx2 / dn2;
    float s2 = dy2 / dn2;
		
		angle=atan2f(c1 * s2 - c2 * s1, c2 * c1 + s2 * s1)*180/3.1415926;
//		if((dx2*dx1+dy1*dy2)==0)
//			angle=90;
//		else
//		{
		//	angle=atan2((dx1*dy2-dx2*dy1),(dx2*dx1+dy1*dy2))*180/3.1415926;
//		}
		if(abs(angle)>abs(angle_max))
			{
				angle_max=abs(angle);
				x=InPoints[clip(i,0,len-1)][0];
				y=InPoints[clip(i,0,len-1)][1];
				*OutPosition=i;
			}
	}
	
	OutPoint[0]=x;
	OutPoint[1]=y;
	
	*OutAngle=angle_max;
	
}

//Ѱ���ұ���
uint16 FindRightSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//��ʼ�����꣬��ͼ�����һ���е㿪ʼ
	uint16 Initiation_Y=height-5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 Point[2]={0};//��ʼ������
	uint16 RightPointsLen=0;//�ұ��߳���
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//���ڱ�����ʼ������ֲ���ɫ���򣨿�Ƭ��
	if(*(image + Initiation_Y * width + Initiation_X)==0||((*(image + Initiation_Y * width + Initiation_X)==255)&&((*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0||*(image + Initiation_Y * width + clip(Initiation_X-CardWidth,0,x_max-1))==0))))//����ײ������Ǻ�ɫ����������Ѱ������
	{
		Initiation_X=Initiation_X-width/4;//�������Ѱ���е�
		if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width + clip(Initiation_X-CardWidth,0,x_max-1))==0))//����ұ��ķ�֮����Ҳ�Ǻ�ɫ���������Ѱ��
		{
			Initiation_X=Initiation_X+width/2;//���ұ�Ѱ���е�
			if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0))//�������ķ�֮һ��Ҳ�Ǻ�ɫ��������
		{
			return 0;	
		}
		}
	}
	//Ѱ����ʼ��
	for(x=Initiation_X;x<width;x++)
	{
		temp=*(image + Initiation_Y * width + x);//Ѱ�ұ���λ��
		if(temp==0)
		{
			Point[0]=x-1;
			Point[1]=Initiation_Y;//��ȡ������ʼ��
			break;
		}
	}
	x=Point[0];
	y=Point[1];
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max-1)
			break;//�Ѿ���ͼ��߽��˳�
		if(*(image + (y-1)* width + x)!=0)//ǰ��һ������
			{
				if(*(image + (y-1)* width + x+1)!=0)
				{
					if(*(image + (y)* width + x+1)==0)
					{
						x=x+1;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
					else
					{
						x=x+1;
						y=y+1;
						Point[0]=x;
						Point[1]=y;
					}
				}
				else
				{
						y=y-1;
						Point[0]=x;
						Point[1]=y;
			
				}
			}
		else
		{
			if(*(image + (y)* width + x-1)!=0)
			{
				if(*(image + (y-1)* width + x-1)!=0)
				{
					x=x-1;
					y=y-1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					x=x-1;
					Point[0]=x;
					Point[1]=y;
				}
			}
			else
			{
				if(*(image + (y+1)* width + x-1)!=0)
				{
					x=x-1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					*(image + (y)* width + x)=0;
					y=y+1;
				}
			
			}
			
		}
	Point[0]=clip(Point[0],0,x_max-1);
	Point[1]=clip(Point[1],0,y_max-1);	
	RightPoints[RightPointsLen][0]=Point[0];
	RightPoints[RightPointsLen][1]=Point[1];	
	RightPointsLen++;
	if(RightPointsLen>=PointsSize)//���������������Խ��
			return RightPointsLen-1;
	
	}
	return RightPointsLen;
}


//Ѱ���ϱ���
void FindUpinflexion(uint8 *image,uint16 width,uint16 height)
{
	int x=0,y=0;
	//��ʼ�����꣬��ͼ���һ���е㿪ʼ
	uint16 Initiation_Y=5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 PointX1=0;//��ʼ������
	uint16 PointX2=0;//��ʼ������
	uint16 LastPointX1=0;//��һ�ε�����
	uint16 LastPointX2=0;//��ʼ������
	//Ѱ����ʼ��
	
	MiddlePointslen2=0;
	RasampleMiddlePointslen2=0;
	uint8 leftflag=0,rightflag=0,startFlag=0;
//	for(uint16 i=1;i<height;i++)//��������Ѱ�ҿ�ʼ��
//	{
//		if(*(image + i * width + Initiation_X)==255&&*(image + (i-1) * width + Initiation_X)==255)//�ɺڱ��
//			Initiation_Y=i;
//	}
	if(*(image + Initiation_Y * width + Initiation_X)==0)//����ײ������Ǻ�ɫ����������Ѱ������
	{
		Initiation_X=Initiation_X+width/4;//�����ұ�Ѱ���е�
		if(*(image + Initiation_Y * width + Initiation_X)==0)//����ұ��ķ�֮����Ҳ�Ǻ�ɫ���������Ѱ��
		{
			Initiation_X=Initiation_X-width/2;//�����Ѱ���е�
//			if(*(image + Initiation_Y * width + Initiation_X)==0)//�������ķ�֮һ��Ҳ�Ǻ�ɫ��������
//		{
//			
//		}
		}
	}
	while(1)
	{
		if(y>=width)
			break;
	for(x=Initiation_X;x>0;x--)//Ѱ����߽߱��
	{
		temp=*(image + y * width + x);//Ѱ�ұ���λ��
		if(temp==0&&*(image + y * width + x+1)==255)
		{
			PointX1=x+1;
			break;
		}
	}
	if(x==0)//��ʾ���û���ҵ���
	{
		leftflag=1;
	}
		for(x=Initiation_X+1;x<width;x++)//Ѱ����߽߱��
	{
		temp=*(image + y * width + x);//Ѱ�ұ���λ��
		if(temp==0&&*(image + y * width + x-1)==255)
		{
			PointX2=x-1;
			break;
		}
	}
	if(x==width)//��ʾ�Ҳ�û���ҵ���
	{
		rightflag=1;
	}
	if(startFlag==1)
	{	
		if(leftflag==0&&rightflag==0)//�Ҳ��е����û��
		{
			
			if(abs(PointX2-LastPointX2)<=10&&abs(PointX1-LastPointX1)<=10)
			{
				Initiation_X=(PointX1+PointX2)/2;
				MiddlePoints2[MiddlePointslen2][0]=(PointX1+PointX2)/2;
				MiddlePoints2[MiddlePointslen2][1]=y;	
				MiddlePointslen2++;
				LastPointX1=PointX1;
				LastPointX2=PointX2;
			}
			
		}
		if(leftflag!=1&&rightflag==1)//����е��Ҳ�û��
		{
				Initiation_X=PointX1+TrackDeviation;
		}
		if(leftflag==1&&rightflag!=1)//��ʾ���඼�е�
		{	
			//if(abs(PointX2-LastPointX2)<=10)//�ж�ǰ����������������Ƿ����
			Initiation_X=PointX2-TrackDeviation;
			
		}
		if(leftflag==1&&rightflag==1)//��ʾ���඼�հ�.�����Ѳ��
		{
			break;
		}
	}
	else
	{
		if(leftflag==0&&rightflag==0)//�Ҳ��е����û��
		{
			 LastPointX1=PointX1;
			 LastPointX2=PointX2;
				if(abs(PointX2-LastPointX2)<=10&&abs(PointX1-LastPointX1)<=10)
			 startFlag=1;
		}
		if(leftflag==1&&rightflag==0)//�Ҳ��е����û��
		{
			 LastPointX2=PointX2;
		}
		if(leftflag==0&&rightflag==1)//�Ҳ��е����û��
		{
			 LastPointX1=PointX1;
		}
		

	}

			leftflag=0;//�����־λ
			rightflag=0;
			
			y++;
				if(MiddlePointslen2>=y_max)
			MiddlePointslen2=MiddlePointslen2-1;
	}
	
	//�ز���
	RasampleMiddlePointslen2=ResamplePoints(MiddlePoints2,MiddlePointslen2,RasampleMiddlePoints2,MiddlePointslen2,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
	
}
//*************************Ѱ�����ϱ���



/*****************************/
uint16 FindLeftUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//��ʼ�����꣬��ͼ�����һ���е㿪ʼ
	uint16 Initiation_Y=0;
	uint16 Initiation_X=0;
	uint8 temp=0;
	uint16 Point[2]={0};//��ʼ������
	uint16 LeftUpPointsLen=0;//���ϱ��߳���
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//Ѱ����ʼ��
	x=Initiation_X;
	y=Initiation_Y;
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//�Ѿ���ͼ��߽��˳�
		if(*(image + clip(y,0,y_max-1)* width + x+1)!=0)//ǰ��һ������
			{
				if(*(image + clip(y-1,0,y_max-1)* width + x+1)!=0)
				{
					if(*(image + clip(y-1,0,y_max-1)* width + x)==0)
					{
						x=x+1;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
					else
					{
						x=x;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
				}
				else
				{
						x=x+1;
						Point[0]=x;
						Point[1]=y;
			
				}
			}
		else
		{
			if(*(image + clip(y+1,0,y_max-1)* width + x)!=0)//����ǲ�����
			{
				if(*(image + clip(y+1,0,y_max-1)* width + x+1)!=0)//ǰ���ұ�
				{
					x=x+1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
			}
			else
			{
				if(*(image + clip(y+1,0,y_max-1)* width + x-1)!=0)
				{
					x=x-1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					*(image + clip(y,0,y_max-1)* width + x)=0;
					y=y+1;
				}
			
			}
			
		}
	Point[0]=clip(Point[0],0,x_max-1);
	Point[1]=clip(Point[1],0,y_max-1);	
	LeftUpPoints[LeftUpPointsLen][0]=Point[0];
	LeftUpPoints[LeftUpPointsLen][1]=Point[1];	
	LeftUpPointsLen++;
	if(LeftUpPointsLen>=PointsSize)//���������������Խ��
			return PointsSize-1;
	
	} 
	return LeftUpPointsLen;
}

//*************************Ѱ�����ϱ���



/*****************************/
uint16 FindRightUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//��ʼ�����꣬��ͼ�����һ���е㿪ʼ
	uint16 Initiation_Y=0;
	uint16 Initiation_X=0;
	uint8 temp=0;
	uint16 Point[2]={0};//��ʼ������
	uint16 RightUpPointsLen=0;//���ϱ��߳���
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//Ѱ����ʼ��
	x=Initiation_X;
	y=Initiation_Y;
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//�Ѿ���ͼ��߽��˳�
		if(*(image + (y)* width + x-1)!=0)//ǰ��һ������
			{
				if(*(image + (y-1)* width + x-1)!=0)
				{
					if(*(image + (y-1)* width + x)==0)
					{
						x=x-1;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
					else
					{
						x=x;
						y=y-1;
						Point[0]=x;
						Point[1]=y;
					}
				}
				else
				{
						x=x-1;
						Point[0]=x;
						Point[1]=y;
			
				}
			}
		else
		{
			if(*(image + (y+1)* width + x)!=0)//����ǲ�����
			{
				if(*(image + (y+1)* width + x-1)!=0)//ǰ���ұ�
				{
					x=x-1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
			}
			else
			{
				if(*(image + (y+1)* width + x+1)!=0)
				{
					x=x+1;
					y=y+1;
					Point[0]=x;
					Point[1]=y;
				}
				else
				{
					*(image + (y)* width + x)=0;
					y=y+1;
				}
			
			}
			
		}
	Point[0]=clip(Point[0],0,x_max-1);
	Point[1]=clip(Point[1],0,y_max-1);	
	RightUpPoints[RightUpPointsLen][0]=Point[0];
	RightUpPoints[RightUpPointsLen][1]=Point[1];	
	RightUpPointsLen++;
	if(RightUpPointsLen>=PointsSize)//���������������Խ��
			return PointsSize-1;
	
	} 
	return RightUpPointsLen;
}


/*****���Ϸ������******
����������ͼ��ͼ���ȣ�ͼ��߶ȣ���ʼ��x���꣬��ʼ��y���꣬���x�����y������(����Ϊ����)
��������ɨ��Ѱ��
*/
uint8 FindUpPoints(uint8 *image,uint16 width,uint16 height,uint16 In_X,uint16 In_Y,uint16 * out_x,uint16 *out_y,uint8 step)
{
	for(uint16 i=In_X;i<width-1;i=i+step)//x�������
	{
		for(int j=In_Y;j>=0;j--)
		{
			if(*(image + clip(j,0,height-1) * width + clip(i,0,width-1))==0&&*(image + clip(j+1,0,height-1)* width + clip(i,0,width-1))==255&&*(image + clip(j-1,0,height-1)* width + clip(i,0,width-1))==0)//�����������ҵ���ɫ�߽��
			{
				//��һ���жϣ������ܵ��б�ʽ����ж�
				* out_x=i;
				* out_y=clip(j+1,0,height-1);
				if(* out_y>=y_max)
					* out_y=y_max-1;
				return 1;
			}
		}
	}
	* out_x=0;
	* out_y=0;
	return 0;
}
/***********ͼ����********************

ʹ������ֱ�Ӳ�ͼ�񣬲�ʹ����С���˷�
*************************************/

void ImageDrawline(uint8 *image,uint16 width,uint16 height,uint16 Start_X,uint16 Start_Y,uint16 End_X,uint16 End_Y)
{
	float k=0;
	int x=0,y=0,y_1=0,Last_y=0;
	int Max_X=0,Max_Y=0,Min_X=0,Min_Y=0,start_y=0,start_x=0;
	int err_y=0;
	if(Start_X!=End_X)
	{
		k=10000*((short)Start_Y-(short)End_Y)/((short)Start_X-(short)End_X);//����10000��������Ϊ�������Ͳ�ƥ�䵼�½ض�
		if(Start_X>End_X)
		{
			Max_X=Start_X;
			Min_X=End_X;
			start_x=End_X;
			start_y=End_Y;
			
		}
		else
		{
			Max_X=End_X;
			Min_X=Start_X;
			start_x=Start_X;
			start_y=Start_Y;
		}
	Last_y=start_y;
	for(x=Min_X;x<=Max_X;x++)
	{
		y=k*(x-start_x)/10000+start_y;
		err_y=(y-Last_y);
		//y=y/10000;
		if(y>y_max-1)
		{
			y=y_max-1;
		}
		else if(y<0)
			y=0;
		
		*(image+(uint16)y*width+(uint16)x)=0;//��ͼ���Ӧ���겹Ϊ��ɫ
		if(abs(err_y)>1)
		{	
			for(int i=abs(err_y)-1;i>=0;i--)
			{
				
				if(err_y>0)
				{
					y_1=Last_y;
					*(image+(uint16)clip(y_1+i,0,y_max-1)*width+(uint16)x)=0;//��ͼ���Ӧ���겹Ϊ��ɫ
				}
				else
				{
					y_1=Last_y;
					*(image+(uint16)clip(y_1-i,0,y_max-1)*width+(uint16)x)=0;//��ͼ���Ӧ���겹Ϊ��ɫ
					
				}
			}
		}
			Last_y=y;

		
		
		}
	}
	else
	{
		x=Start_X;
		if(Start_Y>End_Y)
		{
			Max_Y=Start_Y;
			Min_Y=End_Y;
			
		}
		else
		{
			Max_Y=End_Y;
			Min_Y=Start_Y;
		}
	for(y=Min_Y;y<=Max_Y;y++)
		{
			*(image+(uint16)y*width+(uint16)x)=0;//��ͼ���Ӧ���겹Ϊ��ɫ
		}
	}

}
//ʮ���ж�
//����1��ʾ��ʮ�֣����򷵻�0
uint8 CrossJudge()
{
	
	//ʮ���ж�
		if(LeftAngleMaxPoint[1]>y_max-30&&RightAngleMaxPoint[1]>y_max-30&&LeftMax_Y_Position<y_max/2&&RightMax_Y_Position<y_max/2)//���߽ǵ�λ��С��30
		{
			if(abs(LeftAngleMax-85)<=15&&abs(RightAngleMax-85)<=15)//�������ǵ����Ƕȱ仯Ϊ90�ȣ���Ϊ����ʮ��
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				//��ʮ������һ���жϣ���������
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				//rt_kprintf("left %d,%d,%d,%d,right:&d,%d,%d,%d,%d,%d\n",lefterr1,lefterr2,lefterr3,lefterr4,righterr1,righterr2,righterr3,righterr4,LeftAngleMaxPosition,RightAngleMaxPosition)        ;                                                                                                                      
				if(lefterr1<5&&lefterr2<10&&lefterr3>5&&lefterr4>15&&righterr1<5&&righterr2<5&&righterr3>5&&righterr4>5)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
				MotionStateFlag=1;//ʮ��
				//ˢ�½Ƕ�
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
		else if(abs(LeftAngleMax-85)<=15&&abs(RightAngleMax-85)>=15)//ֻ���ұ���ֱ�ǣ�ֻ���ұ߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
				MotionStateFlag=1;//ʮ��
				//ˢ�½Ƕ�
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
		else if(abs(LeftAngleMax-85)>=15&&abs(RightAngleMax-85)<=15)//ֻ�������ֱ�ǣ�ֻ����߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
				MotionStateFlag=1;//ʮ��
				//ˢ�½Ƕ�
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
			return 0;
		}
		else if (RightMax_Y_Position>y_max/2&&LeftMax_Y_Position<y_max/2)//��ඪ���Ҳ�û��
		{
			if(abs(RightAngleMax-85)<=15)//ֻ�������ֱ�ǣ�ֻ����߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
			//	rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
					MotionStateFlag=1;//ʮ��
				//ˢ�½Ƕ�
					LeftAngleMax=0;
					RightAngleMax=0;
					return 1;
				}
				return 0;
			}
						return 0;
		}
		else if(LeftMax_Y_Position>y_max/2&&RightMax_Y_Position<y_max/2)
		{
			if(abs(LeftAngleMax-85)<=15)//ֻ���ұ���ֱ�ǣ�ֻ���ұ߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
					int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
					lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
					lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
					lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
					lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
					if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
					{
						MotionStateFlag=1;//ʮ��״̬
						//ˢ�½Ƕ�
						LeftAngleMax=0;
						RightAngleMax=0;
						return 1;
					}
					return 0;
			}
			return 0;
		
		}
		else
		{
			return 0;
		}
}
/************�µ��ж�******************
//�������ұ���,������ٸ���

***************************************/

uint8 RampJudge(uint16 LeftPoints[][2],uint16 LeftNum,uint16 RightPoints[][2],uint8 RightNum,uint8 dis)
{
	uint16 Last_width=0,width=0,num=0,first_width=0;
	if(RightNum>LeftNum)//������̵�����
		num=LeftNum;
	else
		num=RightNum;
	if(RightMax_Y_Position>=y_max-10&&LeftMax_Y_Position>=y_max-10)//���߹ᴩ������Ļ
	{
		//rt_kprintf("A\n");
		
		for(uint16 i=0;i<num;i=i+dis)//�жϿ���Ƿ�һֱ���
		{
			width=abs(LeftPoints[i][0]-RightPoints[i][0]);
			//rt_kprintf("%d\n",width);
			if(i==0)
				first_width=width;
			if(width>=Last_width)
			{
				Last_width=width;
			}
			else
				return 0;
		}
		for(int i=0;i<num;i=i+dis)//�жϱ���б���Ƿ���ȷ���ٻ�����
		{
			if(LeftPoints[i][0]>LeftPoints[clip(i-dis,0,num)][0]||RightPoints[i][0]<RightPoints[clip(i-dis,0,num)][0])
			{
				return 0;
			}
		}
		if(width-first_width>=20)
		{
			return 1;
		}
		else
			return 0;
		//rt_kprintf("B\n");
		
	}
	return 0;

}
/*************�������ж�******************
���룺ͼ����ʼxֵ����ֹxֵ����߳��ȣ��ұ߳��ȣ��ظ�����


*/


uint8 ZebraJudge(uint8 *image,uint16 width,uint16 hight,uint16 Start_X,uint16 End_X,uint16 Start_Line)
{
	uint16 i=0;
	uint8 sum=0;//���������
	if(RightMax_Y_Position>=y_max-10&&LeftMax_Y_Position>=y_max-10)
	{
		for(i=Start_X;i<End_X;i++)
		{
				if(*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i)==0&&*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i-1)==255)
				{
					sum++;
				}
				else if(*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i)==255&&*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i-1)==0)
				{ 
					sum++;
				}
		}
	}
	if(sum>=15)
		return 1;
	else
		return 0;

}
/***********�ϰ����ж�***********

3��������ϰ�������ߣ��ϰ�����м䣬�ϰ������ұ�
*/


uint8 BarrierJudge(uint8 *image,uint16 width,uint16 hight,uint16 Start_X,uint16 End_X,uint16 Start_Line)
{
	int X_white2black[5]={0},X_black2white[5]={0};
	uint8  X_white2black_num=0,X_black2white_num=0;
	uint8 i=0;
	/*�ȼ����ϰ�����***/
	for(i=Start_X;i<End_X;i++)
		{
			
				if(*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i)==0&&*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i-1)==255)//�ҵ��ϰ�����ʼ��
				{
					X_white2black[X_white2black_num]=i;
					X_white2black_num++;
				}
				else if(*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i)==255&&*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i-1)==0)
				{ 
					X_black2white[X_black2white_num]=i;
					X_black2white_num++;
				}
		}
		if(X_white2black_num==1&&X_black2white_num==2)//һ���ױ�ڣ������ڱ�׵ĵ㣨������ʼ�ı߽�㣩
		{
			if(X_white2black[0]-X_black2white[0]>=25&&X_black2white[1]-X_white2black[0]>=12&&X_black2white[1]-X_white2black[0]<=16)//���ϰ�
			{
				BarrierDirection=1;//�ұ��ϰ���
				return 1;
			}
			else if(X_white2black[0]-X_black2white[0]<=12&&X_black2white[1]-X_white2black[0]>=12&&X_black2white[1]-X_white2black[0]<=16)//���ϰ�
			{
				BarrierDirection=0;//����ϰ���
				return 1;
			}
			
		
		}
		return 0;
		
		
	
	

}
/*************ʮ��Ѳ�ߴ���************/
void CrossCardImage()
{
			if(CrossStateFlag==0)
			{
										//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
					if(RightMax_Y_Position>LeftMax_Y_Position)
						MiddleLineFlag=1;
					else if(LeftMax_Y_Position>RightMax_Y_Position)
						MiddleLineFlag=0;
					else 
						MiddleLineFlag=1;
							//�ز���
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
											RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
			}
			else if(CrossStateFlag==1)
			{	
				uint8 flag1=0,flag2=0;
				uint16 Points1[2],Points2[2];
				flag1=FindUpPoints(Imagecopy,Image_W,Image_H,x_max-10,clip(y_max-RightMax_Y_Position-1,0,y_max-10),&Points1[0],&Points1[1],1);
				flag2=FindUpPoints(Imagecopy,Image_W,Image_H,10,clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&Points2[0],&Points2[1],1);
				if(flag1==1&&flag2==1)
				{
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,Points1[0],Points1[1]);//�õ����������
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],RightUpAngleMaxPoint[0],y_max-1);	
					
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,Points2[0],Points2[1]);//�õ����������
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],LeftUpAngleMaxPoint[0],y_max-1);
				}
					LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);//�õ����������
					RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);
		//�����˲�
					AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
					AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
				
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//�ز���
			
			
		
			if(MiddleLineFlag==0)
			{
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
				if(CrossCardFlag==0)
					MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
				else
				{
					MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//��������ȡ����
				}
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	
				RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
				if(CrossCardFlag==0)
					MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
				else
				{
					MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//���ұ�����ȡ����
				}
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
//				if(CrossCardFlag==0)//û�п�Ƭ��
//				{
//					
//					RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,0);
//				}
//				else//�п�Ƭ��
//				{
//					if(LeftCrossFlag==1)//��ʮ�֣��ұ�Ѳ��,����
//						{
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,TrackDeviation);
//						}
//						else if(RightCrossFlag==1)//��ʮ��
//						{
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,-TrackDeviation);
//						}	
//				}
			}
			else if(CrossStateFlag==2)
			{

		
					//�п�Ƭ��
					if(CrossCardFlag==0)//û�п�Ƭ��
					{
											//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
						if(RightMax_Y_Position>LeftMax_Y_Position)
							MiddleLineFlag=1;
						else if(LeftMax_Y_Position>RightMax_Y_Position)
							MiddleLineFlag=0;
						else 
							MiddleLineFlag=1;
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
						RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
								
						if(MiddleLineFlag==0)
						{
							MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else 
						{		MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
					
					}
					else//�п�Ƭ��
					{
						if(LeftCrossFlag==1)//��ʮ�֣��ұ�Ѳ��
						{
							RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
							AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
							RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
				
							MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-15,1);//���ұ�����ȡ����
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//��ʮ��
						{
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);//�õ����������
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
							
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
							
								MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-15,1);//��������ȡ����
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}	
					
					}

			}
			else if(CrossStateFlag==3)
			{
				
				if(CrossCardFlag==0)//û�п�Ƭ��
				{
										//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
					if(RightMax_Y_Position>LeftMax_Y_Position)
						MiddleLineFlag=1;
					else if(LeftMax_Y_Position>RightMax_Y_Position)
						MiddleLineFlag=0;
					else 
						MiddleLineFlag=1;
							//�ز���
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
											RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
				}
				else
				{
					if(RightMax_Y_Position>LeftMax_Y_Position)
						MiddleLineFlag=1;
					else if(LeftMax_Y_Position>RightMax_Y_Position)
						MiddleLineFlag=0;
					else 
						MiddleLineFlag=1;
							//�ز���
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-15,1);//��������ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
						RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-15,1);//���ұ�����ȡ����
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
				
				}
			}
			else if(CrossStateFlag==4)
			{
					if(LeftCrossFlag==1)//��ʮ�֣��ұ�Ѳ��,����
						{
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,RightAngleMaxPoint[0],clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
								AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
								RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
								if(CrossCardFlag==1)
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,1,1);//���ұ�����ȡ����
								else
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//��ʮ��
						{
							
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,LeftAngleMaxPoint[0],clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
								if(CrossCardFlag==1)
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,1,1);//���ұ�����ȡ����
								else
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//���ұ�����ȡ����
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
								
						}	
				}
			else if(CrossStateFlag==5)
			{
				if(LeftCrossFlag==1)//��ʮ�֣��ұ�Ѳ��,����
						{
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,x_max-5,clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
								AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
								RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
				
								if(CrossCardFlag==1)
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//���ұ�����ȡ����
								else
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//��ʮ��
						{
							
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,5,clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//�����˲�
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//��ȡ��߽ǵ�
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
				
									if(CrossCardFlag==1)
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//���ұ�����ȡ����
								else
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//���ұ�����ȡ����
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
								
						}	
				
//				if(CrossCardFlag==0)//û�п�Ƭ��
//				{
//					FindUpinflexion(Imagecopy,Image_W,Image_H);//��ȡԶ��ֱ��
//					RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,0);
//				}
//				else//�п�Ƭ��
//				{
//					if(LeftCrossFlag==1)//��ʮ�֣��ұ�Ѳ��,����
//						{
//							FindUpinflexion(Imagecopy,Image_W,Image_H);//��ȡԶ��ֱ��
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,TrackDeviation);
//						}
//						else if(RightCrossFlag==1)//��ʮ��
//						{
//							FindUpinflexion(Imagecopy,Image_W,Image_H);//��ȡԶ��ֱ��
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,-TrackDeviation);
//						}	
//				}
			}


}
/*******�ж�Բ��******************8

//û�в��Թ������ϳ�����
1��Բ����0����
**/
uint8 CircularRingJudge()
{
	//�ж���Բ��
	if(LeftMax_Y_Position>=y_max-1&&abs(RightAngleMax-85)<=15&&RightMax_Y_Position<=y_max/4)
	{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<5&&righterr2<10&&righterr3>5&&righterr4>15)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
						MotionStateFlag=2;//Բ��
						CircularRingDirection=1;//��Բ��
						//ˢ�½Ƕ�
						LeftAngleMax=0;
						RightAngleMax=0;
						return 1;
				}			
				return 0;
		//��һ���ж�
	}
	else if(RightMax_Y_Position>=y_max-1&&abs(LeftAngleMax-85)<=15&&LeftMax_Y_Position<=y_max/4)
	{
		int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				if(lefterr1<5&&lefterr2<10&&lefterr3>5&&lefterr4>15)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
						MotionStateFlag=2;//ʮ��״̬
						//ˢ�½Ƕ�
						CircularRingDirection=0;//��Բ��
						LeftAngleMax=0;
						RightAngleMax=0;
						return 1;
				}
	
	}
		
	return 0;
}

/****��Բ������********

******/
void LeftCircularRing_Handle()
{
	
	if(CircularRingStateFlag==0)//���뻷��1״̬�����뻷��
	{
		if(RightMax_Y_Position>=y_max-1&&LeftMax_Y_Position==0&&LeftPointsNum==0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=1;//���뻷��״̬2�����뻷��
		}
	}
	else if(CircularRingStateFlag==1)
	{
		if(RightMax_Y_Position>=y_max-1&&LeftPointsNum>0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=2;//���뻷��״̬3��׼��ת��
			CardControlState=1;//�����ҿ�Ƭ�׶Σ��ǵ�ȡ��ע��
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//������������ݣ�ʹ֮���»����ȶ���̬
			AngleSpeed_z=0;
		}
	}
	else if(CircularRingStateFlag==2)
	{
		if(RightMax_Y_Position>=y_max-1&&LeftMax_Y_Position==0&&LeftPointsNum==0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=3;//���뻷��״̬3��׼��ת��
		}
	}
	else if(CircularRingStateFlag==3)
	{
			
		if(LeftMax_Y_Position==0&&RightMax_Y_Position==0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=4;//���뻷��״̬4�����ұ��ߵ�һ�ζ�ʧ
		}
		else if(RightMax_Y_Position>0&&RightMax_Y_Position<y_max/2&&RightPoints[RightPointsNum][0]<=5)
		{
			CircularRingStateFlag=5;//ֱ�ӽ���״̬5
		}
			
	}
	else if(CircularRingStateFlag==4)
	{
		if(RightMax_Y_Position>0&&RightMax_Y_Position<=y_max/2)//����ұ�ֱ��������������߶���һ��
		{
			PreviewDist0=4;//ǰհ����
			CircularRingStateFlag=5;//���뻷��״̬5,��ʾ�Ѿ����뻷��
		}
	}
	else if(CircularRingStateFlag==5)
	{
		if(abs(RightAngleMax-90)<=20)//ֻ�������ֱ�ǣ�ֻ����߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
				//ˢ�½Ƕ�
				LeftAngleMax=0;
				RightAngleMax=0;
				CircularRingStateFlag=6;
				//	PreviewDist0=9;//ǰհ����
					//��ʱ�޸�pid����
				}
			}
	}
	else if(CircularRingStateFlag==6)
	{
		
		if(RightMax_Y_Position==0)//����ұ�ֱ��������������߶���һ��
		{
			PreviewDist0=4;//ǰհ����
			CircularRingStateFlag=7;//���뻷��״̬4�����ұ��ߵ�һ�ζ�ʧ
		}

	}
		else if(CircularRingStateFlag==7)
	{
		if(RightMax_Y_Position>0&&RightMax_Y_Position>=y_max/2&&RightPoints[RightPointsNum][1]<=5)//����ұ�ֱ��������������߶���һ��
		{

			CircularRingStateFlag=8;//�˳�Բ��״̬
		}

	}
	else if(CircularRingStateFlag==8)
	{
		
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);//�õ����������
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);
		//�����˲�
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
		if(LeftMax_Y_Position>=y_max/4&&RightMax_Y_Position>=y_max-35)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=0;//�˳�Բ��״̬
			MotionStateFlag=0;
		}

	}
	
}

/****��Բ������********

******/
void RightCircularRing_Handle()
{
	
	if(CircularRingStateFlag==0)//���뻷��1״̬�����뻷��
	{
		if(LeftMax_Y_Position>=y_max-1&&RightMax_Y_Position==0&&RightPointsNum==0)//������ֱ�������������ұ߶���һ��
		{
			CircularRingStateFlag=1;//���뻷��״̬2�����뻷��
		}
	}
	else if(CircularRingStateFlag==1)
	{
		if(LeftMax_Y_Position>=y_max-1&&RightPointsNum>0)//������ֱ�������������ұ������³���
		{
			CircularRingStateFlag=2;//���뻷��״̬3��׼��ת��
			CardControlState=1;//�����ҿ�Ƭ�׶Σ��ǵ�ȡ��ע��
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//������������ݣ�ʹ֮���»����ȶ���̬
			Angle_return_Target=0;
			AngleSpeed_z=0;
			
		}
	}
	else if(CircularRingStateFlag==2)
	{
		if(LeftMax_Y_Position>=y_max-1&&RightMax_Y_Position==0&&RightPointsNum==0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=3;//���뻷��״̬3��׼��ת��
		}
	}
	else if(CircularRingStateFlag==3)
	{
		
		if(LeftMax_Y_Position==0&&RightMax_Y_Position==0)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=4;//���뻷��״̬4�����ұ��ߵ�һ�ζ�ʧ
		}
		else if(LeftMax_Y_Position>0&&LeftMax_Y_Position<y_max/2&&LeftPoints[LeftPointsNum][0]>=x_max-5)
		{
			CircularRingStateFlag=5;//ֱ�ӽ���״̬5
		}
		
	}
	else if(CircularRingStateFlag==4)
	{
		if(LeftMax_Y_Position>y_max/4)//����ұ�ֱ��������������߶���һ��
		{
			PreviewDist0=4;//ǰհ����
			CircularRingStateFlag=5;//���뻷��״̬5,��ʾ�Ѿ����뻷��
		}
	}
	else if(CircularRingStateFlag==5)
	{
		if(abs(LeftAngleMax-90)<=20)//ֻ�������ֱ�ǣ�ֻ����߽ضϣ�ͬʱ���ұ߻�ȡ����
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//�ǵ�ǰ3����ĺ�����
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//�ضϴ���
			//��С���˼������ֱ��б��	
				{
				//ˢ�½Ƕ�
					LeftAngleMax=0;
					RightAngleMax=0;
					CircularRingStateFlag=6;
					//PreviewDist0=9;//ǰհ����
				}
			}
	}
	else if(CircularRingStateFlag==6)
	{
		
		
		if(LeftMax_Y_Position==0)//������ֱ��������������߶���һ��
		{
			
			CircularRingStateFlag=7;//���뻷��״̬4�����ұ��ߵ�һ�ζ�ʧ
		}
		//ע�ͼǵøĻ�ȥ

	}
		else if(CircularRingStateFlag==7)
	{
		if(LeftMax_Y_Position>0&&LeftMax_Y_Position>=y_max/2&&LeftPoints[LeftPointsNum][1]<=5)//����ұ�ֱ��������������߶���һ��
		{
			turnPD.P=1;
			turnPD.D=0.5;
			CircularRingStateFlag=8;//�˳�Բ��״̬
		}

	}
	else if(CircularRingStateFlag==8)
	{
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);//�õ����������
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);
		//�����˲�
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
		if(LeftMax_Y_Position>=y_max-35&&RightMax_Y_Position>=y_max/4)//����ұ�ֱ��������������߶���һ��
		{
			CircularRingStateFlag=0;//�˳�Բ��״̬
			MotionStateFlag=0;
			CartStateFlag=0;
			CardControlState=0;

		}

	}
	
}
//��Բ����ͼ����
void LeftCircularRing_ImageHandle()
{
	//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
	if(CircularRingStateFlag==0||CircularRingStateFlag==1||CircularRingStateFlag==2)
	{
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//�ز���
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			if(MiddleLineFlag==0)
			{
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
		else if(CircularRingStateFlag==3||CircularRingStateFlag==4)
		{

			uint8 flag=1;
			flag=FindUpPoints(Imagecopy,Image_W,Image_H,5,clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
			if(flag==1)
			{
				LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
				AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//�����˲�
				FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//��ȡ��߽ǵ�
				ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],(x_max)/2,Image_H-1);
			}
			RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
			//�����˲�
		//	AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
			AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
			
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		

				MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,1,1);//���ұ�����ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
			
		}
		else if(CircularRingStateFlag==5)
		{

					//�ز���
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-35,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		
		}	
		else if(CircularRingStateFlag==7||CircularRingStateFlag==6)
		{
			uint8 flag=0;
			//flag=FindUpPoints(Imagecopy,Image_W,Image_H,25,y_max-RightMax_Y_Position-1,&UpAngularPostion[0],&UpAngularPostion[1],1);
			//if(flag==1)
		//	{
			ImageDrawline(Imagecopy,Image_W,Image_H,0,y_max/2-10,x_max/2-10,y_max-1);
		//	}
			RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-10);
			//�����˲�
			AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
					//�ز���
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���

			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-30,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		}
		else if(CircularRingStateFlag==8)
		{
			
						//�ز���
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���	
			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
		}

}
/************��Բ����ͼ����****


********************************/
void RightCircularRing_ImageHandle()
{
	
	
	//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
	if(CircularRingStateFlag==0||CircularRingStateFlag==1||CircularRingStateFlag==2)
	{
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//�ز���
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			if(MiddleLineFlag==0)
			{
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
		else if(CircularRingStateFlag==3||CircularRingStateFlag==4)
		{
			uint8 flag=1;
			flag=FindUpPoints(Imagecopy,Image_W,Image_H,x_max-5,clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
			if(flag==1)
			{
				RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//�õ����������
				AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//�����˲�
				FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//��ȡ��߽ǵ�
				ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],(x_max/2),Image_H-1);
			}
			LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);//�õ����������
			AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�

	//�ز���
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
		
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-10,1);//��������ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
			
		}
		else if(CircularRingStateFlag==5)
		{
					//�ز���
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-30,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		
		}		
		else if(CircularRingStateFlag==7||CircularRingStateFlag==6)
		{
			uint8 flag=0;
			ImageDrawline(Imagecopy,Image_W,Image_H,x_max,y_max/2,x_max/2+10,y_max-1);

			LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-10);
			//�����˲�
			AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
					//�ز���
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-30,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		}
		else if(CircularRingStateFlag==8)
		{
			
						//�ز���
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//���ұ�����ȡ����
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
		}
		

}


//Ԫ���ж�
void ElementJudge()
{
	//����Ѳ��״̬��Ԫ�ؽ����ж�
	if(MotionStateFlag==0&&CardControlState==0)
	{
		uint8 Cicular_flag=0,Ramp_flag=0,Barrier_Flag=0;
		CrossJudge();//ʮ��Ԫ���ж�
		Cicular_flag=CircularRingJudge();
		if(Cicular_flag==1)
			MotionStateFlag=2;//Բ��״̬
		Ramp_flag=RampJudge(RasampleLeftPoints,RasampleLeftPointsLen,RasampleRightPoints,RasampleRightPointsLen,2);
		if(Ramp_flag==1)
		{
			MotionStateFlag=3;//�µ�״̬
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//������������ݣ�ʹ֮���»����ȶ���̬
		}
		ZebraJudge(Imagecopy,Image_W,Image_H,RasampleLeftPoints[0][0],RasampleRightPoints[0][0],y_max/2);
		
		//��ֱ�ߵ�������λ�ý����ж��ϰ���
//		Barrier_Flag=BarrierJudge(Imagecopy,Image_W,Image_H,RasampleLeftPoints[4][0],RasampleRightPoints[4][0],RasampleLeftPoints[4][1]);
//		if(Barrier_Flag==1)
//		{
//			MotionStateFlag=4;
//			P_encoder[0]=0;
//			P_encoder[1]=0;
//			P_encoder[2]=0;
//			P_encoder[3]=0;
//			AngleZ_2=0;
//			AngleZ=0;//������������ݣ�ʹ֮���»����ȶ���̬
//		}

		
	}
	else if(MotionStateFlag==1)//ʮ��ģʽ
	{
		if(CrossStateFlag==0&&LeftMax_Y_Position==0&&RightMax_Y_Position==0)//��ʮ�ֹ����У���ʾ���߶�����
		{	
			CrossStateFlag=1;//��ʾ��ʮ��
			LeftAngleMaxPoint[1]=y_max-1;
			RightAngleMaxPoint[1]=y_max-1;
			turnPD.P=0.7;
			turnPD.D=0.3;
		}
		else if(CrossStateFlag==1&&LeftMax_Y_Position>0&&RightMax_Y_Position>0)//��ʾ�ҵ��µ���
		{
			CrossStateFlag=2;//��ʾ�Ѿ���ʮ��
			RasampleMiddlePointslen2=0;//Զ������
			
			if(CrossCardFlag==0)//û�п�Ƭ��
			{
				
				CardControlState=1;//�л����ҿ�Ƭ�׶�
				P_encoder[0]=0;
				P_encoder[1]=0;
				P_encoder[2]=0;
			  P_encoder[3]=0;
			  AngleZ_2=0;
			  AngleZ=0;//������������ݣ�ʹ֮���»����ȶ���̬
			  AngleSpeed_z=0;
			}

			

			
		}
		else if(CrossStateFlag==2)//��ʮ���н����ж�
		{
			uint8 flag=0;
			flag=CrossJudge();
			if(flag==1)
				CrossStateFlag=3;//����׼����ʮ��״̬
		}
		else if(CrossStateFlag==3)
		{
			uint8 flag=0;
			uint16 points[0];
			
			if(RightMax_Y_Position>LeftMax_Y_Position)
			{
				flag=FindUpPoints(Imagecopy,Image_W,Image_H,RightAngleMaxPoint[0],clip(y_max-RightMax_Y_Position-1,0,y_max-10),&points[0],&points[1],1);
				LeftCrossFlag=1;//��ʮ��
			}
			else
			{
				flag=FindUpPoints(Imagecopy,Image_W,Image_H,LeftAngleMaxPoint[0],clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&points[0],&points[1],1);
				RightCrossFlag=1;//��ʮ��
			}
			if(flag==1)
			{
				if(points[1]>10)
				{
								
					CrossStateFlag=4;//��ʾ��ʮ����
					LeftAngleMaxPoint[1]=y_max-1;
					RightAngleMaxPoint[1]=y_max-1;
				}
					
			}


		}
		else if(CrossStateFlag==4&&LeftMax_Y_Position<=5&&RightMax_Y_Position<=5)//��ʾ�ҵ��µ���
		{
			CrossStateFlag=5;//�Ѿ���ʮ�֣�ˢ��ʮ��״̬

		}
		else if(CrossStateFlag==5&&CartStateFlag==0)
		{
			if(CrossCardFlag==1)
			{
				if(LeftMax_Y_Position>y_max/4||RightMax_Y_Position>y_max/4)
				{
						MotionStateFlag=0;//ˢ��Ѳ��ģʽ
						CrossStateFlag=0;
						CardControlState=0;//��Ƭ״̬
						LeftCrossFlag=0;
						RightCrossFlag=0;
						CrossCardFlag=0;//ȫ������
				}
			}
			else
			{
				if(LeftMax_Y_Position>y_max/4&&RightMax_Y_Position>y_max/4)
				{
						MotionStateFlag=0;//ˢ��Ѳ��ģʽ
						CrossStateFlag=0;
						CardControlState=0;//��Ƭ״̬
						LeftCrossFlag=0;
						RightCrossFlag=0;
						CrossCardFlag=0;//ȫ������
						turnPD.P=1;
						turnPD.D=0.5;
				}
			
			
			}

		}
		else if(CrossStateFlag==6)
		{
			if(LeftCrossFlag==1)
			{
				if(RightMax_Y_Position>y_max/2)//��������һ������
					CrossStateFlag=2;
				else//���߶�û��
					CrossStateFlag=1;
			}
			else if(RightCrossFlag==1)
			{
					if(LeftMax_Y_Position>y_max/2)//��������һ������
					CrossStateFlag=2;
				else//���߶�û��
					CrossStateFlag=1;
			}
			
		}
	}
	else if(MotionStateFlag==2)//Բ��ģʽ
	{
		if(CircularRingDirection==0)
		{
			LeftCircularRing_Handle();
		}
		else
		{
			RightCircularRing_Handle();
		}
	}
}
/*****************���������߻�ȡ����*********************
���룺����ϵ�Զ���������꣬Զ�����߳���
�������Ϻ�����꣬��Ϻ�����߳���



****************************************************/
uint16 UPLineGainMiddle(uint16 InPoints[][2],uint16 Inlen,uint16 OutPoints[][2],uint16 Dis)
{
	//��ǰ15���㲻ʹ����С���˷���ϣ�ֱ�Ӹ�ֵ����
	uint16 len=0;
	uint16 dis=0;
	if(Inlen>10)
		dis=3;
	for(uint16 i=0;i<Inlen-dis;i++)//��������3����
	{
			OutPoints[i][0]=InPoints[clip(Inlen-i,0,Inlen-1)][0]+Dis;
			OutPoints[i][1]=InPoints[clip(Inlen-i,0,Inlen-1)][1];
			len++;
	}
	return len;

}
//�Ի�ȡ�ı߽紦��
void LineTreatment()
{
	
		int leftlen,Rightlen;
		LeftMax_Y_Position=0;
		RightMax_Y_Position=0;
		//������ȡ
		
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);//�õ����������
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);
		//�����˲�
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//�����˲�
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//�����˲�
	
	
		//��ȡ���߽ǵ�
		FindAngle(AvargeLeftPoints,AvargeLeftPointsNum,LeftAngleMaxPoint,&LeftAngleMax,&LeftAngleMaxPosition,15);//��ȡ��߽ǵ�
		FindAngle(AvargeRightPoints,AvargeRightPointsNum,RightAngleMaxPoint,&RightAngleMax,&RightAngleMaxPosition,15);//��ȡ�ұ߽ǵ�
		
//		//Ԫ���жϣ��ж��Ƿ����ʮ�ֻ���Բ��
		ElementJudge();
//	
		if(MotionStateFlag==0)//����Ѳ��ģʽ
		{	
			//�ж����������ȡ�����ұ�����ȡ,���������߶�������Ĭ���ұ�����ȡ
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//�ز���
			
			
		
			if(MiddleLineFlag==0)
			{
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���ԭʼ���ݵĳ���
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//��������ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	
				RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//�Ⱦ����,�Ⱦ�����ĳ��Ȳ��ܳ���
				MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//���ұ�����ȡ����
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
//		//ʮ��ģʽ
//		/********ʮ��ģʽ�����׶�*******
//			1,����
//			2,����
//			3,����
//		
//		****************************/
		else if(MotionStateFlag==1)
		{
			CrossCardImage();
			
					//ʹ��Զ�������������
		}	
		else if(MotionStateFlag==2)//Բ������
		{
			if(CircularRingDirection==0)//��Բ��
			{
				LeftCircularRing_ImageHandle();
			}
			else//��Բ��
			{
				RightCircularRing_ImageHandle();
			}
		
		
		}
		
}