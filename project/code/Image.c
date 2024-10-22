#include "Image.h"
#include "math.h"
//临时修改缓存图像大小，用于写寻中线算法，原来参数uint8 Imagecopy[RESULT_ROW][RESULT_COL]
uint8 Imagecopy2[RESULT_ROW][RESULT_COL];//预处理前的图片
uint8 Imagecopy[RESULT_ROW][RESULT_COL];//备份数据,避免摄像头获取数据覆盖掉原来数据
uint8 *ImageUSE[RESULT_ROW][RESULT_COL];//逆透视变换结果图


uint16 LeftPoints[PointsSize][2];//原始左边线坐标
uint16 RightPoints[PointsSize][2];//原始右边线坐标

uint16 AvargeLeftPoints[PointsSize][2];//滤波后左边线坐标
uint16 AvargeRightPoints[PointsSize][2];//滤波后右边线坐标

uint16 RasampleLeftPoints[PointsSize][2];//等距采样后点集
uint16 RasampleRightPoints[PointsSize][2];//等距采样后点集
float LeftAngleMax=0;
float RightAngleMax=0;

uint16 LeftAngleMaxPoint[2];//左边线角度最大的点
uint16 RightAngleMaxPoint[2];//右边线角度最大的点

uint16 LeftMax_Y_Position=0;//左边最大y坐标差值
uint16 RightMax_Y_Position=0;//右边最大y坐标差值

uint16 LeftAngleMaxPosition=0;//边线中的第几个点
uint16 RightAngleMaxPosition=0;//边线中的第几个点




int LeftPointsNum=0;//左边线长度
int RightPointsNum=0;//右边线长度

int AvargeLeftPointsNum=0;//滤波后左边线长度
int AvargeRightPointsNum=0;//滤波后右边线长度

int RasampleLeftPointsLen=0;//等距采样后坐标长度
int RasampleRightPointsLen=0;//等距采样后坐标长度


uint8 MiddleLineFlag=1;//边线标志，用于判断从左边线提取中线还是从右边线提取中线
uint16 MiddlePoints[PointsSize][2];//中线坐标
uint16 ResampleMiddlePoints[PointsSize][2];//重采样后中线坐标
uint16 RasampleMiddlePointsLen=0;//重采样后中线长度
uint16 MiddlePointsNum=0;//中线长度

/************十字元素参数*************************/

uint16 MiddlePoints2[PointsSize][2];//原始左边线坐标
uint16 RasampleMiddlePoints2[PointsSize][2];//原始右边线坐标

uint16 MiddlePointslen2=0;//原始左边线坐标
uint16 RasampleMiddlePointslen2=0;//原始右边线坐标

uint16 LeftAngleMaxPoint2[2];//左拐点
uint16 RightAngleMaxPoint2[2];//右拐点

uint16 LeftAngleMaxPointPosition2;//左拐点位置
uint16 RightAngleMaxPointPosition2;//右拐点位置

uint16 LeftCrossFlag=0;//左十字标志
uint16 RightCrossFlag=0;//右十字标志
uint16 CrossCardFlag=0;//十字有无卡片标志
/************************************************/

/************圆环元素*************************/
uint16 UpAngularPostion[2];

uint16 LeftUpPoints[PointsSize][2];//原始左上边线坐标
uint16 RightUpPoints[PointsSize][2];//原始右上边线坐标

uint16 AvargeLeftUpPoints[PointsSize][2];//滤波后左上边线坐标
uint16 AvargeRightUpPoints[PointsSize][2];//滤波后右上边线坐标

int LeftUpPointsNum=0;//左边线长度
int RightUpPointsNum=0;//右边线长度

int AvargeLeftUpPointsNum=0;//滤波后左边线长度
int AvargeRightUpPointsNum=0;//滤波后右边线长度

float LeftUpAngleMax=0;
float RightUpAngleMax=0;

uint16 LeftUpAngleMaxPoint[2];//左边线角度最大的点
uint16 RightUpAngleMaxPoint[2];//右边线角度最大的点

uint16 LeftUpMax_Y_Position=0;//左边最大y坐标差值
uint16 RightUpMax_Y_Position=0;//右边最大y坐标差值

uint16 LeftUpAngleMaxPosition=0;//边线中的第几个点
uint16 RightUpAngleMaxPosition=0;//边线中的第几个点

uint16 Last_StartAnglePoint[2];//上一次结束的角点
/**********************************************/
//限制参数范围函数,
int clip(int x, int min, int max) {
	
	if(x>=max)
		x=max;
	else if(x<=min)
		x=min;
    return x;
}
/********3x3图像腐蚀**************



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
						//dy=1;//让其退出外部循环	
					}
					
				}
			img1[clip(i,0,y_max-1)][clip(j,0,x_max-1)]=max_value;
		
		}


}
//double  ReversePerspectiveMatrix[3][3]={};//逆透视矩阵
//图像被拉为一维数组，所以需要以寻址方式获取数据
//图像二值化
//处理透视变换后的图像
//将原始数据转存到Imagecopy，避免数据访问发生冲突
void ImageBinaryzation(uint8 ** image,uint16 width,uint16 height,uint8 threshold)
{
	  uint32 i = 0,j = 0;
		uint8 temp=0;
		for(j = 0; j < height; j ++)
    {
        for(i = 0; i < width; i ++)
        {
					 temp = **(image + j * width + i);
                        // 读取像素点
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
/*二值化图像，处理透视变换前的图像


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
                        // 读取像素点
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
//大津法二值化图像
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
  double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; //àà??·?2?
  int16 MinValue, MaxValue; // ×?D??ò?è?μ  ×?′ó?ò?è?μ
  uint8 Threshold = 0;      // ?D?μ
  uint8 HistoGram[256]={0};     // ?ò?è0-255?±·?í?        
 
//  for (j = 0; j < 256; j++)
//	HistoGram[j] = 0;
  
  for (j = 0; j < height; j++) 
  { 
    for (i = 0; i <width; i++) 
    { 
      HistoGram[*(image + j * width + i)]++; //í3???ò?è???D????μ???êy
    } 
  } 
  
  for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++) ;
  for (MaxValue = 255; MaxValue > MinValue && HistoGram[MinValue] == 0; MaxValue--) ;
      
  if (MaxValue == MinValue)     return MaxValue;         //í????D??óDò?????é? ?í·μ??×?′ó?ò?è?μ
  if (MinValue + 1 == MaxValue)  return MinValue;        //í????D??óDá?????é? ?í·μ??×?D??ò?è?μ
    
  for (j = MinValue; j <= MaxValue; j++)    Amount += HistoGram[j];        //  ????×üêy
    
  PixelIntegral = 0;
  for (j = MinValue; j <= MaxValue; j++)
  {
    PixelIntegral += HistoGram[j] * j;
  }
  SigmaB = -1;
  for (j = MinValue; j < MaxValue; j++)
  {
    PixelBack = PixelBack + HistoGram[j];   //?°?°????μ?êy
    PixelFore = Amount - PixelBack;         //±3?°????μ?êy
    OmegaBack = (double)PixelBack / Amount;//?°?°????°ù·?±è
    OmegaFore = (double)PixelFore / Amount;//±3?°????°ù·?±è
    PixelIntegralBack += HistoGram[j] * j;  //?°?°?ò?è?μ
    PixelIntegralFore = PixelIntegral - PixelIntegralBack;//±3?°?ò?è?μ
    MicroBack = (double)PixelIntegralBack / PixelBack;   //?°?°?ò?è°ù·?±è
    MicroFore = (double)PixelIntegralFore / PixelFore;   //±3?°?ò?è°ù·?±è
    Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//????àà??·?2?
    if (Sigma > SigmaB)                    //?
		{
      SigmaB = Sigma;
      Threshold = j;
    }
  }
  return (Threshold);                        //·μ??×????D?μ
}

//透视变换初始化函数，根据透视变换矩阵进行坐标映射
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

/*****加权平均滤波****
输入参数：点集，长度，滤波核大小
返回滤波后点集长度

***/

uint16 AverageFilter(uint16 Points[][2],uint16 len,uint16 Out_Points[][2],uint16 *Max_Y,uint8 Kernel)
{
	uint8 half_size=0;
	uint16 x=0,y=0;
	uint16 size=0;//计算滤波后点长度
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
			min_y=y;//保存最小值
		}
		Out_Points[size][0]=x;
		Out_Points[size][1]=y;
		size++;
	}
	*Max_Y=y_max-min_y;
	return size;
}

//寻找左边线
//实测寻找一次边线为5us
uint16 FindLetfSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//初始化坐标，从图像最后一行中点开始
	uint16 Initiation_Y=height-5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 Point[2]={0};//起始点坐标
	uint16 LeftPointsLen=0;//左边线长度
	//找起始点先不用
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//用于避免起始点陷入局部白色区域（卡片）
	if(*(image + Initiation_Y * width + Initiation_X)==0||((*(image + Initiation_Y * width + Initiation_X)==255)&&((*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0||*(image + Initiation_Y * width +  clip(Initiation_X-CardWidth,0,x_max-1))==0))))//如果底部中心是黑色，则往两边寻找中线
	{
		Initiation_X=Initiation_X+width/4;//先往右边寻找中点
		if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width +  clip(Initiation_X+CardWidth,0,x_max-1))==0))//如果右边四分之三处也是黑色，则往左边寻找
		{
			Initiation_X=Initiation_X-width/2;//往左边寻找中点
			if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width +  clip(Initiation_X-CardWidth,0,x_max-1))==0))//如果左边四分之一处也是黑色，则不找了
		{
			return 0;	
		}
		}
	}
	
	for(x=Initiation_X;x>0;x--)
	{
		temp=*(image + Initiation_Y * width + x);//寻找边线位置
		if(temp==0)
		{
			Point[0]=x+1;
			Point[1]=Initiation_Y;//获取边线起始点
			break;
		}
	}
	x=Point[0];
	y=Point[1];
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//已经到图像边界退出
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
		if(LeftPointsLen>=PointsSize)//以免点过多造成数组越界
			return LeftPointsLen-1;
	}
	return LeftPointsLen;
}

/*从左边线根据斜率获得中线
参数：边线点集合，输入边线长度,OffetValue偏移值,步长
输出：中线长度


*/
uint16 LeftGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size)
{
	float dy=0,dx=0;
	int x=0,y=0;
	uint16 MiddlePointsLen=0;//中线长度
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
从右边线获取中线


*/
uint16 RightGainMidden(uint16 Points[][2],uint16 len,float OffetValue,uint16 Step_size)
{
	float dy=0,dx=0;
	int x=0,y=0;
	uint16 MiddlePointsLen=0;//中线长度
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
/*重采样使相邻点折线距离相等
参数：输入点集，输入点集长度，输出点集，输出点集长度，相邻点距离

返回值，输出点集长度
*/
uint16 ResamplePoints(uint16 InPoints[][2],uint16 len,uint16 OutPoints[][2],uint16 len2,float dist)
{
	int remain=0,num=0;//remain为距离缓存
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

/*寻找角度值最大的点



取前后相邻pointsLen/2的三个点计算角度
*/
void FindAngle(uint16 InPoints[][2],uint16 len ,uint16 OutPoint[2],float *OutAngle,uint16 *OutPosition,uint16 pointsLen)
{
	float angle_max=0;
	float angle=0;
	uint16 x=0,y=0;
	if (len<5)//前面的点忽略掉
		return;
	for(uint16 i=0;i<len-1;i++)
	{
		if(InPoints[clip(i+pointsLen,0,len-1)][0]>=x_max-5||InPoints[clip(i+pointsLen,0,len-1)][0]<=5)
			continue;//忽略过于靠近边线的点
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

//寻找右边线
uint16 FindRightSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//初始化坐标，从图像最后一行中点开始
	uint16 Initiation_Y=height-5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 Point[2]={0};//起始点坐标
	uint16 RightPointsLen=0;//右边线长度
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//用于避免起始点陷入局部白色区域（卡片）
	if(*(image + Initiation_Y * width + Initiation_X)==0||((*(image + Initiation_Y * width + Initiation_X)==255)&&((*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0||*(image + Initiation_Y * width + clip(Initiation_X-CardWidth,0,x_max-1))==0))))//如果底部中心是黑色，则往两边寻找中线
	{
		Initiation_X=Initiation_X-width/4;//先往左边寻找中点
		if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width + clip(Initiation_X-CardWidth,0,x_max-1))==0))//如果右边四分之三处也是黑色，则往左边寻找
		{
			Initiation_X=Initiation_X+width/2;//往右边寻找中点
			if(*(image + Initiation_Y * width + Initiation_X)==0||(*(image + Initiation_Y * width + Initiation_X)==255&&*(image + Initiation_Y * width + clip(Initiation_X+CardWidth,0,x_max-1))==0))//如果左边四分之一处也是黑色，则不找了
		{
			return 0;	
		}
		}
	}
	//寻找起始点
	for(x=Initiation_X;x<width;x++)
	{
		temp=*(image + Initiation_Y * width + x);//寻找边线位置
		if(temp==0)
		{
			Point[0]=x-1;
			Point[1]=Initiation_Y;//获取边线起始点
			break;
		}
	}
	x=Point[0];
	y=Point[1];
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max-1)
			break;//已经到图像边界退出
		if(*(image + (y-1)* width + x)!=0)//前面一个像素
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
	if(RightPointsLen>=PointsSize)//以免点过多造成数组越界
			return RightPointsLen-1;
	
	}
	return RightPointsLen;
}


//寻找上边线
void FindUpinflexion(uint8 *image,uint16 width,uint16 height)
{
	int x=0,y=0;
	//初始化坐标，从图像第一行中点开始
	uint16 Initiation_Y=5;
	uint16 Initiation_X=width/2;
	uint8 temp=0;
	uint16 PointX1=0;//起始点坐标
	uint16 PointX2=0;//起始点坐标
	uint16 LastPointX1=0;//上一次点坐标
	uint16 LastPointX2=0;//起始点坐标
	//寻找起始点
	
	MiddlePointslen2=0;
	RasampleMiddlePointslen2=0;
	uint8 leftflag=0,rightflag=0,startFlag=0;
//	for(uint16 i=1;i<height;i++)//从上往下寻找开始行
//	{
//		if(*(image + i * width + Initiation_X)==255&&*(image + (i-1) * width + Initiation_X)==255)//由黑变白
//			Initiation_Y=i;
//	}
	if(*(image + Initiation_Y * width + Initiation_X)==0)//如果底部中心是黑色，则往两边寻找中线
	{
		Initiation_X=Initiation_X+width/4;//先往右边寻找中点
		if(*(image + Initiation_Y * width + Initiation_X)==0)//如果右边四分之三处也是黑色，则往左边寻找
		{
			Initiation_X=Initiation_X-width/2;//往左边寻找中点
//			if(*(image + Initiation_Y * width + Initiation_X)==0)//如果左边四分之一处也是黑色，则不找了
//		{
//			
//		}
		}
	}
	while(1)
	{
		if(y>=width)
			break;
	for(x=Initiation_X;x>0;x--)//寻找左边边界点
	{
		temp=*(image + y * width + x);//寻找边线位置
		if(temp==0&&*(image + y * width + x+1)==255)
		{
			PointX1=x+1;
			break;
		}
	}
	if(x==0)//表示左边没有找到点
	{
		leftflag=1;
	}
		for(x=Initiation_X+1;x<width;x++)//寻找左边边界点
	{
		temp=*(image + y * width + x);//寻找边线位置
		if(temp==0&&*(image + y * width + x-1)==255)
		{
			PointX2=x-1;
			break;
		}
	}
	if(x==width)//表示右侧没有找到点
	{
		rightflag=1;
	}
	if(startFlag==1)
	{	
		if(leftflag==0&&rightflag==0)//右侧有点左侧没有
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
		if(leftflag!=1&&rightflag==1)//左侧有点右侧没有
		{
				Initiation_X=PointX1+TrackDeviation;
		}
		if(leftflag==1&&rightflag!=1)//表示两侧都有点
		{	
			//if(abs(PointX2-LastPointX2)<=10)//判断前后两个横坐标点差别是否过大
			Initiation_X=PointX2-TrackDeviation;
			
		}
		if(leftflag==1&&rightflag==1)//表示两侧都空白.则结束巡线
		{
			break;
		}
	}
	else
	{
		if(leftflag==0&&rightflag==0)//右侧有点左侧没有
		{
			 LastPointX1=PointX1;
			 LastPointX2=PointX2;
				if(abs(PointX2-LastPointX2)<=10&&abs(PointX1-LastPointX1)<=10)
			 startFlag=1;
		}
		if(leftflag==1&&rightflag==0)//右侧有点左侧没有
		{
			 LastPointX2=PointX2;
		}
		if(leftflag==0&&rightflag==1)//右侧有点左侧没有
		{
			 LastPointX1=PointX1;
		}
		

	}

			leftflag=0;//清除标志位
			rightflag=0;
			
			y++;
				if(MiddlePointslen2>=y_max)
			MiddlePointslen2=MiddlePointslen2-1;
	}
	
	//重采样
	RasampleMiddlePointslen2=ResamplePoints(MiddlePoints2,MiddlePointslen2,RasampleMiddlePoints2,MiddlePointslen2,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
	
}
//*************************寻找左上边线



/*****************************/
uint16 FindLeftUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//初始化坐标，从图像最后一行中点开始
	uint16 Initiation_Y=0;
	uint16 Initiation_X=0;
	uint8 temp=0;
	uint16 Point[2]={0};//起始点坐标
	uint16 LeftUpPointsLen=0;//左上边线长度
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//寻找起始点
	x=Initiation_X;
	y=Initiation_Y;
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//已经到图像边界退出
		if(*(image + clip(y,0,y_max-1)* width + x+1)!=0)//前面一个像素
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
			if(*(image + clip(y+1,0,y_max-1)* width + x)!=0)//左边是不是零
			{
				if(*(image + clip(y+1,0,y_max-1)* width + x+1)!=0)//前面右边
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
	if(LeftUpPointsLen>=PointsSize)//以免点过多造成数组越界
			return PointsSize-1;
	
	} 
	return LeftUpPointsLen;
}

//*************************寻找右上边线



/*****************************/
uint16 FindRightUpSideLine(uint8 *image,uint16 width,uint16 height,uint16 In_x,uint16 In_y)
{
	int x=0,y=0;
	//初始化坐标，从图像最后一行中点开始
	uint16 Initiation_Y=0;
	uint16 Initiation_X=0;
	uint8 temp=0;
	uint16 Point[2]={0};//起始点坐标
	uint16 RightUpPointsLen=0;//左上边线长度
	Initiation_Y=In_y;
	Initiation_X=In_x;
	//寻找起始点
	x=Initiation_X;
	y=Initiation_Y;
	while(1)
	{
		if(y==0||x==0||y>=y_max||x>=x_max)
			break;//已经到图像边界退出
		if(*(image + (y)* width + x-1)!=0)//前面一个像素
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
			if(*(image + (y+1)* width + x)!=0)//左边是不是零
			{
				if(*(image + (y+1)* width + x-1)!=0)//前面右边
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
	if(RightUpPointsLen>=PointsSize)//以免点过多造成数组越界
			return PointsSize-1;
	
	} 
	return RightUpPointsLen;
}


/*****找上方跳变点******
参数：输入图像，图像宽度，图像高度，起始点x坐标，起始点y坐标，输出x，输出y，步长(必须为整数)
从下往上扫线寻找
*/
uint8 FindUpPoints(uint8 *image,uint16 width,uint16 height,uint16 In_X,uint16 In_Y,uint16 * out_x,uint16 *out_y,uint8 step)
{
	for(uint16 i=In_X;i<width-1;i=i+step)//x方向更新
	{
		for(int j=In_Y;j>=0;j--)
		{
			if(*(image + clip(j,0,height-1) * width + clip(i,0,width-1))==0&&*(image + clip(j+1,0,height-1)* width + clip(i,0,width-1))==255&&*(image + clip(j-1,0,height-1)* width + clip(i,0,width-1))==0)//从下往上找找到黑色边界点
			{
				//进一步判断，从四周点的斜率进行判断
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
/***********图像补线********************

使用两点直接补图像，不使用最小二乘法
*************************************/

void ImageDrawline(uint8 *image,uint16 width,uint16 height,uint16 Start_X,uint16 Start_Y,uint16 End_X,uint16 End_Y)
{
	float k=0;
	int x=0,y=0,y_1=0,Last_y=0;
	int Max_X=0,Max_Y=0,Min_X=0,Min_Y=0,start_y=0,start_x=0;
	int err_y=0;
	if(Start_X!=End_X)
	{
		k=10000*((short)Start_Y-(short)End_Y)/((short)Start_X-(short)End_X);//扩大10000倍避免因为数据类型不匹配导致截断
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
		
		*(image+(uint16)y*width+(uint16)x)=0;//将图像对应坐标补为黑色
		if(abs(err_y)>1)
		{	
			for(int i=abs(err_y)-1;i>=0;i--)
			{
				
				if(err_y>0)
				{
					y_1=Last_y;
					*(image+(uint16)clip(y_1+i,0,y_max-1)*width+(uint16)x)=0;//将图像对应坐标补为黑色
				}
				else
				{
					y_1=Last_y;
					*(image+(uint16)clip(y_1-i,0,y_max-1)*width+(uint16)x)=0;//将图像对应坐标补为黑色
					
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
			*(image+(uint16)y*width+(uint16)x)=0;//将图像对应坐标补为黑色
		}
	}

}
//十字判断
//返回1表示是十字，否则返回0
uint8 CrossJudge()
{
	
	//十字判断
		if(LeftAngleMaxPoint[1]>y_max-30&&RightAngleMaxPoint[1]>y_max-30&&LeftMax_Y_Position<y_max/2&&RightMax_Y_Position<y_max/2)//两边角点位置小于30
		{
			if(abs(LeftAngleMax-85)<=15&&abs(RightAngleMax-85)<=15)//如果两侧角点最大角度变化为90度，则为正入十字
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				//对十字再做一次判断，避免误判
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				//rt_kprintf("left %d,%d,%d,%d,right:&d,%d,%d,%d,%d,%d\n",lefterr1,lefterr2,lefterr3,lefterr4,righterr1,righterr2,righterr3,righterr4,LeftAngleMaxPosition,RightAngleMaxPosition)        ;                                                                                                                      
				if(lefterr1<5&&lefterr2<10&&lefterr3>5&&lefterr4>15&&righterr1<5&&righterr2<5&&righterr3>5&&righterr4>5)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
				MotionStateFlag=1;//十字
				//刷新角度
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
		else if(abs(LeftAngleMax-85)<=15&&abs(RightAngleMax-85)>=15)//只有右边有直角，只对右边截断，同时从右边获取边线
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
				MotionStateFlag=1;//十字
				//刷新角度
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
		else if(abs(LeftAngleMax-85)>=15&&abs(RightAngleMax-85)<=15)//只有左边有直角，只对左边截断，同时从右边获取边线
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
				MotionStateFlag=1;//十字
				//刷新角度
				LeftAngleMax=0;
				RightAngleMax=0;
				return 1;
				}
				return 0;
			}
			return 0;
		}
		else if (RightMax_Y_Position>y_max/2&&LeftMax_Y_Position<y_max/2)//左侧丢点右侧没丢
		{
			if(abs(RightAngleMax-85)<=15)//只有左边有直角，只对左边截断，同时从右边获取边线
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
			//	rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
					MotionStateFlag=1;//十字
				//刷新角度
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
			if(abs(LeftAngleMax-85)<=15)//只有右边有直角，只对右边截断，同时从右边获取边线
			{
					int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
					lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
					lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
					lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
					lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
					if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
					{
						MotionStateFlag=1;//十字状态
						//刷新角度
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
/************坡道判断******************
//输入左右边线,相隔多少个点

***************************************/

uint8 RampJudge(uint16 LeftPoints[][2],uint16 LeftNum,uint16 RightPoints[][2],uint8 RightNum,uint8 dis)
{
	uint16 Last_width=0,width=0,num=0,first_width=0;
	if(RightNum>LeftNum)//边线最短点数量
		num=LeftNum;
	else
		num=RightNum;
	if(RightMax_Y_Position>=y_max-10&&LeftMax_Y_Position>=y_max-10)//两边贯穿整个屏幕
	{
		//rt_kprintf("A\n");
		
		for(uint16 i=0;i<num;i=i+dis)//判断宽度是否一直变宽
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
		for(int i=0;i<num;i=i+dis)//判断边线斜率是否正确减少或增大
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
/*************斑马线判断******************
输入：图像，起始x值，终止x值，左边长度，右边长度，重复次数


*/


uint8 ZebraJudge(uint8 *image,uint16 width,uint16 hight,uint16 Start_X,uint16 End_X,uint16 Start_Line)
{
	uint16 i=0;
	uint8 sum=0;//跳变点总数
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
/***********障碍物判断***********

3种情况，障碍物贴左边，障碍物放中间，障碍物贴右边
*/


uint8 BarrierJudge(uint8 *image,uint16 width,uint16 hight,uint16 Start_X,uint16 End_X,uint16 Start_Line)
{
	int X_white2black[5]={0},X_black2white[5]={0};
	uint8  X_white2black_num=0,X_black2white_num=0;
	uint8 i=0;
	/*先计算障碍物宽度***/
	for(i=Start_X;i<End_X;i++)
		{
			
				if(*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i)==0&&*(image+(uint16)clip(Start_Line,0,y_max-1)*width+(uint16)i-1)==255)//找到障碍物起始点
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
		if(X_white2black_num==1&&X_black2white_num==2)//一个白变黑，两个黑变白的点（包括起始的边界点）
		{
			if(X_white2black[0]-X_black2white[0]>=25&&X_black2white[1]-X_white2black[0]>=12&&X_black2white[1]-X_white2black[0]<=16)//右障碍
			{
				BarrierDirection=1;//右边障碍物
				return 1;
			}
			else if(X_white2black[0]-X_black2white[0]<=12&&X_black2white[1]-X_white2black[0]>=12&&X_black2white[1]-X_white2black[0]<=16)//左障碍
			{
				BarrierDirection=0;//左边障碍物
				return 1;
			}
			
		
		}
		return 0;
		
		
	
	

}
/*************十字巡线处理************/
void CrossCardImage()
{
			if(CrossStateFlag==0)
			{
										//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
					if(RightMax_Y_Position>LeftMax_Y_Position)
						MiddleLineFlag=1;
					else if(LeftMax_Y_Position>RightMax_Y_Position)
						MiddleLineFlag=0;
					else 
						MiddleLineFlag=1;
							//重采样
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
											RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
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
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,Points1[0],Points1[1]);//得到左边线坐标
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],RightUpAngleMaxPoint[0],y_max-1);	
					
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,Points2[0],Points2[1]);//得到左边线坐标
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],LeftUpAngleMaxPoint[0],y_max-1);
				}
					LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);//得到左边线坐标
					RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);
		//边线滤波
					AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
					AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
				
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//重采样
			
			
		
			if(MiddleLineFlag==0)
			{
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
				if(CrossCardFlag==0)
					MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
				else
				{
					MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//从左线提取赛道
				}
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	
				RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
				if(CrossCardFlag==0)
					MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
				else
				{
					MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//从右边线提取赛道
				}
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
//				if(CrossCardFlag==0)//没有卡片堆
//				{
//					
//					RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,0);
//				}
//				else//有卡片堆
//				{
//					if(LeftCrossFlag==1)//左十字，右边巡线,补线
//						{
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,TrackDeviation);
//						}
//						else if(RightCrossFlag==1)//右十字
//						{
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,-TrackDeviation);
//						}	
//				}
			}
			else if(CrossStateFlag==2)
			{

		
					//有卡片堆
					if(CrossCardFlag==0)//没有卡片堆
					{
											//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
						if(RightMax_Y_Position>LeftMax_Y_Position)
							MiddleLineFlag=1;
						else if(LeftMax_Y_Position>RightMax_Y_Position)
							MiddleLineFlag=0;
						else 
							MiddleLineFlag=1;
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
						RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
								
						if(MiddleLineFlag==0)
						{
							MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else 
						{		MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
					
					}
					else//有卡片堆
					{
						if(LeftCrossFlag==1)//左十字，右边巡线
						{
							RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
							AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
							RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
				
							MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-15,1);//从右边线提取赛道
							RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//右十字
						{
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);//得到左边线坐标
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
							
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
							
								MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-15,1);//从左线提取赛道
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}	
					
					}

			}
			else if(CrossStateFlag==3)
			{
				
				if(CrossCardFlag==0)//没有卡片堆
				{
										//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
					if(RightMax_Y_Position>LeftMax_Y_Position)
						MiddleLineFlag=1;
					else if(LeftMax_Y_Position>RightMax_Y_Position)
						MiddleLineFlag=0;
					else 
						MiddleLineFlag=1;
							//重采样
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
											RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
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
							//重采样
					if(MiddleLineFlag==0)
					{
						RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,LeftAngleMaxPosition,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
						MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-15,1);//从左线提取赛道
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
					else 
					{	
						RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,RightAngleMaxPosition,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
						MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-15,1);//从右边线提取赛道
						RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
					}
				
				}
			}
			else if(CrossStateFlag==4)
			{
					if(LeftCrossFlag==1)//左十字，右边巡线,补线
						{
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,RightAngleMaxPoint[0],clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
								AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
								RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
								if(CrossCardFlag==1)
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,1,1);//从右边线提取赛道
								else
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//右十字
						{
							
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,LeftAngleMaxPoint[0],clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
								if(CrossCardFlag==1)
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,1,1);//从右边线提取赛道
								else
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从右边线提取赛道
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
								
						}	
				}
			else if(CrossStateFlag==5)
			{
				if(LeftCrossFlag==1)//左十字，右边巡线,补线
						{
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,x_max-5,clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
									AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
								AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
								RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
				
								if(CrossCardFlag==1)
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//从右边线提取赛道
								else
									MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
						}
						else if(RightCrossFlag==1)//右十字
						{
							
								uint8 flag=0;
							  flag=FindUpPoints(Imagecopy,Image_W,Image_H,5,clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
								if(flag==1)
								{
									LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
									AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//边线滤波
									FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//提取左边角点
									ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],x_max/2,y_max-1);
								}
								LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);
								AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
								RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
				
									if(CrossCardFlag==1)
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//从右边线提取赛道
								else
									MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从右边线提取赛道
								RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
								
						}	
				
//				if(CrossCardFlag==0)//没有卡片堆
//				{
//					FindUpinflexion(Imagecopy,Image_W,Image_H);//提取远处直线
//					RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,0);
//				}
//				else//有卡片堆
//				{
//					if(LeftCrossFlag==1)//左十字，右边巡线,补线
//						{
//							FindUpinflexion(Imagecopy,Image_W,Image_H);//提取远处直线
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,TrackDeviation);
//						}
//						else if(RightCrossFlag==1)//右十字
//						{
//							FindUpinflexion(Imagecopy,Image_W,Image_H);//提取远处直线
//							RasampleMiddlePointsLen=UPLineGainMiddle(RasampleMiddlePoints2,RasampleMiddlePointslen2,ResampleMiddlePoints,-TrackDeviation);
//						}	
//				}
			}


}
/*******判断圆环******************8

//没有测试过，得上车调试
1有圆环，0则无
**/
uint8 CircularRingJudge()
{
	//判断右圆环
	if(LeftMax_Y_Position>=y_max-1&&abs(RightAngleMax-85)<=15&&RightMax_Y_Position<=y_max/4)
	{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<5&&righterr2<10&&righterr3>5&&righterr4>15)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
						MotionStateFlag=2;//圆环
						CircularRingDirection=1;//右圆环
						//刷新角度
						LeftAngleMax=0;
						RightAngleMax=0;
						return 1;
				}			
				return 0;
		//进一步判断
	}
	else if(RightMax_Y_Position>=y_max-1&&abs(LeftAngleMax-85)<=15&&LeftMax_Y_Position<=y_max/4)
	{
		int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				if(lefterr1<5&&lefterr2<10&&lefterr3>5&&lefterr4>15)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
						MotionStateFlag=2;//十字状态
						//刷新角度
						CircularRingDirection=0;//左圆环
						LeftAngleMax=0;
						RightAngleMax=0;
						return 1;
				}
	
	}
		
	return 0;
}

/****左圆环处理********

******/
void LeftCircularRing_Handle()
{
	
	if(CircularRingStateFlag==0)//进入环岛1状态，初入环岛
	{
		if(RightMax_Y_Position>=y_max-1&&LeftMax_Y_Position==0&&LeftPointsNum==0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=1;//进入环岛状态2，进入环岛
		}
	}
	else if(CircularRingStateFlag==1)
	{
		if(RightMax_Y_Position>=y_max-1&&LeftPointsNum>0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=2;//进入环岛状态3，准备转向
			CardControlState=1;//进入找卡片阶段，记得取消注释
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//清除陀螺仪数据，使之重新积分稳定姿态
			AngleSpeed_z=0;
		}
	}
	else if(CircularRingStateFlag==2)
	{
		if(RightMax_Y_Position>=y_max-1&&LeftMax_Y_Position==0&&LeftPointsNum==0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=3;//进入环岛状态3，准备转向
		}
	}
	else if(CircularRingStateFlag==3)
	{
			
		if(LeftMax_Y_Position==0&&RightMax_Y_Position==0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=4;//进入环岛状态4，左右边线第一次丢失
		}
		else if(RightMax_Y_Position>0&&RightMax_Y_Position<y_max/2&&RightPoints[RightPointsNum][0]<=5)
		{
			CircularRingStateFlag=5;//直接进入状态5
		}
			
	}
	else if(CircularRingStateFlag==4)
	{
		if(RightMax_Y_Position>0&&RightMax_Y_Position<=y_max/2)//如果右边直线连续，并且左边丢线一次
		{
			PreviewDist0=4;//前瞻更改
			CircularRingStateFlag=5;//进入环岛状态5,表示已经进入环岛
		}
	}
	else if(CircularRingStateFlag==5)
	{
		if(abs(RightAngleMax-90)<=20)//只有左边有直角，只对左边截断，同时从右边获取边线
			{
				int righterr1=0,righterr2=0,righterr3=0,righterr4=0;
				righterr1=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr2=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition-20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr3=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+10,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				righterr4=abs(AvargeRightPoints[RightAngleMaxPosition][0]-AvargeRightPoints[clip(RightAngleMaxPosition+20,0,AvargeRightPointsNum-1)][0]);//角点前3个点的横坐标
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(righterr1<8&&righterr2<16&&righterr3>5&&righterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
				//刷新角度
				LeftAngleMax=0;
				RightAngleMax=0;
				CircularRingStateFlag=6;
				//	PreviewDist0=9;//前瞻更改
					//临时修改pid参数
				}
			}
	}
	else if(CircularRingStateFlag==6)
	{
		
		if(RightMax_Y_Position==0)//如果右边直线连续，并且左边丢线一次
		{
			PreviewDist0=4;//前瞻更改
			CircularRingStateFlag=7;//进入环岛状态4，左右边线第一次丢失
		}

	}
		else if(CircularRingStateFlag==7)
	{
		if(RightMax_Y_Position>0&&RightMax_Y_Position>=y_max/2&&RightPoints[RightPointsNum][1]<=5)//如果右边直线连续，并且左边丢线一次
		{

			CircularRingStateFlag=8;//退出圆环状态
		}

	}
	else if(CircularRingStateFlag==8)
	{
		
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);//得到左边线坐标
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);
		//边线滤波
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
		if(LeftMax_Y_Position>=y_max/4&&RightMax_Y_Position>=y_max-35)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=0;//退出圆环状态
			MotionStateFlag=0;
		}

	}
	
}

/****右圆环处理********

******/
void RightCircularRing_Handle()
{
	
	if(CircularRingStateFlag==0)//进入环岛1状态，初入环岛
	{
		if(LeftMax_Y_Position>=y_max-1&&RightMax_Y_Position==0&&RightPointsNum==0)//如果左边直线连续，并且右边丢线一次
		{
			CircularRingStateFlag=1;//进入环岛状态2，进入环岛
		}
	}
	else if(CircularRingStateFlag==1)
	{
		if(LeftMax_Y_Position>=y_max-1&&RightPointsNum>0)//如果左边直线连续，并且右边线重新出现
		{
			CircularRingStateFlag=2;//进入环岛状态3，准备转向
			CardControlState=1;//进入找卡片阶段，记得取消注释
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//清除陀螺仪数据，使之重新积分稳定姿态
			Angle_return_Target=0;
			AngleSpeed_z=0;
			
		}
	}
	else if(CircularRingStateFlag==2)
	{
		if(LeftMax_Y_Position>=y_max-1&&RightMax_Y_Position==0&&RightPointsNum==0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=3;//进入环岛状态3，准备转向
		}
	}
	else if(CircularRingStateFlag==3)
	{
		
		if(LeftMax_Y_Position==0&&RightMax_Y_Position==0)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=4;//进入环岛状态4，左右边线第一次丢失
		}
		else if(LeftMax_Y_Position>0&&LeftMax_Y_Position<y_max/2&&LeftPoints[LeftPointsNum][0]>=x_max-5)
		{
			CircularRingStateFlag=5;//直接进入状态5
		}
		
	}
	else if(CircularRingStateFlag==4)
	{
		if(LeftMax_Y_Position>y_max/4)//如果右边直线连续，并且左边丢线一次
		{
			PreviewDist0=4;//前瞻更改
			CircularRingStateFlag=5;//进入环岛状态5,表示已经进入环岛
		}
	}
	else if(CircularRingStateFlag==5)
	{
		if(abs(LeftAngleMax-90)<=20)//只有左边有直角，只对左边截断，同时从右边获取边线
			{
				int lefterr1=0,lefterr2=0,lefterr3=0,lefterr4=0;
				lefterr1=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr2=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition-20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr3=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+10,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				lefterr4=abs(AvargeLeftPoints[LeftAngleMaxPosition][0]-AvargeLeftPoints[clip(LeftAngleMaxPosition+20,0,AvargeLeftPointsNum-1)][0]);//角点前3个点的横坐标
				//rt_kprintf("%d,%d,%d,%d,%d\n",righterr1,righterr2,righterr3,righterr4,RightAngleMaxPosition)        ;                           
				if(lefterr1<8&&lefterr2<16&&lefterr3>5&&lefterr4>10)
			//截断处理
			//最小二乘计算拟合直线斜率	
				{
				//刷新角度
					LeftAngleMax=0;
					RightAngleMax=0;
					CircularRingStateFlag=6;
					//PreviewDist0=9;//前瞻更改
				}
			}
	}
	else if(CircularRingStateFlag==6)
	{
		
		
		if(LeftMax_Y_Position==0)//如果左边直线连续，并且左边丢线一次
		{
			
			CircularRingStateFlag=7;//进入环岛状态4，左右边线第一次丢失
		}
		//注释记得改回去

	}
		else if(CircularRingStateFlag==7)
	{
		if(LeftMax_Y_Position>0&&LeftMax_Y_Position>=y_max/2&&LeftPoints[LeftPointsNum][1]<=5)//如果右边直线连续，并且左边丢线一次
		{
			turnPD.P=1;
			turnPD.D=0.5;
			CircularRingStateFlag=8;//退出圆环状态
		}

	}
	else if(CircularRingStateFlag==8)
	{
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);//得到左边线坐标
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-20);
		//边线滤波
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
		if(LeftMax_Y_Position>=y_max-35&&RightMax_Y_Position>=y_max/4)//如果右边直线连续，并且左边丢线一次
		{
			CircularRingStateFlag=0;//退出圆环状态
			MotionStateFlag=0;
			CartStateFlag=0;
			CardControlState=0;

		}

	}
	
}
//左圆环对图像处理
void LeftCircularRing_ImageHandle()
{
	//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
	if(CircularRingStateFlag==0||CircularRingStateFlag==1||CircularRingStateFlag==2)
	{
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//重采样
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			if(MiddleLineFlag==0)
			{
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
		else if(CircularRingStateFlag==3||CircularRingStateFlag==4)
		{

			uint8 flag=1;
			flag=FindUpPoints(Imagecopy,Image_W,Image_H,5,clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
			if(flag==1)
			{
				LeftUpPointsNum=FindLeftUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
				AvargeLeftUpPointsNum=AverageFilter(LeftUpPoints,LeftUpPointsNum,AvargeLeftUpPoints,&LeftUpMax_Y_Position,7);//边线滤波
				FindAngle(AvargeLeftUpPoints,AvargeLeftUpPointsNum,LeftUpAngleMaxPoint,&LeftUpAngleMax,&LeftUpAngleMaxPosition,15);//提取左边角点
				ImageDrawline(Imagecopy,Image_W,Image_H,LeftUpAngleMaxPoint[0],LeftUpAngleMaxPoint[1],(x_max)/2,Image_H-1);
			}
			RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/10,Image_H-5);
			//边线滤波
		//	AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
			AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
			
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		

				MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,1,1);//从右边线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
			
		}
		else if(CircularRingStateFlag==5)
		{

					//重采样
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-35,1);//从右边线提取赛道
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
			//边线滤波
			AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
					//重采样
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过

			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,-30,1);//从右边线提取赛道
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		}
		else if(CircularRingStateFlag==8)
		{
			
						//重采样
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过	
			MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,0,1);//从右边线提取赛道
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
		}

}
/************右圆环对图像处理****


********************************/
void RightCircularRing_ImageHandle()
{
	
	
	//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
	if(CircularRingStateFlag==0||CircularRingStateFlag==1||CircularRingStateFlag==2)
	{
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//重采样
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
			RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			if(MiddleLineFlag==0)
			{
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
		else if(CircularRingStateFlag==3||CircularRingStateFlag==4)
		{
			uint8 flag=1;
			flag=FindUpPoints(Imagecopy,Image_W,Image_H,x_max-5,clip(y_max-RightMax_Y_Position-1,0,y_max-10),&UpAngularPostion[0],&UpAngularPostion[1],1);
			if(flag==1)
			{
				RightUpPointsNum=FindRightUpSideLine(Imagecopy,Image_W,Image_H,UpAngularPostion[0],UpAngularPostion[1]);//得到左边线坐标
				AvargeRightUpPointsNum=AverageFilter(RightUpPoints,RightUpPointsNum,AvargeRightUpPoints,&RightUpMax_Y_Position,7);//边线滤波
				FindAngle(AvargeRightUpPoints,AvargeRightUpPointsNum,RightUpAngleMaxPoint,&RightUpAngleMax,&RightUpAngleMaxPosition,15);//提取左边角点
				ImageDrawline(Imagecopy,Image_W,Image_H,RightUpAngleMaxPoint[0],RightUpAngleMaxPoint[1],(x_max/2),Image_H-1);
			}
			LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-5);//得到左边线坐标
			AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波

	//重采样
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
		
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-10,1);//从左线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
			
		}
		else if(CircularRingStateFlag==5)
		{
					//重采样
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-30,1);//从右边线提取赛道
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		
		}		
		else if(CircularRingStateFlag==7||CircularRingStateFlag==6)
		{
			uint8 flag=0;
			ImageDrawline(Imagecopy,Image_W,Image_H,x_max,y_max/2,x_max/2+10,y_max-1);

			LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W*9/10,Image_H-10);
			//边线滤波
			AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
					//重采样
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,-30,1);//从右边线提取赛道
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
		}
		else if(CircularRingStateFlag==8)
		{
			
						//重采样
			RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
		
			MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,0,1);//从右边线提取赛道
			RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			
		}
		

}


//元素判断
void ElementJudge()
{
	//常规巡线状态对元素进行判断
	if(MotionStateFlag==0&&CardControlState==0)
	{
		uint8 Cicular_flag=0,Ramp_flag=0,Barrier_Flag=0;
		CrossJudge();//十字元素判断
		Cicular_flag=CircularRingJudge();
		if(Cicular_flag==1)
			MotionStateFlag=2;//圆环状态
		Ramp_flag=RampJudge(RasampleLeftPoints,RasampleLeftPointsLen,RasampleRightPoints,RasampleRightPointsLen,2);
		if(Ramp_flag==1)
		{
			MotionStateFlag=3;//坡道状态
			P_encoder[0]=0;
			P_encoder[1]=0;
			P_encoder[2]=0;
			P_encoder[3]=0;
			AngleZ_2=0;
			AngleZ=0;//清除陀螺仪数据，使之重新积分稳定姿态
		}
		ZebraJudge(Imagecopy,Image_W,Image_H,RasampleLeftPoints[0][0],RasampleRightPoints[0][0],y_max/2);
		
		//从直线第五个点的位置进行判断障碍物
//		Barrier_Flag=BarrierJudge(Imagecopy,Image_W,Image_H,RasampleLeftPoints[4][0],RasampleRightPoints[4][0],RasampleLeftPoints[4][1]);
//		if(Barrier_Flag==1)
//		{
//			MotionStateFlag=4;
//			P_encoder[0]=0;
//			P_encoder[1]=0;
//			P_encoder[2]=0;
//			P_encoder[3]=0;
//			AngleZ_2=0;
//			AngleZ=0;//清除陀螺仪数据，使之重新积分稳定姿态
//		}

		
	}
	else if(MotionStateFlag==1)//十字模式
	{
		if(CrossStateFlag==0&&LeftMax_Y_Position==0&&RightMax_Y_Position==0)//入十字过程中，表示两边都丢线
		{	
			CrossStateFlag=1;//表示入十字
			LeftAngleMaxPoint[1]=y_max-1;
			RightAngleMaxPoint[1]=y_max-1;
			turnPD.P=0.7;
			turnPD.D=0.3;
		}
		else if(CrossStateFlag==1&&LeftMax_Y_Position>0&&RightMax_Y_Position>0)//表示找到新的线
		{
			CrossStateFlag=2;//表示已经入十字
			RasampleMiddlePointslen2=0;//远处中线
			
			if(CrossCardFlag==0)//没有卡片堆
			{
				
				CardControlState=1;//切换到找卡片阶段
				P_encoder[0]=0;
				P_encoder[1]=0;
				P_encoder[2]=0;
			  P_encoder[3]=0;
			  AngleZ_2=0;
			  AngleZ=0;//清除陀螺仪数据，使之重新积分稳定姿态
			  AngleSpeed_z=0;
			}

			

			
		}
		else if(CrossStateFlag==2)//在十字中进行判断
		{
			uint8 flag=0;
			flag=CrossJudge();
			if(flag==1)
				CrossStateFlag=3;//进入准备出十字状态
		}
		else if(CrossStateFlag==3)
		{
			uint8 flag=0;
			uint16 points[0];
			
			if(RightMax_Y_Position>LeftMax_Y_Position)
			{
				flag=FindUpPoints(Imagecopy,Image_W,Image_H,RightAngleMaxPoint[0],clip(y_max-RightMax_Y_Position-1,0,y_max-10),&points[0],&points[1],1);
				LeftCrossFlag=1;//左十字
			}
			else
			{
				flag=FindUpPoints(Imagecopy,Image_W,Image_H,LeftAngleMaxPoint[0],clip(y_max-LeftMax_Y_Position-1,0,y_max-10),&points[0],&points[1],1);
				RightCrossFlag=1;//右十字
			}
			if(flag==1)
			{
				if(points[1]>10)
				{
								
					CrossStateFlag=4;//表示出十字中
					LeftAngleMaxPoint[1]=y_max-1;
					RightAngleMaxPoint[1]=y_max-1;
				}
					
			}


		}
		else if(CrossStateFlag==4&&LeftMax_Y_Position<=5&&RightMax_Y_Position<=5)//表示找到新的线
		{
			CrossStateFlag=5;//已经出十字，刷掉十字状态

		}
		else if(CrossStateFlag==5&&CartStateFlag==0)
		{
			if(CrossCardFlag==1)
			{
				if(LeftMax_Y_Position>y_max/4||RightMax_Y_Position>y_max/4)
				{
						MotionStateFlag=0;//刷回巡线模式
						CrossStateFlag=0;
						CardControlState=0;//捡卡片状态
						LeftCrossFlag=0;
						RightCrossFlag=0;
						CrossCardFlag=0;//全部重置
				}
			}
			else
			{
				if(LeftMax_Y_Position>y_max/4&&RightMax_Y_Position>y_max/4)
				{
						MotionStateFlag=0;//刷回巡线模式
						CrossStateFlag=0;
						CardControlState=0;//捡卡片状态
						LeftCrossFlag=0;
						RightCrossFlag=0;
						CrossCardFlag=0;//全部重置
						turnPD.P=1;
						turnPD.D=0.5;
				}
			
			
			}

		}
		else if(CrossStateFlag==6)
		{
			if(LeftCrossFlag==1)
			{
				if(RightMax_Y_Position>y_max/2)//两边任意一边有线
					CrossStateFlag=2;
				else//两边都没线
					CrossStateFlag=1;
			}
			else if(RightCrossFlag==1)
			{
					if(LeftMax_Y_Position>y_max/2)//两边任意一边有线
					CrossStateFlag=2;
				else//两边都没线
					CrossStateFlag=1;
			}
			
		}
	}
	else if(MotionStateFlag==2)//圆环模式
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
/*****************从上面中线获取中线*********************
输入：待拟合的远处中线坐标，远处中线长度
输出：拟合后的坐标，拟合后的中线长度



****************************************************/
uint16 UPLineGainMiddle(uint16 InPoints[][2],uint16 Inlen,uint16 OutPoints[][2],uint16 Dis)
{
	//对前15个点不使用最小二乘法拟合，直接赋值即可
	uint16 len=0;
	uint16 dis=0;
	if(Inlen>10)
		dis=3;
	for(uint16 i=0;i<Inlen-dis;i++)//抛弃上面3个点
	{
			OutPoints[i][0]=InPoints[clip(Inlen-i,0,Inlen-1)][0]+Dis;
			OutPoints[i][1]=InPoints[clip(Inlen-i,0,Inlen-1)][1];
			len++;
	}
	return len;

}
//对获取的边界处理
void LineTreatment()
{
	
		int leftlen,Rightlen;
		LeftMax_Y_Position=0;
		RightMax_Y_Position=0;
		//边线提取
		
		LeftPointsNum=FindLetfSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);//得到左边线坐标
		RightPointsNum=FindRightSideLine(Imagecopy,Image_W,Image_H,Image_W/2,Image_H-5);
		//边线滤波
		AvargeLeftPointsNum=AverageFilter(LeftPoints,LeftPointsNum,AvargeLeftPoints,&LeftMax_Y_Position,7);//边线滤波
		AvargeRightPointsNum=AverageFilter(RightPoints,RightPointsNum,AvargeRightPoints,&RightMax_Y_Position,7);//边线滤波
	
	
		//提取两边角点
		FindAngle(AvargeLeftPoints,AvargeLeftPointsNum,LeftAngleMaxPoint,&LeftAngleMax,&LeftAngleMaxPosition,15);//提取左边角点
		FindAngle(AvargeRightPoints,AvargeRightPointsNum,RightAngleMaxPoint,&RightAngleMax,&RightAngleMaxPosition,15);//提取右边角点
		
//		//元素判断，判断是否存在十字或者圆环
		ElementJudge();
//	
		if(MotionStateFlag==0)//常规巡线模式
		{	
			//判断由左边线提取还是右边线提取,如果两侧边线都存在则默认右边线提取
			if(RightMax_Y_Position>LeftMax_Y_Position)
				MiddleLineFlag=1;
			else if(LeftMax_Y_Position>RightMax_Y_Position)
				MiddleLineFlag=0;
			else 
				MiddleLineFlag=1;

					//重采样
			
			
		
			if(MiddleLineFlag==0)
			{
				RasampleLeftPointsLen=ResamplePoints(AvargeLeftPoints,AvargeLeftPointsNum,RasampleLeftPoints,LeftPointsNum,PointsDist);//等距采样,等距采样的长度不能超过原始数据的长度
				MiddlePointsNum=LeftGainMidden(RasampleLeftPoints,RasampleLeftPointsLen,TrackDeviation,1);//从左线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
			else 
			{	
				RasampleRightPointsLen=ResamplePoints(AvargeRightPoints,AvargeRightPointsNum,RasampleRightPoints,RightPointsNum,PointsDist);//等距采样,等距采样的长度不能超过
				MiddlePointsNum=RightGainMidden(RasampleRightPoints,RasampleRightPointsLen,TrackDeviation,1);//从右边线提取赛道
				RasampleMiddlePointsLen=ResamplePoints(MiddlePoints,MiddlePointsNum,ResampleMiddlePoints,MiddlePointsNum,PointsDist);
			}
		}
//		//十字模式
//		/********十字模式三个阶段*******
//			1,有线
//			2,丢线
//			3,有线
//		
//		****************************/
		else if(MotionStateFlag==1)
		{
			CrossCardImage();
			
					//使用远处边线拟合中线
		}	
		else if(MotionStateFlag==2)//圆环处理
		{
			if(CircularRingDirection==0)//左圆环
			{
				LeftCircularRing_ImageHandle();
			}
			else//右圆环
			{
				RightCircularRing_ImageHandle();
			}
		
		
		}
		
}