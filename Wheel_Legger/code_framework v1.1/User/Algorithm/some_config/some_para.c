#include "some_para.h"

float Find_Min_RADIAN(float measure, float ref)
{
	static float a = 0,b = 0;
	static float min;
	
	if(measure>0 && ref<0)
	{
		a= 2*PI-measure+ref;
		b=ref-measure;
		(a<-b)?(min=a):(min=b);
	}
	else if(measure<0 && ref>0)
	{
		a=-2*PI-measure+ref;
		b=ref-measure;
		(-a<b)?(min=a):(min=b);
	}
	else
	{
		min = ref - measure;
	}
	
	return min;
}

void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }

}
