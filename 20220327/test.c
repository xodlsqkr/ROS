#include <stdio.h>
#include <math.h>

int main(void){
    int x1=2,x2=4,x3=6,x4=8;
    int y1=81,y2=93,y3=91,y4=97;
    float x0 = (x1+x2+x3+x4)/4.0;
    float y0 = (y1+y2+y3+y4)/4.0;
    printf("x의 평균값 : %f\n",x0);
    printf("y의 평균값 : %f\n",y0);
    float divisor = (x1-x0)*(y1-y0)+(x2-x0)*(y2-y0)+(x3-x0)*(y3-y0)+(x4-x0)*(y4-y0);
    float dividend = pow(x1-x0,2)+pow(x2-x0,2)+pow(x3-x0,2)+pow(x4-x0,2);
    printf("분모 : %f\n",divisor);
    printf("분자 : %f\n",dividend);
    float x = divisor / dividend;
    float y = y0 - (x0*x);
    printf("기울기 a=%f\n",x);
    printf("y절편 b=%f\n",y);
    return 0;
}
