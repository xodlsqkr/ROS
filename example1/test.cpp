#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace std;

int main(int argc, char* argv[])
{
	printf("%s+%s = %f\n",argv[1],argv[2],atof(argv[1])+atof(argv[2]));
	printf("%s-%s = %f\n",argv[1],argv[2],atof(argv[1])-atof(argv[2]));
	printf("%s*%s = %f\n",argv[1],argv[2],atof(argv[1])*atof(argv[2]));
	printf("%s/%s = %f\n",argv[1],argv[2],atof(argv[1])/atof(argv[2]));
	return 1;
}
