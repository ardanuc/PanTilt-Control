#include <iostream>
#include <stdio.h>

enum AxisLabelType {X_AXIS, Y_AXIS};
void printlabel (AxisLabelType mytype)
{

std::cout<<"Your Type is "<<mytype<<std::endl;
}

struct myStruct{
int x;
int y;
int z;
};

class myClass{

public:
  void  myFunction(void); 



};


void myClass::myFunction() 
{
printf("You rock man!\n");
}

int main(){
std::cout<<"You are a cool cat\n";
//WHat is yur name
int a=2;
int b=-2;
int &c =b;

void (myClass::*myfunctionpt) (void);

myClass instance;

c=4;
printf("c is %d\n",c);
printf("A is %d , But a shifted left by 2 is %d\n",a,a<<2);
printf("b is %d , But a shifted left by 2 is %d\n",b,b<<2);

myStruct f {1, 4,5};
printf("Values of f are as follows %d %d %d\n",f.x,f.y,f.z);


return (0);
}
