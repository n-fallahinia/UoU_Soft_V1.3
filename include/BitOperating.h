
#ifndef BITOPERATING_H
#define BITOPERATING_H
//#include <conio.h>
#include <iostream>

using namespace std;


//All index start from 0
#define BIT(x) (1 << (x))							//Set bit to 1 at x and the rest as 0
#define SETBITS(x,y) ((x) |= (y))					//Set bit to 1 as the bit in y is 1 
#define CLEARBITS(x,y) ((x) &= (~(y)))				//Set bit to 0 as the bit in y is 1
#define SETBIT(x,y) SETBITS((x), (BIT((y))))		//Set a bit y to 1  
#define CLEARBIT(x,y) CLEARBITS((x), (BIT((y))))
#define BITSET(x,y) ((x) & (BIT(y)))
#define BITCLEAR(x,y) !BITSET((x), (y))
#define BITSSET(x,y) (((x) & (y)) == (y))
#define BITSCLEAR(x,y) (((x) & (y)) == 0)
#define BITVAL(x,y) (((x)>>(y)) & 1) 

void showBits(unsigned char);
int countOnes(unsigned char);
#endif
