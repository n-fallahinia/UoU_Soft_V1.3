#include "BitOperating.h"

void showBits(unsigned char a)
{
  int i  , k , mask;

  for( i = 7 ; i >= 0 ; i--)
  {
     mask = 1 << i;
     k = a & mask;
     if( k == 0)
        cout<<"0 ";
     else
        cout<<"1 ";
  }
  cout << "\n";
}

int countOnes(unsigned char a)
{
	int count = 0;
	for (int i = 0; i< 8; i++) 
	{
		if ( BITVAL(a, i) )
		{
			count = count + 1;
		}
	}
	return count;
}

