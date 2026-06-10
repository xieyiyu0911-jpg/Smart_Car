
#include <STC32G.H>
#include <long64.h> 
#include <INTRINS.h>   

volatile   LONGLONG  L1;
volatile   LONGLONG  L2;
volatile   LONGLONG  L3;
volatile   LONGLONG  L4;
volatile   LONGLONG  L5;
volatile   LONGLONG  L6;
volatile   LONGLONG  L7;
volatile   LONGLONG  L8;
volatile   LONGLONG  L9;
volatile   LONGLONG  L10[2];
volatile  SLONGLONG  SL1;
volatile  SLONGLONG  SL2;
volatile  SLONGLONG  SL3;
volatile  SLONGLONG  SL4;
volatile  SLONGLONG  SL5;
volatile  SLONGLONG  SL6;
volatile  SLONGLONG  SL7;
volatile  SLONGLONG  SL8;
volatile  SLONGLONG  SL9;
volatile  SLONGLONG  SL10[2];  

void main(void)
{  	
  	WORD64(L1, 0xfedcba98, 0x76543210);	  // 商   0x0e
    WORD64(L2, 0x12345678, 0x9abcdef0);   // 余数 0xf0

    WORD64(L3, 0xfedcba98, 0x76543210);	  // 商   0xdf
    WORD64(L4, 0x01234567, 0x89fedcba);   // 余数 0x01234567,0x4151ec0a
	
    WORD64(L5, 0xfedcba98, 0x76543210);	  // 商   0x01,0x04360463
    WORD64(L6, 0x00000000, 0xfabcdef9);   // 余数 0x9f4813c5

    WORD64(L7, 0x00000000, 0xfabcdef9);   // 商   0 
    WORD64(L8, 0xfedcba98, 0x76543210);	  // 余数 0xfabcdef9

  	WORD64(L9, 0x0, 0x0);    
				
    ULDIVm64(L1, L2, L10[0]);	
    ULDIVm64(L3, L4, L10[0]);	
    ULDIVm64(L5, L6, L10[0]);	
    ULDIVm64(L7, L8, L10[0]);	
    ULDIVm64(L9, L8, L10[0]);	
    ULDIVm64(L8, L9, L10[0]);	

    ULDIV64(L1, L2, L10[0]);	
    ULMOD64(L3, L4, L10[0]);	
    ULDIV64(L5, L6, L10[0]);	
    ULMOD64(L7, L8, L10[0]);	

  	WORD64(SL1, 0xfedcba98, 0x76543210);	  // -0x01234567,0x89ABCDF0 商   0x0		 						 
    WORD64(SL2, 0x12345678, 0x9abcdef0);    //                        余数 -0x01234567, 0x89ABCDF0 --> 0xFEDCBA98,0x76543210

    WORD64(SL3, 0xfedcba98, 0x76543210);	  // -0x01234567,0x89ABCDF0 商   -0x0129624E --> 0xFFFFFFFF,0xFED69DB2
    WORD64(SL4, 0x00000000, 0xfabcdef9);    //                        余数 -0x27E38C12 --> 0xFFFFFFFF,0xD81C73EE  

		WORD64(SL5, 0x0edcba98, 0x76543210);	  //                        商   -0x0d --> 0xFFFFFFFF,0xFFFFFFF3                    
    WORD64(SL6, 0xfedcba98, 0x76543210);    // -0x01234567,0x89ABCDF0 余数 -0x00123456,0x789ABCE0 --> 0x00123456,0x789ABCE0(符合Keil标准)
	
    WORD64(SL7, 0xfabcdef9, 0xfabcdef9);    // -0x05432106,0x05432107 商   0x04 
    WORD64(SL8, 0xfedcba98, 0x76543210);	  // -0x01234567,0x89ABCDF0 余数 0x00B60B67,0xDE93E947 --> 0xFF49F498,0x216C1689(符合Keil标准)

  	WORD64(SL9, 0x0, 0x0);    

		SLDIVm64(SL1, SL2, SL10[0]);
    SLDIVm64(SL3, SL4, SL10[0]);	
    SLDIVm64(SL5, SL6, SL10[0]);	
    SLDIVm64(SL7, SL8, SL10[0]);	
    SLDIVm64(SL9, SL8, SL10[0]);	
    SLDIVm64(SL8, SL9, SL10[0]);			
		
  	LMUL64(SL1, SL2, L5);		  // 积: 0x236D88FE,0x5618CF00
  	LADD64(L1, L2, L6);		    // 和: 0x11111111,0x11111100
  	LSUB64(L1, L2, L7);	 	    // 差: 0xECA8641F,0xDB975320
 
    _nop_();
//  MOV64(SL3, L1);

  while(1); 

}
