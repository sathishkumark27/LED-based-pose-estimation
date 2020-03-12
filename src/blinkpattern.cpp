#include "blinkpattern.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace LED_pose_estimator
{

#define SYNC_PATTERN_16 0X00FF

#define SYNC_PATTERN_20 0X000003FF

#define BIT_PATTERN_20  0X7FFFF  // 1100 == 11111 11111 00000 00000 if we have 5 bit redundency for 1 bit we can correct it if
                                 // a 5bit pattern has >=3 1's then it will be decoded as 1s
#define BIT_PATTERN_MSB_20  19

#define FRAMES_PER_PATTERN_20  20

#define BIT_PATTERN_16  0X7FFF

#define BIT_PATTERN_MSB_16  15

#define FRAMES_PER_PATTERN_16  16

#define BIT_PATTERN_15  0X3FFF  // 5bit led pattern each bit with 3 bit redundency (span acress 3 frames) i.e 15 bits

#define BIT_PATTERN_MSB_15  14

#define FRAMES_PER_PATTERN_15  15


#define BIT_PATTERN_10  0X03FF  // SAME AS oculs rift

#define BIT_PATTERN_MSB_10  9

#define FRAMES_PER_PATTERN_10  10


#define BIT_PATTERN  BIT_PATTERN_10

#define BIT_PATTERN_MSB  BIT_PATTERN_MSB_10

#define FRAMES_PER_PATTERN  FRAMES_PER_PATTERN_10

#define SYNC_PATTERN SYNC_PATTERN_20
/*
void FillLedPatternsTable(LEDPatternsTable *GlbPatternTable)
{
    GlbPatternTable->num_patterns = 2;
    GlbPatternTable->patterns[0] = 0X13;
    GlbPatternTable->patterns[1] = 0X15;
}
*/

/*static int hamming_distance(uint16_t a, uint16_t b)
{
        uint16_t tmp = a ^ b;
        int distance = 0;
        int bit;

        for (bit = 1 << 15; bit; bit >>= 1)
                if (tmp & bit)
                        distance++;

        return distance;
}
*/

static int hamming_distance(uint32_t a, uint32_t b)
{
        uint32_t tmp = a ^ b;
        int distance = 0;
        uint32_t bit;

        for (bit = 1 << 31; bit; bit >>= 1)
                if (tmp & bit)
                        distance++;

        return distance;
}

/*
static int pattern_find_id(uint16_t pattern, int16_t *id)
{
        int i;
        LEDPatternsTable  GlbPattersTbl;
        uint16_t *patterns = GlbPattersTbl.patterns;
        unsigned num_patterns = GlbPattersTbl.num_patterns;

cout<<" pattern_find_id"<<endl;
        for (i = 0; i < num_patterns; i++) {
            cout<<" pattern_find_id i :"<<i<<endl;
                if (pattern == patterns[i]) {
                        *id = i;
                        return 2;
                }
                if (hamming_distance(pattern, patterns[i]) < 2) {
                        *id = i;
                        return 1;
                }
        }

        return -2;
}

*/


/*********************************************************************************
 * ERROR CORRECTION: if we found 2nd and 3rd bit as 11 or 00 in a nibble it can be
 * modified to 1111 or 0000 irrespective of 1st and 4th bit
 * ********************************************************************************/
void ErrCorrection(blob *blobs)
 {
    if (NULL == blobs)
    {
        cout<<"IsSyncPatternFound invalid input"<<endl;
    }

    blob *b = blobs;

    uint16_t A = 0;
    uint16_t B = 0;
    uint16_t C = 0;
    uint16_t D = 0;

    A = b->pattern & 0XF000;
    B = b->pattern & 0X0F00;
    C = b->pattern & 0X00F0;
    D = b->pattern & 0X000F;

    printf("err corr A : %x , %u \n",A, A);
    printf("err corr B : %x , %u \n",B, B);
    printf("err corr C : %x , %u \n",C, C);
    printf("err corr D : %x , %u \n",D, D);

    /* get all the bits to first nibble by right shifting to convert them to decimal  */
    A = A >> 12;
    B = B >> 8;
    C = C >> 4;

    /* check the 2nd ona d3rd bits whether they are 1 or 0
     * here A,B,C and D can be 0110 = 6 = 0X0006 or 0000 = 0 = 0X0000*/

    A = A & 0X0006;
    B = B & 0X0006;
    C = C & 0X0006;
    D = D & 0X0006;


    /*******************************************************
     * if we get x11x --> 1111, x00x -->0000
     * ******************************************************/

    if (0X0006 == A)
    {
        b->pattern |= (0x000F << 12);
    }
    else if (0x0000 == A)
    {
        b->pattern &= (0x0000 << 12);
    }

    if (0X0006 == B)
    {
        b->pattern |= (0x000F << 8);
    }
    else if (0x0000 == B)
    {
        b->pattern &= (0x0000 << 8);
    }

    if (0X0006 == C)
    {
        b->pattern |= (0x000F << 4);
    }
    else if (0x0000 == C)
    {
        b->pattern &= (0x0000 << 4);
    }

    if (0X0006 == D)
    {
        b->pattern |= 0x000F;
    }
    else if (0x0000 == D)
    {
        b->pattern &= 0x0000;
    }

    return;
 }

/*********************************************************************************
 * ERROR CORRECTION: if we found 2nd and 3rd bit as 11 or 00 in a nibble it can be
 * modified to 1111 or 0000 irrespective of 1st and 4th bit
 * ********************************************************************************/
#if 0
void DataErrCorrection(blob *blobs, int num_blobs)
 {
    if (NULL == blobs)
    {
        cout<<"DataErrCorrection invalid input"<<endl;
    }

    blob *b = NULL;


    for (b = blobs; b < blobs + num_blobs; b++)
    {
        uint16_t A = 0;
        uint16_t B = 0;
        uint16_t C = 0;
        uint16_t D = 0;

        b->patternCrrctd = b->pattern;

        A = b->patternCrrctd & 0XF000;
        B = b->patternCrrctd & 0X0F00;
        C = b->patternCrrctd & 0X00F0;
        D = b->patternCrrctd & 0X000F;

/*

        printf("data err corr pattern : %x , %u \n",b->pattern, b->pattern);
        printf("data err corr patternCrrctd : %x , %u \n",b->patternCrrctd, b->patternCrrctd);

        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);

*/

        /* get all the bits to first nibble by right shifting to convert them to decimal  */
        A = A >> 12;
        B = B >> 8;
        C = C >> 4;
/*
        cout<<"AFTER RIGHT SHFT"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);
*/

        /* check the 2nd ona d3rd bits whether they are 1 or 0
         * here A,B,C and D can be 0110 = 6 = 0X0006 or 0000 = 0 = 0X0000*/

        A = A & 0X0006;
        B = B & 0X0006;
        C = C & 0X0006;
        D = D & 0X0006;

/*
        cout<<"AFTER AND IWTH 6"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);
*/

        /*******************************************************
         * if we get x11x --> 1111, x00x -->0000
         * ******************************************************/

        if (0X0006 == A)
        {
            b->patternCrrctd |= 0xF000;
        }
        else if (0x0000 == A)
        {
            b->patternCrrctd &= 0x0FFF;
        }

        if (0X0006 == B)
        {
            b->patternCrrctd |= 0x0F00;
        }
        else if (0x0000 == B)
        {
            b->patternCrrctd &= 0xF0FF;
        }

        if (0X0006 == C)
        {
            b->patternCrrctd |= 0X00F0;
        }
        else if (0x0000 == C)
        {
            b->patternCrrctd &= 0XFF0F;
        }

        if (0X0006 == D)
        {
            b->patternCrrctd |= 0x000F;
        }
        else if (0x0000 == D)
        {
            b->patternCrrctd &= 0xFFF0;
        }
    }

    return;
 }

#endif
/*******************************************************************
 * Find the pattern using one blob, as all the LDEs are blinking
 * with same frequency(sync)  if we found sync pattern for one LED
 * we can read the data from all LEDs from the next frame onwords
 ********************************************************************/
#if 0
bool IsSyncPatternFound(blobservation *CurrOb)
{
    if (NULL == CurrOb)
    {
        cout<<"IsSyncPatternFound invalid input"<<endl;
    }

    blob *b = &CurrOb->blobs[0];

    uint16_t pattern = 0;
    unsigned HammingDist = 0xFFFF;

    //cout<<"b->led_id : "<<b->led_id<<endl;
    //cout<<"b->age : "<<b->age<<endl;

    /* Update pattern only if blob was observed previously */
    if (b->age < 1)
    {
        cout<<"bachha LED"<<endl;
        return false;
    }

    /*
     * Interpret brightness change of more than 10% as rising
     * or falling edge. Right shift the pattern and add the
     * new brightness level as MSB.
     */

    printf("b->pattern : %x \n",b->pattern);
    pattern = (b->pattern >> 1) & 0x7fff;  // 16-bit sync pattern 1111 0000 1111 0000

    //printf("pattern : %x \n",pattern);
    cout<<"b->area : "<<b->area<<endl;
    cout<<"b->last_area : "<<b->last_area<<endl;

    int asd = (b->area * 10);
    int asd1 = (b->last_area * 11);
    //cout<<"b->area by ten : "<<asd<<endl;
    //cout<<"b->last_area by eleven :"<<asd1<<endl;

    if ((b->area * 10) > (b->last_area * 11))
    {
        pattern |= (1 << 15); //16-bit pattern
        cout<<"1 at MSB"<<endl;
    }
    else if ((b->area * 11) < (b->last_area * 10))
    {
        pattern |= (0 << 15); //16-bit pattern
        cout<<"0 at MSB"<<endl;
    }
    else
    {
        pattern |= b->pattern & (1 << 15);
        cout<<"copy prv bit "<<endl;
    }

    b->pattern = pattern;

    printf("b->pattern 2 : %x \n",b->pattern);

    /*
     * Determine LED ID only if a full pattern was recorded and
     * consensus about the blinking phase is established
     */
    if (b->age < 15) //5-bit pattern
    {
       return false;
    }

    printf("b->pattern 3 : %x \n",b->pattern);

    //ErrCorrection(b);

    /*

    HammingDist = hamming_distance(b->pattern, 0X00FF);

    if (HammingDist <= 1)
    {
        return true;
    }
*/

    if (0X00FF == b->pattern)
    {
        return true;
    }
    return false;

}

#endif


bool IsSyncPatternFound(blob *blobs, int num_blobs)
{
    blob *b = NULL;
    unsigned HammingDist = 0xFFFF;

    if (NULL == blobs)
    {
        cout<<"FindBlinkPattern invalid input"<<endl;
    }

    //FindBlinkPattern(blobs, num_blobs, 0);  // no need to do data correction for sync pattern so DataFrmCnt = 0

    /*b = blobs;
    if (b->age < (FRAMES_PER_PATTERN - 1)) //16-bit sync pattern
    {
       return false;
    }*/

    /*if (0X00FF == b->pattern)
    {
        return true;
    }*/

    for (b=blobs; b < blobs + num_blobs;b++)
    {
        if (b->age < (FRAMES_PER_PATTERN - 1)) //16-bit sync pattern
        {
           continue;
        }

        HammingDist = hamming_distance(b->pattern, SYNC_PATTERN);
        //cout<<"SYNC_PATTERN : "<<SYNC_PATTERN<<endl;
        //printf("Data b->pattern : %x , %u \n",b->pattern, b->pattern);;
        cout<<"HammingDist : "<<HammingDist<<endl;


        if (HammingDist <= 1)
        {
            return true;
        }
    }
    return false;

}

void DataErrCorrectionFor20BitPattern(blob *blobs, int num_blobs)
{
    if (NULL == blobs)
    {
        cout<<"DataErrCorrection invalid input"<<endl;
    }

    blob *b = NULL;


    for (b = blobs; b < blobs + num_blobs; b++)
    {
        uint32_t A = 0;
        uint32_t B = 0;
        uint32_t C = 0;
        uint32_t D = 0;

        uint8_t NoOf1s_A = 0;
        uint8_t NoOf1s_B = 0;
        uint8_t NoOf1s_C = 0;
        uint8_t NoOf1s_D = 0;

        b->patternCrrctd = b->pattern;

        A = b->patternCrrctd & 0X000F8000; //0000 0000 0000 1111 1000 0000 0000 0000
        B = b->patternCrrctd & 0X00007C00; //0000 0000 0000 0000 0111 1100 0000 0000
        C = b->patternCrrctd & 0X000003E0; //0000 0000 0000 0000 0000 0011 1110 0000
        D = b->patternCrrctd & 0X0000001F; //0000 0000 0000 0000 0000 0000 0001 1111

        NoOf1s_A = hamming_distance(0X00000000, A);
        NoOf1s_B = hamming_distance(0X00000000, B);
        NoOf1s_C = hamming_distance(0X00000000, C);
        NoOf1s_D = hamming_distance(0X00000000, D);

/*

        printf("data err corr pattern : %x , %u \n",b->pattern, b->pattern);
        printf("data err corr patternCrrctd : %x , %u \n",b->patternCrrctd, b->patternCrrctd);

        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);


        cout<<"AFTER RIGHT SHFT"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);

        cout<<"AFTER AND IWTH 6"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);
*/

        /*******************************************************
         * if we get x11x --> 1111, x00x -->0000
         * ******************************************************/

        if (NoOf1s_A >= 3)
        {
            b->patternCrrctd |= 0X000F8000;
        }
        else
        {
            b->patternCrrctd &= ~(0X000F8000);
        }

        if (NoOf1s_B >= 3)
        {
            b->patternCrrctd |= 0X00007C00;
        }
        else
        {
            b->patternCrrctd &= ~(0X00007C00);
        }

        if (NoOf1s_C >= 3)
        {
            b->patternCrrctd |= 0X000003E0;
        }
        else
        {
            b->patternCrrctd &= ~(0X000003E0);
        }

        if (NoOf1s_D >= 3)
        {
            b->patternCrrctd |= 0X0000001F;
        }
        else
        {
            b->patternCrrctd &= ~(0X0000001F);
        }
    }

    return;

}

void CorrectPattern(blobservation *CurrOb)
{

    blob *b = NULL;
    int i;

    if (NULL == CurrOb)
    {
        cout<<"AssociateBlobs invalid input blob observation is NULL"<<endl;
        return;
    }

    //cout<<"FindBlinkPattern start"<<endl;

    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        b = &CurrOb->blobs[i];
        uint32_t A = 0;
        uint32_t B = 0;
        uint32_t C = 0;
        uint32_t D = 0;
        uint32_t E = 0;
        uint32_t F = 0;
        uint32_t G = 0;
        uint32_t H = 0;

        uint8_t NoOf1s_A = 0;
        uint8_t NoOf1s_B = 0;
        uint8_t NoOf1s_C = 0;
        uint8_t NoOf1s_D = 0;
        uint8_t NoOf1s_E = 0;
        uint8_t NoOf1s_F = 0;
        uint8_t NoOf1s_G = 0;
        uint8_t NoOf1s_H = 0;

        b->patternCrrctd = b->pattern;

        A = b->patternCrrctd & 0X00E00000; //0000 0000 1110 0000 0000 0000 0000 0000
        B = b->patternCrrctd & 0X001C0000; //0000 0000 0001 1100 0000 0000 0000 0000
        C = b->patternCrrctd & 0X00038000; //0000 0000 0000 0011 1000 0000 0000 0000
        D = b->patternCrrctd & 0X00007000; //0000 0000 0000 0000 0111 0000 0000 0000
        E = b->patternCrrctd & 0X00000E00; //0000 0000 0000 0000 0000 1110 0000 0000
        F = b->patternCrrctd & 0X000001C0; //0000 0000 0000 0000 0000 0001 1100 0000
        G = b->patternCrrctd & 0X00000038; //0000 0000 0000 0000 0000 0000 0011 1000
        H = b->patternCrrctd & 0X00000007; //0000 0000 0000 0000 0000 0000 0000 0111

        NoOf1s_A = hamming_distance(0X00000000, A);
        NoOf1s_B = hamming_distance(0X00000000, B);
        NoOf1s_C = hamming_distance(0X00000000, C);
        NoOf1s_D = hamming_distance(0X00000000, D);
        NoOf1s_E = hamming_distance(0X00000000, E);
        NoOf1s_F = hamming_distance(0X00000000, F);
        NoOf1s_G = hamming_distance(0X00000000, G);
        NoOf1s_H = hamming_distance(0X00000000, H);

/*

        printf("data err corr pattern : %x , %u \n",b->pattern, b->pattern);
        printf("data err corr patternCrrctd : %x , %u \n",b->patternCrrctd, b->patternCrrctd);

        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);


        cout<<"AFTER RIGHT SHFT"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);

        cout<<"AFTER AND IWTH 6"<<endl;
        printf("data err corr A : %x , %u \n",A, A);
        printf("data err corr B : %x , %u \n",B, B);
        printf("data err corr C : %x , %u \n",C, C);
        printf("data err corr D : %x , %u \n",D, D);
*/

        /*******************************************************
         * if we get x11x --> 1111, x00x -->0000
         * ******************************************************/

        if (NoOf1s_A >= 2)
        {
            b->patternCrrctd |= 0X00E00000;
        }
        else
        {
            b->patternCrrctd &= ~(0X00E00000);
        }

        if (NoOf1s_B >= 2)
        {
            b->patternCrrctd |= 0X001C0000;
        }
        else
        {
            b->patternCrrctd &= ~(0X001C0000);
        }

        if (NoOf1s_C >= 2)
        {
            b->patternCrrctd |= 0X00038000;
        }
        else
        {
            b->patternCrrctd &= ~(0X00038000);
        }

        if (NoOf1s_D >= 2)
        {
            b->patternCrrctd |= 0X00007000;
        }
        else
        {
            b->patternCrrctd &= ~(0X00007000);
        }

        if (NoOf1s_E >= 2)
        {
            b->patternCrrctd |= 0X00000E00;
        }
        else
        {
            b->patternCrrctd &= ~(0X00000E00);
        }
        if (NoOf1s_F >= 2)
        {
            b->patternCrrctd |= 0X000001C0;
        }
        else
        {
            b->patternCrrctd &= ~(0X000001C0);
        }
        if (NoOf1s_G >= 2)
        {
            b->patternCrrctd |= 0X00000038;
        }
        else
        {
            b->patternCrrctd &= ~(0X00000038);
        }
        if (NoOf1s_H >= 2)
        {
            b->patternCrrctd |= 0X00000007;
        }
        else
        {
            b->patternCrrctd &= ~(0X00000007);
        }
    }

    return;
}

/************************************************************************************************************
 * once we validate the correspodences we get the LED world co-ordinates and LED-ID by matching the pattern
 * **********************************************************************************************************/
bool GetLEDInfoFromPattern(blobservation *CurrOb)
{
    blob *b = NULL;
    uint16_t tmpptn = 0X0000;
    unsigned LEDInfoCount = 0;
    int i, j;
    if (NULL == CurrOb)
    {
        cout<<"GetLEDInfoFromPattern invalid input blob observation is NULL"<<endl;
        return false;
    }


    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        b = &CurrOb->blobs[i];
        if (b->age < (FRAMES_PER_PATTERN - 1)) //16-bit sync pattern
        {
           continue;
        }
        if ((b->led_id != -1))
        {
            //cout<<"GetLEDInfoFromPattern already id assigned from prv info b->worldpoint : "<<b->worldpoint<<endl;
            //cout<<"GetLEDInfoFromPattern already id assigned from prv info b->led_id : "<<b->led_id<<endl;

            LEDInfoCount++;  // this led has already found in prv frames
            continue;
        }
        unsigned HammingDist = 0xFFFF;


        //printf("GetLEDInfoFromPattern b->pattern: %x\n", b->pattern);

        for (j=0; j<NUM_OF_LEDS; j++)
        {
            tmpptn = gLEDsInfo[j].pattern;
            HammingDist = hamming_distance(b->pattern, tmpptn);
            //if (HammingDist < 2)
            if (HammingDist < 1)
            {
                b->worldpoint = gLEDsInfo[j].worldpoint;
                b->led_id = gLEDsInfo[j].LEDId;

                //printf("GetLEDInfoFromPattern b->tmpptn: %x\n", tmpptn);

                //cout<<"GetLEDInfoFromPattern b->worldpoint: "<<b->worldpoint<<endl;
                //cout<<"GetLEDInfoFromPattern b->led_id : "<<b->led_id<<endl;
                LEDInfoCount++;
                break;

            }
        }
    }
    cout<<" LEDInfoCount  :"<<LEDInfoCount<<endl;
    if (LEDInfoCount >= 4)
    {
        cout<<" found ledinfo  "<<endl;
        return true;
    }
    return false;
}

void FindColorPattern(blobservation *CurrOb)
{
    blob *b;
    int i;

    if (NULL == CurrOb)
    {
        cout<<"FindColorPattern invalid input blob observation is NULL"<<endl;
        return;
    }

    //cout<<"FindColorPattern start"<<endl;

    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        b = &CurrOb->blobs[i];
        uint16_t pattern = 0 ;

        pattern = (b->pattern >> 1) & BIT_PATTERN;  // 10-bit data

        if (b->color >= 0)  // valid color R or B
        {
            pattern |= (b->color << BIT_PATTERN_MSB);
        }
        else // invalid color copy previous color
        {
            cout<<"Invalid Color"<<endl;
            pattern |= b->pattern & (1 << BIT_PATTERN_MSB);

        }

        b->pattern = pattern;
        cout<<"blob :"<<b->Center<<endl;
        printf("b->pattern : %x , %u \n",b->pattern, b->pattern);
    }

}

void FindBlinkPattern(blobservation *CurrOb)
{
    blob *b;
    int i;

    if (NULL == CurrOb)
    {
        cout<<"FindBlinkPattern invalid input blob observation is NULL"<<endl;
        return;
    }

    //cout<<"FindBlinkPattern start"<<endl;

    for (i = 0; i < CurrOb->num_blobs; i++)
    {
        b = &CurrOb->blobs[i];
        uint16_t pattern = 0 ;
        uint32_t last_bit = 0;
        uint32_t lastbutone_bit = 0;

        /* Update pattern only if blob was observed previously */
        if (b->age < 1)
            continue;

        /*
         * Interpret brightness change of more than 10% as rising
         * or falling edge. Right shift the pattern and add the
         * new brightness level as MSB.
         */

        //printf("Data b->pattern : %x , %u \n",b->pattern, b->pattern);
        pattern = (b->pattern >> 1) & BIT_PATTERN;  // 20-bit data

        //printf("Data pattern : %x, %u \n",pattern, pattern);
        //cout<<"Data b->area : "<<b->area<<endl;
        //cout<<"Data b->last_area : "<<b->last_area<<endl;

        int asd = (b->area * 10);
        int asd1 = (b->last_area * 11);
        //cout<<"Data b->area by ten : "<<asd<<endl;
        //cout<<"Data b->last_area by eleven :"<<asd1<<endl;

        if ((b->area * 10) > (b->last_area * 11))
        {
            pattern |= (1 << BIT_PATTERN_MSB); //16-bit pattern
            //cout<<"1 AT MSB"<<endl;
        }
        else if ((b->area * 11) < (b->last_area * 10))
        {
            pattern |= (0 << BIT_PATTERN_MSB); //16-bit pattern
            //cout<<"0 at MSB"<<endl;
        }
        else
        {
            pattern |= b->pattern & (1 << BIT_PATTERN_MSB);
            //cout<<"CPOY PRV BIT "<<(b->pattern & (1 << BIT_PATTERN_MSB))<<endl;

        }

        b->pattern = pattern;

        //printf("Data b->pattern 2 : %x , %u \n",b->pattern, b->pattern);

    }
  /* if getting any errors suspect the phase calcultion */

}

} //namespace end
