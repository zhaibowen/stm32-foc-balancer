#include "foc_utils.h"

// int array instead of float array 
// 4x200 points per 360 deg
// 2x storage save (int 2Byte float 4 Byte )
// sin*10000
const int sine_array[200] = {0,79,158,237,316,395,473,552,631,710,789,867,946,1024,1103,1181,1260,1338,1416,1494,1572,1650,1728,1806,1883,1961,2038,2115,2192,2269,2346,2423,2499,2575,2652,2728,2804,2879,2955,3030,3105,3180,3255,3329,3404,3478,3552,3625,3699,3772,3845,3918,3990,4063,4135,4206,4278,4349,4420,4491,4561,4631,4701,4770,4840,4909,4977,5046,5113,5181,5249,5316,5382,5449,5515,5580,5646,5711,5775,5839,5903,5967,6030,6093,6155,6217,6279,6340,6401,6461,6521,6581,6640,6699,6758,6815,6873,6930,6987,7043,7099,7154,7209,7264,7318,7371,7424,7477,7529,7581,7632,7683,7733,7783,7832,7881,7930,7977,8025,8072,8118,8164,8209,8254,8298,8342,8385,8428,8470,8512,8553,8594,8634,8673,8712,8751,8789,8826,8863,8899,8935,8970,9005,9039,9072,9105,9138,9169,9201,9231,9261,9291,9320,9348,9376,9403,9429,9455,9481,9506,9530,9554,9577,9599,9621,9642,9663,9683,9702,9721,9739,9757,9774,9790,9806,9821,9836,9850,9863,9876,9888,9899,9910,9920,9930,9939,9947,9955,9962,9969,9975,9980,9985,9989,9992,9995,9997,9999,10000,10000};

/***************************************************************************/
// function approximating the sine calculation by using fixed size array
// ~40us (float array)
// ~50us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _sin(float a){
  if(a < _PI_2){
    //return sine_array[(int)(199.0*( a / (_PI/2.0)))];
    //return sine_array[(int)(126.6873* a)];           // float array optimized
    return 0.0001*sine_array[_round(126.6873* a)];      // int array optimized
  }else if(a < _PI){
    // return sine_array[(int)(199.0*(1.0 - (a-_PI/2.0) / (_PI/2.0)))];
    //return sine_array[398 - (int)(126.6873*a)];          // float array optimized
    return 0.0001*sine_array[398 - _round(126.6873*a)];     // int array optimized
  }else if(a < _3PI_2){
    // return -sine_array[(int)(199.0*((a - _PI) / (_PI/2.0)))];
    //return -sine_array[-398 + (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[-398 + _round(126.6873*a)];      // int array optimized
  } else {
    // return -sine_array[(int)(199.0*(1.0 - (a - 3*_PI/2) / (_PI/2.0)))];
    //return -sine_array[796 - (int)(126.6873*a)];           // float array optimized
    return -0.0001*sine_array[796 - _round(126.6873*a)];      // int array optimized
  }
}
/***************************************************************************/
// function approximating cosine calculation by using fixed size array
// ~55us (float array)
// ~56us (int array)
// precision +-0.005
// it has to receive an angle in between 0 and 2PI
float _cos(float a){
  float a_sin = a + _PI_2;
  a_sin = a_sin > _2PI ? a_sin - _2PI : a_sin;
  return _sin(a_sin);
}
/***************************************************************************/
// normalizing radian angle to [0,2PI]
float _normalizeAngle(float angle){
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}
/***************************************************************************/
// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}
/***************************************************************************/
// square root approximation function using 
// https://reprap.org/forum/read.php?147,219210
// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float _sqrtApprox(float number) {//low in fat
  long i;
  float y;
  // float x;
  // const float f = 1.5F; // better precision

  // x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i; 
  // y = y * ( f - ( x * y * y ) ); // better precision
  return number * y;
}
/***************************************************************************/

float _atan2(float y,float x)
{
  float absx, absy;
  absy = fabs(y);
  absx = fabs(x);
  short octant = ((x<0) << 2) + ((y<0) << 1 ) + (absx <= absy);
  switch (octant) {
    case 0: {
        if (x == 0 && y == 0)
          return 0;
        float val = absy/absx;
        return (_PI_4_P_0273 - 0.273*val)*val; //1st octant
       }
    case 1:{
        if (x == 0 && y == 0)
          return 0.0;
        float val = absx/absy;
        return _PI_2 - (_PI_4_P_0273 - 0.273*val)*val; //2nd octant
      }
    case 2: {
        float val =absy/absx;
        return -(_PI_4_P_0273 - 0.273*val)*val; //8th octant
      }
    case 3: {
        float val =absx/absy;
        return -_PI_2 + (_PI_4_P_0273 - 0.273*val)*val;//7th octant
      }
    case 4: {
        float val =absy/absx;
        return  _PI - (_PI_4_P_0273 - 0.273*val)*val;  //4th octant
      }
    case 5: {
        float val =absx/absy;
        return  _PI_2 + (_PI_4_P_0273 - 0.273*val)*val;//3rd octant
      }
    case 6: {
        float val =absy/absx;
        return -_PI + (_PI_4_P_0273 - 0.273*val)*val; //5th octant
      }
    case 7: {
        float val =absx/absy;
        return -_PI_2 - (_PI_4_P_0273 - 0.273*val)*val; //6th octant
      }
    default:
      return 0.0;
    }
}
