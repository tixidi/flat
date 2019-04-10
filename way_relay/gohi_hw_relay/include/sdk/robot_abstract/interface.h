#ifndef INTERFACE_H
#define INTERFACE_H




typedef  struct{
     short int  temp[5];
     short int  z_speed;
     short int  y_speed;
     short int  x_speed;
}CarSpeedConfig;    



typedef  struct{
     short int  temp[5];
     short int  position;
     short int  type;
     short int  speed;
}CarPositionConfig; 



typedef  struct{
     short int  temp[5];
     short int  m3_speed;
     short int  m2_speed;
     short int  m1_speed;

}CarSingleSpeedConfig; 

typedef  struct{
     short int  temp[5];
     short int  position_Z;
     short int  position_Y;
     short int  position_X;
}CarGLobalPositionConfig; 

typedef  struct{
     short int  temp[6];   
     short int  laser_range_R;
     short int  laser_range_L;
}LaserRangeConfig; 


#endif 
