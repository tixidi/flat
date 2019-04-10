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

typedef  struct{
     short int  temp[7];   
     short int  type;
}AckToPadDataType; 

typedef struct {
    short int battery_capacity_percentage;
    short int total_voltage;
    short int battery_health;
    short int current_capacity;
}BmsBatteryDate;


typedef struct {
     short int mot1_error;
     short int mot2_error;
     short int mot3_error;
     short int mot4_error;
     short int mot5_error;
     short int mot6_error;

}MoterErrorState;

typedef struct {
     short int mot1_speed;
     short int mot2_speed;
     short int mot3_speed;
     short int mot4_speed;
     short int mot5_speed;
     short int mot6_speed;

}MoterSpeedState;
#endif 
