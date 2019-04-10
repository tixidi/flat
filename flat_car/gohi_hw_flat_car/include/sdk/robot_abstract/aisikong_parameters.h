#ifndef AISIKONG_PARAMETERS_H
#define AISIKONG_PARAMETERS_H

enum IF485{
    YES,
    NO,
    OTHERS};


typedef  struct{
     float servo1;
     float servo2;
     float servo3;
     float servo4;
     float servo5;
     float servo6;
}AiSiKongMotorDOFVector;

typedef  struct{
     float position1;
     float position2;
     float position3;
     float position4;
     float position5;
     float position6;
}AiSiKongMotorRTPosition;

typedef  struct{
     float position1;
     float position2;
     float position3;
     float position4;
     float position5;
     float position6;
}AiSiKongMotorRTPositionLast;

typedef  struct{
     float position1;
     float position2;
     float position3;
     float position4;
     float position5;
     float position6;
}AiSiKongMotorRTPositionDif;

typedef  struct{
    unsigned char  error1;
    unsigned char  error2;
    unsigned char  error3;
    unsigned char  error4;
    unsigned char  error5;
    unsigned char  error6;
}AiSiKongMotorErrorState;

typedef  struct{
    unsigned char  posComp1;
    unsigned char  posComp2;
    unsigned char  posComp3;
    unsigned char  posComp4;
    unsigned char  posComp5;
    unsigned char  posComp6;
}AiSiKongPositionCompleteState;


typedef  struct{
    unsigned char  brake1;
    unsigned char  brake2;
    unsigned char  brake3;
    unsigned char  brake4;
    unsigned char  brake5;
    unsigned char  brake6;
}AiSiKongBrakeConfig;



typedef  struct{
    short int      positionPhaseChange ;
    short int      positiontype;
    short int      positionH;
    short int      positionL;

    
}AiSiKongPositionConfig;

typedef  struct{
    short int      positionPhaseChange ;
    short int      positiontype;
    int            position;
    
}AiSiKongPositionSet;



typedef  struct{
    IF485 type;
    float speed_low_filter;
    unsigned char dof;
    unsigned char imu_fusion_enalbe;
    unsigned char control_enable;
}AiSiKongMotorParameters;


typedef struct {
        float  d_past_angle1;   //degree/s
        float  d_past_angle2;
        float  d_past_angle3;
        float  d_past_angle4;
        float  d_past_angle5;  //recording d angle for robot coordinate calculation
        float  d_past_angle6;  //recording d angle for robot coordinate calculation
        
    }AiSiKongMotorDPastAngle;



typedef struct{

     short int MotorA_State;
     short int MotorB_State;
     short int MotorA_Dir;
     short int MotorB_Dir;
     short int MotorA_rpmValue;
     short int MotorB_rpmValue;

}XingSongMotorControl;
    


#endif 
