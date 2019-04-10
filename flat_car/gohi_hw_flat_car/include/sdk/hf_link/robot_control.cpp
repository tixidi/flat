
#include "robot_control.h"




/***********************************************************************************************************************
* Function:    void HFLink_Modbus::updataRobotSpeed(void)
*
* Scope:       public
*
* Description: send a command or data to the friend_id
*              this function is olny belongs to master
*
*
* Arguments:
*
* Return:
*
* Cpu_Time:
*
* History:
***********************************************************************************************************************/


void Robot_Control::robotSpeedSet(const float* expect_robot_speed , float* expect_motor_angle_speed)
    {
        float Motor_Line_Speed[6] ;
        robotToMotorTF(expect_robot_speed , Motor_Line_Speed) ;
        for (unsigned char i = 0  ; i < motor_num ; i++)
        {
            *(expect_motor_angle_speed + i)  = ( Motor_Line_Speed[i] / wheel_radius );
            
        }
    //  std::cerr <<"ex m1:" << *expect_motor_angle_speed  <<std::endl;
    //  std::cerr <<"ex m2:" << *(expect_motor_angle_speed+1)<<std::endl;
    }
  
void Robot_Control::robotToMotorTF(const float* robot , float* motor )
{
    *motor = -1 * (*robot) + 0 * (*(robot+1)) + body_radius * (*(robot+2)) ;
    *(motor + 1) =  1 * (*robot) + 0 * (*(robot+1)) + body_radius * (*(robot+2)) ;
    if(motor_num >= 4)
    {
         *(motor + 2) = *motor;
         *(motor + 3) = *(motor + 1);
     }
    if(motor_num >= 6)
        {
            *(motor + 4) = *motor;
            *(motor + 5) = *(motor + 1);
        }


}
    

void Robot_Control::getRobotSpeed( const float* measure_motor_angle_speed , float* measure_robot_speed)
{
    float measure_motor_line_speed[6];
    for (unsigned char i = 0 ; i < motor_num ; i++  )
    {
       *(measure_motor_line_speed + i)  = (float)*(measure_motor_angle_speed + i) * wheel_radius;
             
        // std::cerr <<"mmm:" <<  *(measure_motor_line_speed + i) <<std::endl;
        // std::cerr <<"Z:" <<robot->ask_measure_motor_speed.z<<std::endl;
    }
           
    motorToRobotTF(measure_motor_line_speed , measure_robot_speed ) ;
}
    
void Robot_Control::motorToRobotTF(const float* motor , float* robot)
{
    *robot = -0.5f * (*motor) + 0.5f* (*(motor+1));
    *(robot + 1) =  0 * (*motor)  + 0 * (*(motor+1));
    *(robot + 2) =  ( 0.5f/ body_radius ) * ( (*motor) + (*(motor+1)) );

    // std::cerr <<"X:" << *robot  <<std::endl;
    // std::cerr <<"Y:" << *(robot + 1) <<std::endl;
    // std::cerr <<"Z:" << *(robot + 2) <<std::endl;
}

            //measure_global_coordinate: x , y , z
void Robot_Control::getGlobalCoordinate(const float* d_motor_len  , float* measure_global_coordinate)
{
    float D_Global_Coordinate[3] ;
    // static int aaa=0;
    

    motorToGlobalTF(d_motor_len  , D_Global_Coordinate , *(measure_global_coordinate+2));
    *measure_global_coordinate += D_Global_Coordinate[0];
    *(measure_global_coordinate+1) += D_Global_Coordinate[1];
    *(measure_global_coordinate+2) += D_Global_Coordinate[2];

    // std::cerr <<"X1 Position  " <<  D_Global_Coordinate [0] <<std::endl;
    // std::cerr <<"Y1 Position  " <<  D_Global_Coordinate [1]<<std::endl;
    // std::cerr <<"Z1 Position  " <<  D_Global_Coordinate[2]<<std::endl;
}

void Robot_Control::motorToGlobalTF(const float* motor , float* global ,float R_theta)
{
    *global = (-cos(R_theta)/2)*(*motor) + (cos(R_theta)/2)*(*(motor+1)) ;
    *(global + 1) =  (-sin(R_theta)/2) * (*motor) +  (sin(R_theta)/2) * (*(motor+1)) ;
    *(global + 2) =  (0.5f/body_radius) * (*motor) + (0.5f/body_radius) * (*(motor+1)) ;
}