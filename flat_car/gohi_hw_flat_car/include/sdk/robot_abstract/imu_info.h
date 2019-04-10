#ifndef IMU_PARAMETERS_H
#define IMU_PARAMETERS_H

typedef struct{
    unsigned short int year;
    unsigned char month;
    unsigned char date;
    unsigned char hour;
    unsigned char min;
    unsigned char sec;
}UtcTime;

typedef  struct{
    float  pitch;
    float  roll;
    float  yaw;
    float bar_altitude;   //unit : m
    float magnetic_angle;
}IMUSensorData;

typedef  struct{
    short int  pitch;
    short int  roll;
    short int  yaw;
}IMUEulerData;

////NMEA 0183 协议解析后数据存放结构体
//typedef struct
//{
//    unsigned char svnum;					      //可见卫星数
//    nmea_slmsg slmsg[12];		//最多12颗卫星
//    nmea_utc_time utc;			//UTC时间
//    unsigned int latitude;				//纬度 分扩大100000倍,实际要除以100000
//    unsigned char nshemi;					//北纬/南纬,N:北纬;S:南纬
//    unsigned int longitude;		  //经度 分扩大100000倍,实际要除以100000
//    unsigned char ewhemi;					//东经/西经,E:东经;W:西经
//    unsigned char gpssta;					//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
//    unsigned char posslnum;				//用于定位的卫星数,0~12.
//    unsigned char possl[12];				//用于定位的卫星编号
//    unsigned char fixmode;					//定位类型:1,没有定位;2,2D定位;3,3D定位
//    unsigned short int pdop;					  //位置精度因子 0~500,对应实际值0~50.0
//    unsigned short int hdop;					  //水平精度因子 0~500,对应实际值0~50.0
//    unsigned short int vdop;					  //垂直精度因子 0~500,对应实际值0~50.0

//    int altitude;			 	//海拔高度,放大了10倍,实际除以10.单位:0.1m
//    unsigned short int speed;					//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
//}nmea_msg;

typedef  struct{
    UtcTime uct_time;
    unsigned char satellite_num;
    float altitude;     //unit : m
    float ground_speed;   //unit: m/s
    unsigned int latitude;         //纬度 分扩大100000倍,实际要除以100000
    unsigned char nshemi;	     //北纬/南纬,N:北纬;S:南纬
    unsigned int longitude;	     //经度 分扩大100000倍,实际要除以100000
    unsigned char ewhemi;	     //东经/西经,E:东经;W:西经
}GPSData;




typedef  struct{
    short int              write_to_reg_data8;         //rfid 第1字节数据
    short int              write_to_reg_data7;         //rfid 第2字节数据
    short int              write_to_reg_data6;         //rfid 第3字节数据
    short int              write_to_reg_data5;         //rfid 第4字节数据
    short int              write_to_reg_data4;         //rfid 第5字节数据
    short int              write_to_reg_data3;         //rfid 第6字节数据
    short int              write_to_reg_data2;         //rfid 第7字节数据
    short int              write_to_reg_data1;         //rfid 第8字节数据

}RfidWriteRegData;



typedef  struct{
    short int              read_from_reg_data8;         //rfid 第1字节数据
    short int              read_from_reg_data7;         //rfid 第2字节数据
    short int              read_from_reg_data6;         //rfid 第3字节数据
    short int              read_from_reg_data5;         //rfid 第4字节数据
    short int              read_from_reg_data4;         //rfid 第5字节数据
    short int              read_from_reg_data3;         //rfid 第6字节数据
    short int              read_from_reg_data2;         //rfid 第7字节数据
    short int              read_from_reg_data1;         //rfid 第8字节数据
    short int              singnal_intensity; //信号强度指示
    short int              read_state;        //读状态
    short int              write_state;       //写状态 
}RfidReadRegData;



typedef  struct{
    short int              CF;

}SET_MOT3_SPEED;

#endif // IMU_PARAMETERS_H
