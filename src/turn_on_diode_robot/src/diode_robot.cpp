#include "diode_robot.h"
#include "Quaternion_Solution.h"
int cnt=0;
sensor_msgs::Imu Mpu6050;//Instantiate an IMU object //实例化IMU对象 


double angle;

double getAngle(double x, double y)
{
  double angle = asin (y / sqrt(x*x + y*y));
  if (x < 0){
    angle = M_PI - angle;
  }
  return angle;
}


double angle_init;
int initialized_ = 0;
double angle_;

#include <std_msgs/Int16.h>
bool mode = false;
void mode_Callback(const std_msgs::Int16& msg)
{
  if (msg.data == 1){
    mode = true;
  }
  if (msg.data == 0){
    mode = false;
  }
}

/**************************************
Date: January 28, 2021
Function: The main function, ROS initialization, creates the Robot_control object through the Turn_on_robot class and automatically calls the constructor initialization
功能: 主函数，ROS初始化，通过turn_on_robot类创建Robot_control对象并自动调用构造函数初始化
***************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "diode_robot"); //ROS initializes and sets the node name //ROS初始化 并设置节点名称 
  turn_on_robot Robot_Control; //Instantiate an object //实例化一个对象
  Robot_Control.Control(); //Loop through data collection and publish the topic //循环执行数据采集和发布话题等操作
  return 0;  
} 
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;
/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::Twist &twist_aux)   // 0为0x7D 1～2为0 3～6为x速度下位机需要的是厘米每秒 7～8为crc16校验 
                                                                              // 9～12为y速度 13～16为z角速度 17～18为crc16校验 19为0x8c
{
  short  transition;  //intermediate variable //中间变量
  float2uchar need_return_vel_x;
  float2uchar need_return_vel_y;
  float2uchar need_return_vel_z;
  // Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B //帧头0X7B
  // Send_Data.tx[1] = 0; //set aside //预留位
  // Send_Data.tx[2] = 0; //set aside //预留位
  Send_Data.tx[0]=0x7D; //frame head 0x7B //帧头0X7B
  Send_Data.tx[1] = 0; //set aside //预留位
  Send_Data.tx[2] = 0; //set aside //预留位
  //The target velocity of the X-axis of the robot
  //机器人x轴的目标线速度
  //transition=0;
  // transition = twist_aux.linear.x*1000; //将浮点数放大一千倍，简化传输
  // Send_Data.tx[4] = transition;     //取数据的低8位
  // Send_Data.tx[3] = transition>>8;  //取数据的高8位

  double x = twist_aux.linear.x;
  double y = twist_aux.linear.y;
  double rotation = 0;
  double cur_angel = 0;
  double len = 0;

  if (!mode){
    if (x != 0 || y != 0){
      len = sqrt(x*x + y*y);
      cur_angel = getAngle(x, y) - angle;
    }

    // ROS_INFO("%.2f, %.2f, %.2f", angle_ - angle_init, angle, angle_init);
    
    x = len * cos(cur_angel);
    y = len * sin(cur_angel);
    rotation = 0.5;
  }
  else{
    x = 0;
    y = 0;
    rotation = 2.0;
  }
  

  // ROS_INFO("%.2f, %.2f", x, y);

  // need_return_vel_x.f = twist_aux.linear.x*100;
  need_return_vel_x.f = x*100;
  Send_Data.tx[3] = need_return_vel_x.c[0];
  Send_Data.tx[4] = need_return_vel_x.c[1];
  Send_Data.tx[5] = need_return_vel_x.c[2];
  Send_Data.tx[6] = need_return_vel_x.c[3];

  // need_return_vel_y.f = twist_aux.linear.y*100;
  need_return_vel_y.f = y*100;
  Append_CRC16_Check_Sum(Send_Data.tx , 9);
  Send_Data.tx[9] = need_return_vel_y.c[0];
  Send_Data.tx[10] = need_return_vel_y.c[1];
  Send_Data.tx[11] = need_return_vel_y.c[2];
  Send_Data.tx[12] = need_return_vel_y.c[3];

  // need_return_vel_z.f = twist_aux.angular.z;
  need_return_vel_z.f = rotation;
  Send_Data.tx[13] = need_return_vel_z.c[0];
  Send_Data.tx[14] = need_return_vel_z.c[1];
  Send_Data.tx[15] = need_return_vel_z.c[2];
  Send_Data.tx[16] = need_return_vel_z.c[3];
  Append_CRC16_Check_Sum(Send_Data.tx , 19);
  // Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function //BCC校验位，规则参见Check_Sum函数
  Send_Data.tx[19]=0x8c; //frame tail 0x7D //帧尾0X7D
  try
  {
    int bytes = 0;
    bytes=Stm32_Serial.write(Send_Data.tx,20); //Sends data to the downloader via serial port //通过串口向下位机发送数据 
    if(bytes != 0 && need_return_vel_x.f!=0){
      // cout<<"bytes:"<<bytes<<endl;
      // cout<<"velback_x:"<<need_return_vel_x.f<<endl;
      // cout<<"velback_y:"<<need_return_vel_y.f<<endl;
      // cout<<"velback_z:"<<need_return_vel_z.f<<endl;
      // cout<<need_return_vel_x.f<<endl;
      // printf("0X%x ",Send_Data.tx[19]);
    }
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
}
/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  sensor_msgs::Imu Imu_Data_Pub; //Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = ros::Time::now(); 
  Imu_Data_Pub.header.frame_id = gyro_frame_id; //IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack 
                                                //IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; //A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y; 
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;
  // cout<<"orientation.x:"<<Imu_Data_Pub.orientation.x<<endl;
  // cout<<"orientation.y:"<<Imu_Data_Pub.orientation.y<<endl;
  // cout<<"orientation.z:"<<Imu_Data_Pub.orientation.z<<endl;
  Imu_Data_Pub.orientation_covariance[0] = 1e6; //Three-axis attitude covariance matrix //三轴姿态协方差矩阵
  Imu_Data_Pub.orientation_covariance[4] = 1e6;
  Imu_Data_Pub.orientation_covariance[8] = 1e-6;
  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; //Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;
  // cout<<"angular_velocity.x:"<<Imu_Data_Pub.angular_velocity.x<<endl;
  // cout<<"angular_velocity.y:"<<Imu_Data_Pub.angular_velocity.y<<endl;
  // cout<<"angular_velocity.z:"<<Imu_Data_Pub.angular_velocity.z<<endl;
  // ROS_INFO("angular_velocity.x:%f\n",Imu_Data_Pub.angular_velocity.x);
  // ROS_INFO("angular_velocity.y:%f\n",Imu_Data_Pub.angular_velocity.y);
  // ROS_INFO("angular_velocity.z:%f\n",Imu_Data_Pub.angular_velocity.z);
  Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; //Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
  Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
  Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; //Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y; 
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;  
  // cout<<"linear_acceleration.x:"<<Imu_Data_Pub.linear_acceleration.x<<endl;
  // cout<<"linear_acceleration.y:"<<Imu_Data_Pub.linear_acceleration.y<<endl;
  // cout<<"linear_acceleration.z:"<<Imu_Data_Pub.linear_acceleration.z<<endl;
  // ROS_INFO("linear_acceleration.x:%f\n",Imu_Data_Pub.linear_acceleration.x);
  // ROS_INFO("linear_acceleration.y:%f\n",Imu_Data_Pub.linear_acceleration.y);
  // ROS_INFO("linear_acceleration.z:%f\n",Imu_Data_Pub.linear_acceleration.z);

  imu_publisher.publish(Imu_Data_Pub); //Pub IMU topic //发布IMU话题
}
/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转换为四元数进行表达
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

    nav_msgs::Odometry odom; //Instance the odometer topic data //实例化里程计话题数据
    odom.header.stamp = ros::Time::now(); 
    odom.header.frame_id = odom_frame_id; // Odometer TF parent coordinates //里程计TF父坐标
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数

    odom.child_frame_id = robot_frame_id; // Odometer TF subcoordinates //里程计TF子坐标
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴角速度 

    //There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
    //这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
      //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
      //如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
      memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    else
      //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
      //如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
      memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
      memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));       
    odom_publisher.publish(odom); //Pub odometer topic //发布里程计话题
}
/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
  unsigned char check_sum=0,k;
  
  if(mode==0) //Receive data mode //接收数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
     }
  }
  if(mode==1) //Send data mode //发送数据模式
  {
   for(k=0;k<Count_Number;k++)
    {
     check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
     }
  }
  return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}
/**************************************
Date: May 11, 2023
功能: CRC发送校验中间函数
输入参数： 
***************************************/
uint16_t CRC_INIT = 0xffff;
const unsigned char CRC8_INIT = 0xff;
uint16_t turn_on_robot::Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t chData;

	if (pchMessage == NULL)
	{
		return 0xFFFF;
	}

	while (dwLength--)
	{
		chData = *pchMessage++;
		(wCRC) = ((uint16_t)(wCRC) >> 8) ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
	}

	return wCRC;
}
unsigned char turn_on_robot::Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8)
{
	unsigned char ucIndex;

	while (dwLength--)
	{
		ucIndex = ucCRC8 ^ (*pchMessage++);
		ucCRC8 = CRC8_TAB[ucIndex];
	}

	return (ucCRC8);
}
/**************************************
Date: May 11, 2023
功能: CRC发送校验
输入参数： 
***************************************/
void turn_on_robot::Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wCRC = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return;
	}

	wCRC = Get_CRC16_Check_Sum((uint8_t *)pchMessage, dwLength - 2, CRC_INIT);
	pchMessage[dwLength - 2] = (uint8_t)(wCRC & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((wCRC >> 8) & 0x00ff);
}
void turn_on_robot::Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucCRC = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return;

	ucCRC = Get_CRC8_Check_Sum((unsigned char *)pchMessage, dwLength - 1, CRC8_INIT);
	pchMessage[dwLength - 1] = ucCRC;
}
/**************************************
Date: May 11, 2023
功能: CRC接收校验
输入参数： 
***************************************/
unsigned int turn_on_robot::Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength)
{
	unsigned char ucExpected = 0;

	if ((pchMessage == 0) || (dwLength <= 2)) return 0;

	ucExpected = Get_CRC8_Check_Sum(pchMessage, dwLength - 1, CRC8_INIT);
	return (ucExpected == pchMessage[dwLength - 1]);
}
uint32_t turn_on_robot::Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength)
{
	uint16_t wExpected = 0;

	if ((pchMessage == NULL) || (dwLength <= 2))
	{
		return 0;
	}

	wExpected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC_INIT);
	return ((wExpected & 0xff) == pchMessage[dwLength - 2] && ((wExpected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}
/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()         // 0为0XA5 1～12为角速度 13～24为加速度 25～36为速度 37～38为crc16校验 39为0XB2 
{
  int bytes;
  bytes = Stm32_Serial.read(rdata,1);
  if(rdata[0]==0XA5){
  bytes = Stm32_Serial.read(rdata+1,31);
  bytes = Stm32_Serial.read(rdata+32,8);
  //  printf("0x%02X ", (unsigned char)rdata[0]);
  // cout<<hex<<rdata[0];
  //  printf("0X%x ",rdata[0]);
  // printf("0X%x ",rdata[59]);
    if (rdata[0]==0XA5 && Verify_CRC16_Check_Sum(rdata,39) && rdata[39]==0XB2)
    {
        getGyro(&rdata[1]);
        getAcc(&rdata[13]);
        get_xyspeed_and_z(&rdata[25]);
        Robot_Vel.X = vel[0];
        Robot_Vel.Y = vel[1];
        Robot_Vel.Z = vel[2];
        // cout<<"Robot_Vel.X:"<<Robot_Vel.X<<endl;
        // cout<<"Robot_Vel.Y:"<<Robot_Vel.Y<<endl;
        // cout<<"Robot_Vel.Z:"<<Robot_Vel.Z<<endl;
        Mpu6050_Data.gyros_x_data = -gyro[0];
        Mpu6050_Data.gyros_y_data = -gyro[1];
        Mpu6050_Data.gyros_z_data = gyro[2];
        Mpu6050_Data.accele_x_data = -acc[0];
        Mpu6050_Data.accele_y_data = -acc[1];
        Mpu6050_Data.accele_z_data = acc[2];          //TODO:为什么要倒？？？
        // Mpu6050_Data.gyros_x_data = 0;?
        // cout<<"Robot_gyro_x:"<<gyro[0]*57.3<<endl;
        // cout<<"Robot_gyro_y:"<<gyro[1]*57.3<<endl;
        // cout<<"Robot_gyro_z:"<<gyro[2]*57.3<<endl;
        Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data ;
        Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data ;
        Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data ;
        // cout<<"Mpu6050_Data.accele_x_data:"<<Mpu6050_Data.accele_x_data<<endl;
        // cout<<"Mpu6050_Data.accele_x_data:"<<Mpu6050_Data.accele_x_data<<endl;
        // cout<<"Mpu6050_Data.accele_x_data:"<<Mpu6050_Data.accele_x_data<<endl;
        // cout<<"linear_acceleration.x:"<<Mpu6050.linear_acceleration.x<<endl;
        // cout<<"linear_acceleration.y:"<<Mpu6050.linear_acceleration.y<<endl;
        // cout<<"linear_acceleration.z:"<<Mpu6050.linear_acceleration.z<<endl;
        // const std::type_info& typeInfo = typeid(Mpu6050.linear_acceleration.z);
        // ROS_INFO("Variable type: %s", typeInfo().name());
        Mpu6050.angular_velocity.x =  Mpu6050_Data.gyros_x_data ;
        Mpu6050.angular_velocity.y =  Mpu6050_Data.gyros_y_data ;
        Mpu6050.angular_velocity.z =  Mpu6050_Data.gyros_z_data ;
        // cout<<"Mpu6050.angular_velocity.x:"<<Mpu6050.angular_velocity.x<<endl;
        // cout<<"Mpu6050.angular_velocity.y:"<<Mpu6050.angular_velocity.y<<endl;
        // cout<<"Mpu6050.angular_velocity.z:"<<Mpu6050.angular_velocity.z<<endl;
        return true;
    }else 
      return false;
  }else
    return false;
}

bool turn_on_robot::get_xyspeed_and_z(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    vel[0] = exchange_data(f1,process_float_data[24]);
    vel[1] = exchange_data(f2,process_float_data[28]);
    vel[2] = exchange_data(f3,process_float_data[32]);
    return true;
}

/**
 * @brief 解算角速度数据
 * 
 * @param data 角速度首地址指针
 * @return
 */

bool turn_on_robot::getGyro(unsigned char *data)
{    
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    gyro[0] = exchange_data(f1,process_float_data[0]);
    gyro[1] = exchange_data(f2,process_float_data[4]);
    gyro[2] = exchange_data(f3,process_float_data[8]);

    if (initialized_ >= 5 && initialized_ < 6){
      angle_init = gyro[0];
    }
    angle_ = gyro[0];
    initialized_++;

    return true;
}

/**
 * @brief 解算加速度数据
 * 
 * @param data 加速度首地址指针
 * @return
 */
bool turn_on_robot::getAcc(unsigned char *data)
{
    unsigned char* f1 = &data[0];
    unsigned char* f2 = &data[4];
    unsigned char* f3 = &data[8];

    acc[0] = exchange_data(f1,process_float_data[12]);
    acc[1] = exchange_data(f2,process_float_data[16]);
    acc[2] = exchange_data(f3,process_float_data[20]);
    return true;
}
/**
 * @brief 将4个uchar转换为float
 * @param data data首地址指针
 * @return
 */
// float SerialPort::exchange_data(unsigned char *data)
// {
//     float float_data;
//     float_data = *((float*)data);
//     return float_data;
// };
float turn_on_robot::exchange_data(unsigned char *data,float need_return_float_data)
{
    *((uint8_t *)&need_return_float_data) = data[0];
    *(((uint8_t *)&need_return_float_data+1)) = data[1];
    *(((uint8_t *)&need_return_float_data+2)) = data[2];
    *(((uint8_t *)&need_return_float_data+3)) = data[3];
    return need_return_float_data;
};
/**************************************
Date: January 28, 2021
Function: Loop access to the lower computer data and issue topics
功能: 循环获取下位机数据与发布话题
***************************************/
void turn_on_robot::Control()
{
  while(ros::ok())
  { 
    
    if (true == Get_Sensor_Data_New()) //The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
                                       //通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      _Now = ros::Time::now();
      if(_Last_Time.toSec()==0) _Last_Time=_Now; //Perform this operation when entering for the first time to avoid excessive integration time
                                                 //首次进入时进行此操作，避免积分时间过大
      Sampling_Time = (_Now - _Last_Time).toSec(); //Retrieves time interval, which is used to integrate velocity to obtain displacement (mileage) 
                                                   //获取时间间隔，用于积分速度获得位移(里程)
      
      //Odometer correction parameters
      //里程计误差修正
      Robot_Vel.X=Robot_Vel.X*odom_x_scale;
      Robot_Vel.Y=Robot_Vel.Y*odom_y_scale;
      if(Robot_Vel.Z>=0)
        Robot_Vel.Z=Robot_Vel.Z*odom_z_scale_positive;
      else
        Robot_Vel.Z=Robot_Vel.Z*odom_z_scale_negative;

      //Speed * Time = displacement (odometer)
      //速度*时间=位移（里程计）
      Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
      Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; //Calculate the displacement in the Y direction, unit: m //计算Y方向的位移，单位：m
      Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; //The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad 
      // cout<<"Robot_Pos.Z"<<Robot_Pos.Z*57.3<<endl;
      //Calculate the three-axis attitude from the IMU with the angular velocity around the three-axis and the three-axis acceleration
      //通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);
      Publish_Odom();      //Pub the speedometer topic //发布里程计话题
      Publish_ImuSensor(); //Pub the IMU topic //发布IMU话题
      _Last_Time = _Now; //Record the time and use it to calculate the time interval //记录时间，用于计算时间间隔
    }
    
    ros::spinOnce();   //The loop waits for the callback function //循环等待回调函数
  }
}


#include <tf/tfMessage.h>
#include <cmath>

tf::Transform trans_map_odom;
tf::Transform trans_odom_camera;


void tf_callback(const tf::tfMessage& msg)
{
  if (msg.transforms[0].header.frame_id == "map" && msg.transforms[0].child_frame_id == "odom")
  {
    tf::Quaternion q(msg.transforms[0].transform.rotation.x, 
                     msg.transforms[0].transform.rotation.y,
                     msg.transforms[0].transform.rotation.z,
                     msg.transforms[0].transform.rotation.w);
    tf::Vector3 v(msg.transforms[0].transform.translation.x,
                  msg.transforms[0].transform.translation.y,
                  msg.transforms[0].transform.translation.z);
    trans_map_odom.setRotation(q);
    trans_map_odom.setOrigin(v);

    tf::Vector3 v0(0, 0, 0);
    tf::Vector3 v1(1, 0, 0);
    tf::Vector3 u0 = trans_map_odom * (trans_odom_camera * v0);
    tf::Vector3 u1 = trans_map_odom * (trans_odom_camera * v1);
    angle = getAngle(u1.x() - u0.x(), u1.y() - u0.y());
  }
  else if (msg.transforms[0].header.frame_id == "odom" && msg.transforms[0].child_frame_id == "base_link")
  {
    tf::Quaternion q(msg.transforms[0].transform.rotation.x, 
                     msg.transforms[0].transform.rotation.y,
                     msg.transforms[0].transform.rotation.z,
                     msg.transforms[0].transform.rotation.w);
    tf::Vector3 v(msg.transforms[0].transform.translation.x,
                  msg.transforms[0].transform.translation.y,
                  msg.transforms[0].transform.translation.z);
    trans_odom_camera.setRotation(q);
    trans_odom_camera.setOrigin(v);

    tf::Vector3 v0(0, 0, 0);
    tf::Vector3 v1(1, 0, 0);
    tf::Vector3 u0 = trans_map_odom * (trans_odom_camera * v0);
    tf::Vector3 u1 = trans_map_odom * (trans_odom_camera * v1);
    angle = getAngle(u1.x() - u0.x(), u1.y() - u0.y());
  }
}

/**************************************
Date: January 28, 2021
Function: Constructor, executed only once, for initialization
功能: 构造函数, 只执行一次，用于初始化
***************************************/
turn_on_robot::turn_on_robot():Sampling_Time(0)
{
  //Clear the data
  //清空数据
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data)); 
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));

  ros::NodeHandle private_nh("~"); //Create a node handle //创建节点句柄
  //The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  //private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("usart_port_name",  usart_port_name,  "/dev/ttyUSB0"); //Fixed serial port number //固定串口号
  private_nh.param<int>        ("serial_baud_rate", serial_baud_rate, 115200); //Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  private_nh.param<std::string>("odom_frame_id",    odom_frame_id,    "odom_combined");      //The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  private_nh.param<std::string>("robot_frame_id",   robot_frame_id,   "base_footprint"); //The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  private_nh.param<std::string>("gyro_frame_id",    gyro_frame_id,    "gyro_link"); //IMU topics correspond to TF coordinates //IMU话题对应TF坐标

  //Odometer correction parameters
  //里程计误差修正参数
  private_nh.param<float>("odom_x_scale",    odom_x_scale,    1.0); 
  private_nh.param<float>("odom_y_scale",    odom_y_scale,    1.0); 
  private_nh.param<float>("odom_z_scale_positive",    odom_z_scale_positive,    1.0); 
  private_nh.param<float>("odom_z_scale_negative",    odom_z_scale_negative,    1.0); 

  odom_publisher    = n.advertise<nav_msgs::Odometry>("odom", 50); //Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher     = n.advertise<sensor_msgs::Imu>("robot/imu", 20); //Create an IMU topic publisher //创建IMU话题发布者

  //Set the velocity control command callback function
  //速度控制命令订阅回调函数设置
  Cmd_Vel_Sub     = n.subscribe("cmd_vel",     100, &turn_on_robot::Cmd_Vel_Callback, this); 

  tf_sub = n.subscribe("/tf", 10, tf_callback);
  signal_sub = n.subscribe("/mode_change", 10, mode_Callback);

  ROS_INFO_STREAM("Data ready"); //Prompt message //提示信息
  
  try
  { 
    //Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(usart_port_name); //Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate); //Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    ROS_INFO_STREAM("Stm32_open_success");
    Stm32_Serial.open(); //Open the serial port //开启串口
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("diode_robot can not open Stm32_serial port,Please check the serial port cable!: " << e.what()); //If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if(Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("diode_robot Stm32_serial port opened"); //Serial port opened successfully //串口开启成功提示
  }
}
/**************************************
Date: January 28, 2021
Function: Destructor, executed only once and called by the system when an object ends its life cycle
功能: 析构函数，只执行一次，当对象结束其生命周期时系统会调用这个函数
***************************************/
turn_on_robot::~turn_on_robot()
{
  //Sends the stop motion command to the lower machine before the turn_on_robot object ends
  //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口  
  ROS_INFO_STREAM("Shutting down"); //Prompt message //提示信息
}
