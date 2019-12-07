#ifndef WJ_715_LIDAR_PROTOCOL_H
#define WJ_715_LIDAR_PROTOCOL_H
#include <iostream>
#include "string.h"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <wj_715_lidar/wj_715_lidarConfig.h>
using namespace std ;
namespace wj_lidar
{
  #define MAX_LENGTH_DATA_PROCESS 200000
  #define SCAN_DATA_N0 676
  typedef struct TagDataCache
  {
    char m_acdata[MAX_LENGTH_DATA_PROCESS];
    unsigned int m_u32in;
    unsigned int m_u32out;
  }DataCache;
  class wj_715_lidar_protocol
  {
  public:
    wj_715_lidar_protocol();
    bool dataProcess(const char *data,const int reclen);
    bool protocl(const char *data,const int len);
    bool OnRecvProcess(char *data, int len);
    bool checkXor(char *recvbuf, int recvlen);
    void send_scan(const char *data,const int len);
    ros::NodeHandle nh;
    ros::Publisher marker_pub;                       //saomiao shujufabu
    sensor_msgs::LaserScan scan;

    bool setConfig(wj_715_lidar::wj_715_lidarConfig &new_config,uint32_t level);
    bool heartstate;
  private:
    char        data_[MAX_LENGTH_DATA_PROCESS];
    DataCache   m_sdata;
    wj_715_lidar::wj_715_lidarConfig config_;
    unsigned int g_u32PreFrameNo;
    float scandata[SCAN_DATA_N0];
    float scandata_te[SCAN_DATA_N0];
    float scaninden[SCAN_DATA_N0];

  };

}
#endif // WJ_715_LIDAR_PROTOCOL_H
