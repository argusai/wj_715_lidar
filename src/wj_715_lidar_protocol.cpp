#include "wj_715_lidar_protocol.h"
#include <iostream>

namespace wj_lidar
{
  bool wj_715_lidar_protocol::setConfig(wj_715_lidar::wj_715_lidarConfig &new_config,uint32_t level)
  {
    config_ = new_config;
    scan.header.frame_id = config_.frame_id;
    scan.angle_min = config_.min_ang;
    scan.angle_max = config_.max_ang;
    scan.angle_increment = config_.angle_increment;
    scan.time_increment = config_.time_increment;
    scan.range_min = config_.range_min;
    scan.range_max = config_.range_max;
    scan.intensities.resize(config_.resize);

    cout << "frame_id:" <<scan.header.frame_id<<endl;
    cout << "min_ang:" <<scan.angle_min<<endl;
    cout << "max_ang:" <<scan.angle_max<<endl;
    cout << "angle_increment:" <<scan.angle_increment<<endl;
    cout << "time_increment:" <<scan.time_increment<<endl;
    cout << "range_min:" <<scan.range_min<<endl;
    cout << "range_max:" <<scan.range_max<<endl;
    int resizeNum;
    cout << "resizeNum:" <<resizeNum<<endl;
    return true;
  }
   wj_715_lidar_protocol::wj_715_lidar_protocol()
   {
     memset(&m_sdata,0,sizeof(m_sdata));
     marker_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
     ros::Time scan_time = ros::Time::now();      //make a virtual data per sec
     scan.header.stamp = scan_time;

     g_u32PreFrameNo = 0;

     scan.header.frame_id = "wj_715_lidar_frame";
     scan.angle_min = -2.35619449;
     scan.angle_max = 2.35619449;
     scan.angle_increment = 0.00698;
     scan.time_increment = 0.0000222222;
     scan.range_min = 0.05;
     scan.range_max = 60;
     scan.ranges.resize(676);
     scan.intensities.resize(676);
     start_scan_time = ros::Time::now();

     cout << "wj_715_lidar_protocl start success" << endl;

   }
   bool wj_715_lidar_protocol::dataProcess(const char *data, const int reclen)
   {
     if(reclen > MAX_LENGTH_DATA_PROCESS)
     {
       return false;
     }

     if(m_sdata.m_u32in + reclen > MAX_LENGTH_DATA_PROCESS)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
       return false;
     }
     memcpy(&m_sdata.m_acdata[m_sdata.m_u32in],data,reclen*sizeof(char));
     m_sdata.m_u32in += reclen;
     while(m_sdata.m_u32out < m_sdata.m_u32in)
     {
       if(m_sdata.m_acdata[m_sdata.m_u32out] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+1] == 0x02 &&
          m_sdata.m_acdata[m_sdata.m_u32out+2] == 0x02 && m_sdata.m_acdata[m_sdata.m_u32out+3] == 0x02)
       {
         unsigned l_u32reallen = (unsigned char)(m_sdata.m_acdata[m_sdata.m_u32out + 4] << 24) |
                                 (unsigned char)(m_sdata.m_acdata[m_sdata.m_u32out + 5] << 16) |
                                 (unsigned char)(m_sdata.m_acdata[m_sdata.m_u32out + 6] << 8)  |
                                 (unsigned char)(m_sdata.m_acdata[m_sdata.m_u32out + 7] << 0);
         l_u32reallen = l_u32reallen + 9;

         if(l_u32reallen <= (m_sdata.m_u32in - m_sdata.m_u32out + 1))
         {
           if(OnRecvProcess(&m_sdata.m_acdata[m_sdata.m_u32out],l_u32reallen))
           {
             m_sdata.m_u32out += l_u32reallen;
           }
           else
           {
             cout << "continuous frame"<<endl;
             int i;
             for(i == 1; i<l_u32reallen; i++)
             {
               if((m_sdata.m_acdata[m_sdata.m_u32out + i] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 1] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 2] == 0x02) &&
                  (m_sdata.m_acdata[m_sdata.m_u32out + i + 3] == 0x02))
               {
                 m_sdata.m_u32out += i;
                 break;
               }
               if(i == l_u32reallen)
               {
                 m_sdata.m_u32out += l_u32reallen;
               }
             }
           }
         }
         else if(l_u32reallen >= MAX_LENGTH_DATA_PROCESS)
         {
           cout << "l_u32reallen >= MAX_LENGTH_DATA_PROCESS"<<endl;
           cout << "reallen: "<<l_u32reallen<<endl;
           memset(&m_sdata,0,sizeof(m_sdata));

         }
         else
         {
           break;
         }
       }
       else
       {
         m_sdata.m_u32out++;
       }
     } //end while(m_sdata.m_u32out < m_sdata.m_u32in)

     if(m_sdata.m_u32out >= m_sdata.m_u32in)
     {
       memset(&m_sdata,0,sizeof(m_sdata));
     }
      return true;
   }

   bool wj_715_lidar_protocol::OnRecvProcess( char *data, int len)
   {
     if(len > 0)
     {
       //if(checkXor(data,len))
       {
         protocl(data,len);
       }
     }
     else
     {
       return false;
     }
     return true;
   }

   bool wj_715_lidar_protocol::protocl(const char *data, const int len)
   {
     if((data[8] == 0x73 && data[9] == 0x52)||(data[8] == 0x73 && data[9] == 0x53) )   //command type:0x73 0x52/0X53
     {
       static int s_n32ScanIndex;
       int l_n32PackageNo=  data[82];                                        //shuju bao xu hao
       unsigned int l_u32FrameNo = (unsigned char)(data[77]<<24) + (unsigned char)(data[78]<<16) + (unsigned char)(data[79]<<8) + (unsigned char)data[80];        //quan hao

       if(l_n32PackageNo == 1)
       {
         s_n32ScanIndex = 0;
         g_u32PreFrameNo = l_u32FrameNo;
         float scaleFactor = (unsigned char)data[1092];
         for(int j = 0; j < 1000;j=j+2)
         {
           scandata[s_n32ScanIndex] = (((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
           scandata[s_n32ScanIndex] /= 1000.0;
           if(scandata[s_n32ScanIndex]>60 || scandata[s_n32ScanIndex]==0)
           {
             scandata[s_n32ScanIndex]= NAN;
           }
           scaninden[s_n32ScanIndex] = (unsigned char)data[114+1000+j / 2] * scaleFactor * 2;
           s_n32ScanIndex++;
         }
       }
       else if(l_n32PackageNo == 2)
       {
         if(g_u32PreFrameNo == l_u32FrameNo)
         {
           float scaleFactor = (unsigned char)data[444];
           for(int j = 0; j < 352;j=j+2)
           {
             scandata[s_n32ScanIndex] =(((unsigned char)data[85+j]) << 8) + ((unsigned char)data[86+j]);
             scandata[s_n32ScanIndex] /= 1000.0;
             if(scandata[s_n32ScanIndex]>60 || scandata[s_n32ScanIndex]==0)
             {
               scandata[s_n32ScanIndex]= NAN;
             }
             scaninden[s_n32ScanIndex] = (unsigned char)data[114+352+j / 2] * scaleFactor * 2;
             s_n32ScanIndex++;
           }
          // adjust angle_min to min_ang config param
           int index_min = (config_.min_ang+2.35619449)/scan.angle_increment;
           // adjust angle_max to max_ang config param
           int index_max =  676-((2.35619449-config_.max_ang)/ scan.angle_increment);
           scan.ranges.resize(index_max-index_min);
           scan.intensities.resize(index_max-index_min);

           for (int j = index_min; j <= index_max; ++j)
           {
               scan.ranges[j - index_min] = scandata[j];
               scan.intensities[j - index_min]=scaninden[j];
           }

           ros::Time end_scan_time = ros::Time::now();
           scan.header.stamp = end_scan_time;
           scan.scan_time = (end_scan_time - start_scan_time).toSec();
           marker_pub.publish(scan);
           start_scan_time = end_scan_time;
         }

          else
         {
           s_n32ScanIndex = 0;
           //s_n32State = 0;
           return false;
         }
       }
       return true;
     }
     else
     {
       return false;
     }

   }

   bool wj_715_lidar_protocol::checkXor( char *recvbuf,  int recvlen)
   {
     int i = 0;
     char check = 0;
     char *p = recvbuf;
     int len;
     if(*p == (char)0x02)
     {
       p = p+8;
       len = recvlen - 9;
       for(i = 0; i < len; i++)
       {
         check ^= *p++;
       }
       if(check == *p)
       {
         return true;
       }
       else
         return false;
     }
     else
     {
       return false;
     }
   }
}
