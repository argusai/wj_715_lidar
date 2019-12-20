#include <ros/ros.h>
#include "async_client.h"
#include "wj_715_lidar_protocol.h"
using namespace wj_lidar;

int state = 0;
int InitTcpConnection(const char *addr, int port, Async_Client **client_,
                      fundata_t fundata_) {
  io_service iosev;

  ip::tcp::endpoint ep(ip::address_v4::from_string(addr), port);
  *client_ = new Async_Client(iosev, ep, fundata_);
  iosev.run();

  return 1;
}

int boost_tcp_init_connection(const char *addr, int port,
                              Async_Client **client_, fundata_t fundata_) {
  int timecnt = 0;
  *client_ = NULL;

  boost::thread tmp(&InitTcpConnection, addr, port, client_, fundata_);
  tmp.detach();

  while (timecnt < 50) {
    timecnt++;
    usleep(20000); // 20 ms
    if ((*client_)->client_return_status()) {
      return 0;
    }
  }
  *client_ = NULL;
  return -1;
}

int boost_tcp_sync_send(Async_Client *client_, const char *msg, const int len) {
  if (client_ == NULL || client_->client_return_status() == 0) {
    printf("not connected , please connect first \n");
    ros::shutdown();
    return -1;
  } else {
    client_->client_async_write((char *)msg, len);
    return 0;
  }

  return 1;
}

int boost_tcp_sync_read(Async_Client *client_) {
  if (client_ == NULL || client_->client_return_status() == 0) {
    printf("not connected , please connect first \n");
    return -1;
  } else {
    client_->client_async_read();
    return 0;
  }
  return 1;
}

/* ------------------------------------------------------------------------------------------
 *  show demo --
 * ------------------------------------------------------------------------------------------
 */
wj_715_lidar_protocol *protocol;
Async_Client *client;
void CallBackRead(const char *addr, int port, const char *data, const int len) {
  protocol->dataProcess(data, len);
}

void callback(wj_715_lidar::wj_715_lidarConfig &config, uint32_t level) {
  protocol->setConfig(config, level);
}

void timerCallback(const ros::TimerEvent &) {
  char continuequary[26] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00,
                            0x11, 0x73, 0x45, 0x4E, 0x20, 0x57, 0x4C,
                            0x52, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61,
                            0x74, 0x61, 0x20, 0x01, 0x3F};

  boost_tcp_sync_send(client, continuequary, 26);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wj_715_lidar_01");
  ros::NodeHandle nh("~");
  ros::Timer timer;
  std::string hostname;
  nh.getParam("hostname", hostname);
  cout << hostname << endl;
  std::string port;
  nh.getParam("port", port);

  protocol = new wj_715_lidar_protocol();
  dynamic_reconfigure::Server<wj_715_lidar::wj_715_lidarConfig> server;
  dynamic_reconfigure::Server<wj_715_lidar::wj_715_lidarConfig>::CallbackType f;
  f = boost::bind(&callback, _1, _2);
  server.setCallback(f);
  client = NULL;

  ros::Duration(2).sleep();
  boost_tcp_init_connection(hostname.c_str(),atoi(port.c_str()),&client,&CallBackRead);
  boost_tcp_sync_read(client);
  timer= nh.createTimer(ros::Duration(1), timerCallback);
  ROS_INFO("Hello wj_715_lidar!");
  ros::spin();

  // while (ros::ok()) {
  //   if (state == 0) {
  //     cout << state << endl;
  //     state = boost_tcp_init_connection(hostname.c_str(), atoi(port.c_str()),
  //                                       &client, &CallBackRead);
  //     boost_tcp_sync_read(client);
  //     cout << state << endl;
  //     if (state != -1) {
  //       timer = nh.createTimer(ros::Duration(1), timerCallback);
  //       state = 1;
  //       ROS_INFO("Hello wj_715_lidar!");
  //     }
  //     ros::Duration(1).sleep();
  //   } else if (state == -1) {
  //     cout << state << endl;
  //     ros::spinOnce();
  //     ros::Duration(2).sleep();
  //     state = 0;
  //   } else {
  //     cout << state << endl;
  //     ros::spinOnce();
  //     ros::Duration(1).sleep();
  //   }
  // }
}
