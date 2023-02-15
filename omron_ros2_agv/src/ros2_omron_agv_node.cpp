#include "ros2_omron_agv/ros2_omron_agv.hpp"

using namespace std::chrono_literals;


#include <cmath>




statusPub::statusPub(std::string node_name, ArClientBase *client, std::string name, std::string topic) : 
      Node(node_name),
      myClient(client), 
      myPoseCB(this, &statusPub::pose_cb),
      dockedStatusCB(this, &statusPub::dock_stats_cb)
{

  RCLCPP_INFO(this->get_logger(),"Setup Callback for %s publishing on %s",name.c_str(), topic.c_str());

  service = this->create_service<omron_ros2_msgs::srv::DockRequest>("dock", std::bind(&statusPub::requestDock,this,std::placeholders::_1, std::placeholders::_2));

  status_pub = this->create_publisher<omron_ros2_msgs::msg::Omron>("robot_status", 1);
  odom_pub   = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  cmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&statusPub::cmdVelCB, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(200ms, std::bind(&statusPub::cmdVelWD, this));

  tf_broadcaster_        = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);



  sub_initpose = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10, std::bind(&statusPub::LocaliseCallback, this, std::placeholders::_1));

  velCount = 0;
  prevVelCount = 0;
  vel_valid = false;


  myClient->addHandler("updateNumbers", &myPoseCB);
  myClient->request("updateNumbers", 50); 

  myClient->addHandler("dockInfoChanged", &dockedStatusCB);
  myClient->requestOnce("dockInfoChanged");
  myClient->request("dockInfoChanged", -1);

  geometry_msgs::msg::TransformStamped t;

  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "map";
  t.child_frame_id  = "/omron/odom";

  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0;
  t.transform.rotation.z = 0.0;
  t.transform.rotation.w = 1.0;

  tf_static_broadcaster_->sendTransform(t);

}

bool statusPub::requestDock(const std::shared_ptr<omron_ros2_msgs::srv::DockRequest::Request> req, std::shared_ptr<omron_ros2_msgs::srv::DockRequest::Response>  res){
  res->result = true;
  myClient->requestOnce("dock");
  return true;
} 

void statusPub::pose_cb(ArNetPacket *packet)
{
  double batVolt, x , y, theta, x_vel, y_vel, theta_vel, temp;

  batVolt = ( (double) packet->bufToByte2() )/10.0;
  x = (double) packet->bufToByte4();
  y = (double) packet->bufToByte4();
  theta = (double) packet->bufToByte2();
  x_vel = (double) packet->bufToByte2();
  theta_vel = (double) packet->bufToByte2();
  y_vel = (double) packet->bufToByte2();
  temp = (double) packet->bufToByte();

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "/omron/odom";
  transform.child_frame_id = "/omron/base_link";

  transform.transform.translation.x = x/1000.0;
  transform.transform.translation.y = y/1000.0; 
  transform.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, angles::from_degrees(theta/1.0)); 

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_->sendTransform(transform);

  currentPose.header.frame_id = "/omron/base_link";
  currentPose.header.stamp = this->get_clock()->now();
  currentPose.pose.orientation.x = q.getX();
  currentPose.pose.orientation.y = q.getY();
  currentPose.pose.orientation.z = q.getZ();
  currentPose.pose.orientation.w = q.getW();
  currentPose.pose.position.x = x/1000.0;
  currentPose.pose.position.y = y/1000.0;
  currentPose.pose.position.z = 0;

  omron_ros2_msgs::msg::Omron data;
  data.battery_percentage = batVolt;
  data.dock_status = dock_status;
  data.robot_status = robotStatus;
  status_pub->publish(data);

  nav_msgs::msg::Odometry message_odom;

  message_odom.header.stamp =  this->get_clock()->now();
  message_odom.child_frame_id  = "/omron/base_link";
  message_odom.header.frame_id = "/omron/odom";

  message_odom.pose.pose.orientation.x = q.getX();
  message_odom.pose.pose.orientation.y = q.getY();
  message_odom.pose.pose.orientation.z = q.getZ();
  message_odom.pose.pose.orientation.w = q.getW();

  message_odom.pose.pose.position.x = x/1000.0;
  message_odom.pose.pose.position.y = y/1000.0;

  message_odom.twist.twist.linear.x  = x_vel/1000.0;
  message_odom.twist.twist.linear.y  = y_vel/1000.0;
  message_odom.twist.twist.angular.z = theta_vel *(0.0174);

  odom_pub->publish(message_odom);

  //TODO odometry
}

void statusPub::dock_stats_cb(ArNetPacket *packet)
{
  int state = packet->bufToUByte();
  int forcedDock = packet->bufToUByte();
  int secondsToShutdown = packet->bufToUByte2();

  std::string stateStr;
  std::string forcedStr;

  if (state == 0)
    stateStr = "  Undocked";
  else if (state == 1)
    stateStr = "   Docking";
  else if (state == 2)
    stateStr = "   Docked";
  else if (state == 3)
    stateStr = "Undocking";
  else
    stateStr = "  Unknown";
  
  if (forcedDock == 0)
    forcedStr = "false";
  else if (forcedDock == 1)
    forcedStr = " true";
  else
    forcedStr = "unknown";

  //Store it
  this->dock_status = state;

  if (secondsToShutdown == 0)
    RCLCPP_INFO(this->get_logger(),"State: %s Forced: %s Shutdown: never", 
	       stateStr.c_str(), forcedStr.c_str());
  else
    RCLCPP_INFO(this->get_logger(), "State: %s Forced: %s Shutdown: %d", 
	       stateStr.c_str(), forcedStr.c_str(), secondsToShutdown);
  
}

void statusPub::simplePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (optional 2-byte int)</td>

  int x = msg->pose.position.x*1000;
  int y = msg->pose.position.y*1000;

  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.orientation,orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int theta = angles::to_degrees(yaw);

  RCLCPP_WARN(this->get_logger(),"I got a pose at (%d, %d) Theta: %d", x, y, theta);

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte4ToBuf(theta); //Theta
  myClient->requestOnce("gotoPose", &p);

}

void statusPub::LocaliseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  //<td>gotoPose</td> <td> X (4-byte integer), Y (4-byte int), Theta (4-byte int)</td>

  int x = msg->pose.pose.position.x*1000;
  int y = msg->pose.pose.position.y*1000;

  tf2::Quaternion orientation;
  tf2::fromMsg(msg->pose.pose.orientation,orientation);

  double roll, pitch, yaw;
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  int theta = angles::to_degrees(yaw);

  RCLCPP_WARN(this->get_logger(),"Set pose to (%d, %d) Theta: %d", x, y, theta);

  ArNetPacket p;
  p.byte4ToBuf(x); //X
  p.byte4ToBuf(y); //Y
  p.byte4ToBuf(theta); //ThetaI got a pose at
  myClient->requestOnce("localizeToPose", &p);

}

void statusPub::cmdVelCB(const geometry_msgs::msg::Twist::SharedPtr msg){
  velCount++;
  if (fabs(msg->linear.x) > 0.001 || fabs(msg->angular.z) > 0.001){
    ArNetPacket packet;
    vel_valid = true;
    packet.doubleToBuf(100 * msg->linear.x);
    packet.doubleToBuf((msg->angular.z * 2)/0.0174);
    packet.doubleToBuf(100);
    packet.doubleToBuf(0.0);
    myClient->requestOnce("ratioDrive", &packet);
  }
  else {
    vel_valid = false;
  }

}

void statusPub::cmdVelWD(){
    if ((uint8_t)(velCount - prevVelCount) > 0){
      //Valid data
      prevVelCount = velCount;
    }
    else if (vel_valid == true){
      vel_valid = false;
      myClient->requestOnce("stop");
      RCLCPP_WARN(this->get_logger(),"Timeout on cmd_vel. velCount: %d, prevVelCount: %d", velCount, prevVelCount);
    }
}

//LASER

laserPub::laserPub(std::string node_name, ArClientBase *client, std::string name, std::string topic): 
Node(node_name),
myClient(client), 
myLaserCB(this, &laserPub::laser_cb)
{
  laser_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic, rclcpp::SensorDataQoS());
  myClient->addHandler(name.c_str(), &myLaserCB);
  myClient->request(name.c_str(), 100);

  RCLCPP_WARN(this->get_logger(),"Setup Callback for %s publishing on %s",name.c_str(), topic.c_str());
}

void laserPub::laser_cb(ArNetPacket *packet)
{
  sensor_msgs::msg::PointCloud2 myScan;
  
  myScan.header.stamp = this->get_clock()->now();
  // myScan.header.seq = seq++;
  myScan.header.frame_id = "map";

  int x, y;
  int numReadings;
  int i;

  numReadings = packet->bufToByte4();

  myScan.height = 1;
  myScan.width  = numReadings;
  myScan.fields.resize (3);
  myScan.fields[0].name = "x";
  myScan.fields[0].offset = 0;
  myScan.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  myScan.fields[0].count = 1;
  myScan.fields[1].name = "y";
  myScan.fields[1].offset = 4;
  myScan.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  myScan.fields[1].count = 1;
  myScan.fields[2].name = "z";
  myScan.fields[2].offset = 8;
  myScan.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  myScan.fields[2].count = 1;
  myScan.point_step = 12;
  myScan.row_step   = myScan.point_step * myScan.width;
  myScan.data.resize (myScan.row_step   * myScan.height);
  myScan.is_dense = false;

  if (numReadings == 0)
  {
    RCLCPP_WARN(this->get_logger(),"No readings for sensor %s\n\n", myClient->getName(packet));
    return;
  }

  for (i = 0; i < numReadings; i++)
  {
    x = packet->bufToByte4();
    y = packet->bufToByte4();

    float *pstep = (float*)&myScan.data[i * myScan.point_step];

    pstep[0] = x/1000.0;
    pstep[1] = y/1000.0;
    pstep[2] = 0;
  }

  laser_pub->publish(myScan);
}

//MAP

mapPub::mapPub(std::string node_name, ArClientBase *client) : 
      Node(node_name),
      myClient(client), 
      getMapNameCB(this, &mapPub::handleGetMapName),
      getMapCB(this, &mapPub::handleGetMap)
{

  

  metadata_pub = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata",rclcpp::QoS(1).reliable().transient_local());
  map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map",rclcpp::QoS(1).reliable().transient_local());


  client->addHandler("getMap", &getMapCB);
  client->addHandler("getMapName", &getMapNameCB);
  client->requestOnce("getMapName");
  start.setToNow();
  client->requestOnce("getMap");

  while(!mapped){
    sleep(1);
  }

  metadata_pub->publish(meta_data_message_);
  map_pub->publish(map_resp_);

}

void mapPub::handleGetMapName(ArNetPacket *packet)
{
  char buffer[512];

  packet->bufToStr(buffer, sizeof(buffer));
  printf("MapFile: %s\n", buffer);
}

void mapPub::handleGetMap(ArNetPacket *packet)
{
  char buffer[10000];

  if (packet->getDataReadLength() == packet->getDataLength())
  {
    printf("Empty packet signifying end of map (for central forward)\n");
    return;
  }
  
  packet->bufToStr(buffer, sizeof(buffer));
  // if we got an end of line char instead of a line it means the map is over
  if (buffer[0] == '\0')
  {
    printf("First line \n %.*s", 20 , buffer);
    
    printf("Map took %g seconds\n", start.mSecSince() / 1000.0);
    arMap.parsingComplete();

    int n_points = arMap.getNumPoints();
    printf("Map has %d points\n", n_points);

    std::vector<ArPose> *point_list = new std::vector<ArPose> (n_points);
    point_list = arMap.getPoints();

    int res = arMap.getResolution();
    printf("Map has resolution of %dmm\n", res);

    ArPose minPose, maxPose;
    minPose = arMap.getMinPose();
    maxPose = arMap.getMaxPose();

    printf("Map has Min Coords of (%f %f) mm\n", minPose.getX(), minPose.getY() );
    printf("Map has Min Coords of (%f %f) mm\n", maxPose.getX(), maxPose.getY() );

    int gridX = (maxPose.getX() - minPose.getX())/res;
    int gridY = (maxPose.getY() - minPose.getY())/res;

    printf("Map has grid of (%d %d) mm\n", gridX, gridY);
    
    
    //client.disconnect();
    //exit(0);

    map_resp_.info.width = gridX;
    map_resp_.info.height = gridY;
    map_resp_.info.resolution = res/1000.0;

    map_resp_.info.origin.position.x = minPose.getX()/1000.0; //Y?
    map_resp_.info.origin.position.y = minPose.getY()/1000.0; //X?

    map_resp_.info.map_load_time = this->get_clock()->now();
    map_resp_.header.frame_id = "map";
    map_resp_.header.stamp = this->get_clock()->now();
    RCLCPP_WARN(this->get_logger(),"Read a %d X %d map @ %.3lf m/cell",
            map_resp_.info.width,
            map_resp_.info.height,
            map_resp_.info.resolution);
    meta_data_message_ = map_resp_.info;

    //Lets fill some datas
    map_resp_.data.resize(gridX * gridY);

    //Iterate through points and fill
    for(int i=0; i < point_list->size(); i++){
      int coord = gridX *  ((point_list->at(i).getY()-minPose.getY())/res) + (point_list->at(i).getX()-minPose.getX()) /res; //Y is flipped apparent
      //printf("Point at Coord %d of %d\n", coord, gridX*gridY);
      map_resp_.data[coord] = 100;
    }

    mapped = 1;

  }


  else
  {

    //The header has changed but it still works with the old format for what we want. 
    char *header_location = strstr(buffer, "2D-Map-Ex4");
    if (header_location != NULL) /* Old header found */
    {
      buffer[6] = '\0'; //Cut the -EX4 off 
    }


    //printf("line '%s'\n", buffer);
    arMap.parseLine(buffer);
  }

}


int main(int argc, char **argv)
{
  
  rclcpp::init(argc, argv);

  Aria::init();
  
  ArClientBase client;

  client.enforceProtocolVersion("5MTX");

  ArArgumentBuilder args;

  args.addPlain("-host");
  // std::string sparam;
  // if (np.getParam("host", sparam))
  // {
  //   args.addPlain(sparam.c_str());
  // }
  // else
  // {
    args.addPlain("192.168.1.32");  //Default IP
  // }
  
  //PORT
  args.addPlain("-p");
  // if (np.getParam("port", sparam))
  // {
  //   args.addPlain(sparam.c_str());
  // }
  // else
  // {
    args.addPlain("7272");  //Default PORT
  // }

  //USER
  args.addPlain("-u");
  // if (np.getParam("user", sparam))
  // {
  //   args.addPlain(sparam.c_str());
  // }
  // else
  // {
    args.addPlain("admin");  //Default user
  // }

  //PASSWD
  args.addPlain("-pwd admin");

  ArClientSimpleConnector clientConnector(&args);

  //Reard in args
  clientConnector.parseArgs();

  //Connect
  if (!clientConnector.connectClient(&client))
  {
    if (client.wasRejected())
      std::cout << "Server" << client.getHost() << "rejected connection, exiting\n" << "\n";
    else
      std::cout << "Could not connect to server" << client.getHost() <<  "exiting\n" << "\n";;
    exit(1);
  } 

  client.runAsync();
  // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::cout << "Connected to server.\n";

  rclcpp::executors::MultiThreadedExecutor executor;

  auto nodeState    = std::make_shared<statusPub>("omron_agv_statusPub",&client);
  auto nodeLaser    = std::make_shared<laserPub> ("omron_agv_laserPub",&client, "Laser_1Current", "cloud_in");
  // auto nodeLaserLow = std::make_shared<laserPub> ("omron_agv_laserPub_low",&client, "Laser_2Current", "cloud_in_low");
  auto nodeMap      = std::make_shared<mapPub>   ("omron_agv_mapPub",&client);

  executor.add_node(nodeState);
  executor.add_node(nodeLaser);
  // executor.add_node(nodeLaserLow);
  executor.add_node(nodeMap);


  std::cout << "Spin\n";
  // rclcpp::spin(std::make_shared<statusPub>(&client));
  executor.spin();
  std::cout << "End\n";
  rclcpp::shutdown();
  client.disconnect();
  Aria::exit(0);
}