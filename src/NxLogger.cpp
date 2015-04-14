// -*- C++ -*-
/*!
 * @file  NxLogger.cpp
 * @brief record images and robot states with time stamp
 * @date $Date$
 *
 * $Id$
 */

#include "NxLogger.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Image.h"

// Module specification
// <rtc-template block="module_spec">
static const char* nxlogger_spec[] =
  {
    "implementation_id", "NxLogger",
    "type_name",         "NxLogger",
    "description",       "record images and robot states with time stamp",
    "version",           "1.0.0",
    "vendor",            "Ryo Hanai",
    "category",          "Tools",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
NxLogger::NxLogger(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_lhImageIn("LeftHandImage", m_lhImage),
    m_rhImageIn("RightHandImage", m_rhImage),
    m_robotStateIn("RobotState", m_robotState)

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
NxLogger::~NxLogger()
{
}



RTC::ReturnCode_t NxLogger::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("LeftHandImage", m_lhImageIn);
  addInPort("RightHandImage", m_rhImageIn);
  addInPort("RobotState", m_robotStateIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // </rtc-template>
  
  return RTC::RTC_OK;
}

RTC::ReturnCode_t NxLogger::onFinalize()
{
  std::cerr << "onFinalize called" << std::endl;
  m_logBag->close();
  return RTC::RTC_OK;
}


RTC::ReturnCode_t NxLogger::onStartup(RTC::UniqueId ec_id)
{
  std::cerr << "onStartup called" << std::endl;
  rosbag::Bag bag;
  bag.open("test.bag", rosbag::bagmode::Write);
  //std_msgs::Int32 i;
  // for (int j = 0; j < 100; j++) {
  //   i.data = j;
  //i.data = 34;
  // bag.write("numbers", ros::Time::now(), i);
  //   std::cerr << j << std::endl;
        
  //   sleep(2.0);
  // }
      
  // bag.close();


  m_logBag = new rosbag::Bag("test.bag", rosbag::bagmode::Write);
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NxLogger::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t NxLogger::onExecute(RTC::UniqueId ec_id)
{
  static int jsSeq = 0;
  static int lhImgSeq = 0;  
  static int rhImgSeq = 0;  

  if (m_robotStateIn.isNew()) {
    m_robotStateIn.read();
    sensor_msgs::JointState js;
    js.header.seq = jsSeq++;
    js.header.stamp.sec = m_robotState.tm.sec;
    js.header.stamp.nsec = m_robotState.tm.nsec;
    // js.name.clear();
    // js.position.clear();
    js.name.push_back("CHEST_JOINT0");
    js.name.push_back("HEAD_JOINT0");
    js.name.push_back("HEAD_JOINT1");
    js.name.push_back("RARM_JOINT0");
    js.name.push_back("RARM_JOINT1");
    js.name.push_back("RARM_JOINT2");
    js.name.push_back("RARM_JOINT3");
    js.name.push_back("RARM_JOINT4");
    js.name.push_back("RARM_JOINT5");
    js.name.push_back("RHAND_JOINT0"); // LARM_JOINT0, 9
    js.name.push_back("RHAND_JOINT1"); // LARM_JOINT1
    js.name.push_back("RHAND_JOINT2"); // LARM_JOINT2
    js.name.push_back("RHAND_JOINT3"); // LARM_JOINT3
    js.name.push_back("LARM_JOINT0"); // LARM_JOINT4, 13
    js.name.push_back("LARM_JOINT1"); // LARM_JOINT5
    js.name.push_back("LARM_JOINT2"); // RHAND_JOINT0, 15
    js.name.push_back("LARM_JOINT3"); // RHAND_JOINT1
    js.name.push_back("LARM_JOINT4"); // RHAND_JOINT2
    js.name.push_back("LARM_JOINT5"); // RHAND_JOINT3
    js.name.push_back("LHAND_JOINT0"); // 19
    js.name.push_back("LHAND_JOINT1");
    js.name.push_back("LHAND_JOINT2");
    js.name.push_back("LHAND_JOINT3");
    for (unsigned int i = 0; i < m_robotState.qState.length(); i++) {
      for (unsigned int j = 0; j < m_robotState.qState[i].length(); j++) {
        js.position.push_back(m_robotState.qState[i][j]);
        js.velocity.push_back(m_robotState.dqState[i][j]);
        js.effort.push_back(0.0);
      }
    }
    
    try {      
      std::cerr << "/joint_states" << std::endl;
      timespec start;
      clock_gettime(CLOCK_REALTIME, &start);
      js.header.stamp.sec = start.tv_sec;
      js.header.stamp.nsec = start.tv_nsec;

      m_logBag->write("/joint_states", js.header.stamp, js);
      //m_logBag->write("/joint_states", ros::Time::now(), js);
    } catch (rosbag::BagIOException e) {
      // ROS_ERROR("Problem when writing to bag file");
      std::cerr << "error when writing to bag file" << std::endl;
    }
  }

  if (m_lhImageIn.isNew()) {
    m_lhImageIn.read();
    sensor_msgs::Image lhImg;
    lhImg.header.seq = lhImgSeq++;
    lhImg.header.stamp.sec = m_lhImage.tm.sec;
    lhImg.header.stamp.nsec = m_lhImage.tm.nsec;
    // lhImg.header.frame_id =;
    // m_lhImage.data.captured_time;
    lhImg.width = m_lhImage.data.image.width;
    lhImg.height = m_lhImage.data.image.height;
    // ColorFormat m_lhImage.data.image.format;
    lhImg.encoding = "rgb8";
    lhImg.is_bigendian = 0;
    const unsigned int nChannels = 3;
    lhImg.step = lhImg.width * nChannels;
    lhImg.data.resize(lhImg.width * lhImg.height * nChannels);
    for (unsigned int k = 0; k < lhImg.height; k++) {
      for (unsigned int l = 0; l < lhImg.width; l++) {
        for (unsigned int m = 0; m < nChannels; m++) {
          lhImg.data[k * lhImg.step + l * nChannels + m]
            = m_lhImage.data.image.raw_data[k * lhImg.step + l * nChannels + m];
        }
      }
    }  

    try {      
      std::cerr << "/lhand/usb_camera/image_raw" << std::endl;
      m_logBag->write("/lhand/usb_camera/image_raw", lhImg.header.stamp, lhImg);
    } catch (rosbag::BagIOException e) {
      std::cerr << "error when writing to bag file" << std::endl;
    }
  }
  
  if (m_rhImageIn.isNew()) {
    m_rhImageIn.read();
    sensor_msgs::Image rhImg;
    rhImg.header.seq = rhImgSeq++;
    rhImg.header.stamp.sec = m_rhImage.tm.sec;
    rhImg.header.stamp.nsec = m_rhImage.tm.nsec;
    // rhImg.header.frame_id =;
    // m_rhImage.data.captured_time;
    rhImg.width = m_rhImage.data.image.width;
    rhImg.height = m_rhImage.data.image.height;
    // ColorFormat m_rhImage.data.image.format;
    rhImg.encoding = "rgb8";
    rhImg.is_bigendian = 0;
    const unsigned int nChannels = 3;
    rhImg.step = rhImg.width * nChannels;
    rhImg.data.resize(rhImg.width * rhImg.height * nChannels);
    for (unsigned int k = 0; k < rhImg.height; k++) {
      for (unsigned int l = 0; l < rhImg.width; l++) {
        for (unsigned int m = 0; m < nChannels; m++) {
          rhImg.data[k * rhImg.step + l * nChannels + m]
            = m_rhImage.data.image.raw_data[k * rhImg.step + l * nChannels + m];
        }
      }
    }

    try {      
      std::cerr << "/rhand/usb_camera/image_raw" << std::endl;
      m_logBag->write("/rhand/usb_camera/image_raw", rhImg.header.stamp, rhImg);
    } catch (rosbag::BagIOException e) {
      std::cerr << "error when writing to bag file" << std::endl;
    }

  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t NxLogger::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t NxLogger::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void NxLoggerInit(RTC::Manager* manager)
  {
    coil::Properties profile(nxlogger_spec);
    manager->registerFactory(profile,
                             RTC::Create<NxLogger>,
                             RTC::Delete<NxLogger>);
  }
  
};


