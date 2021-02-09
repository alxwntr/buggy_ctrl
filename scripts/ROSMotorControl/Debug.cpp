#include "Debug.h"

//Two info structs to hold data
buggy_ctrl::pidInfo pidInfo[2];

//Array to hold the structs
buggy_ctrl::ctrlDebug debugInfo;

void publish_debug( ros::Publisher& pub ) {
  debugInfo.data=pidInfo;
  debugInfo.data_length=2;
  pub.publish( &debugInfo );
}
