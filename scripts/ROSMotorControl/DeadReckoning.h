#ifndef DEADRECKONING_H
#define DEADRECKONING_H

tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_link";
char odom[] = "/odom";

void publish_tf(ros::NodeHandle nh, float x, float y, float theta)
{
  geometry_msgs::TransformStamped t;
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
}

#endif
