
#include <robot_self_filter/self_filter.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "self_filter");

  robot_self_filter::SelfFilter s;
  ros::spin ();
    
  return (0);
}
