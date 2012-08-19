
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <robot_self_filter/self_filter.h>

namespace robot_self_filter
{
  class SelfFilterNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit() 
    {
      NODELET_DEBUG("Initializing nodelet...");

      filter_.reset(new SelfFilter(this->getPrivateNodeHandle()));
    }
  private:
    boost::shared_ptr<SelfFilter> filter_;
  };
}


PLUGINLIB_DECLARE_CLASS(robot_self_filter, SelfFilterNodelet, robot_self_filter::SelfFilterNodelet, nodelet::Nodelet);

