#include "jackal_broadcaster/jackal_broadcaster_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "jackal_tf2_broadcaster");
    ros::NodeHandle nh;
    jackal_tf2_broadcaster_ns::TransformBroadcaster tf2_broadcaster(nh);
    tf2_broadcaster.spin();
    return 0;
}
