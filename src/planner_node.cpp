#include <PlannerWrapper.h>

int main(int argc, char* argv[]) {
    ros::init (argc, argv, "corridor_generator");
    ros::NodeHandle nh( "~" );
    libCorridorGen::Wrapper corridor_wrapper(nh);


    // load map
    corridor_wrapper.load_map("/worlds/map_reduced_tmp3.bt");

    // generate corridor
    corridor_wrapper.update();

    ros::Rate rate(20);
    double start_time, current_time;
    start_time = ros::Time::now().toSec();
    while (ros::ok()) {
        current_time = ros::Time::now().toSec() - start_time;
        corridor_wrapper.publish(current_time);
        rate.sleep();
    }

    return 0;
}