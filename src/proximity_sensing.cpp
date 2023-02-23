#include <string>
#include <iostream>
#include <vector>

#include <ros/ros.h>

#include <proximity_sensing/Point.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher point_head_pub;
std::vector<float> scan_t_1;
std::vector<float> scan_t_2;
ros::Time last_move_time;

constexpr const float next_distance_threshold = 0.02;
constexpr const float distance_threshold = 0.3;
constexpr const float min_distance = 0.3; // Do not look at things farther than
constexpr const float max_distance = 3.0; // Do not look at things farther than
constexpr const float look_altitude = 1.2; // From laser frame
constexpr const float time_not_moving_uninteresting = 2.0; // sec

bool isFlatWindow(const std::vector<float>& ranges, size_t index, size_t window)
{
    size_t half_window = window / 2;
    for(size_t i = 0; i < half_window; i++)
    {
        if(abs(ranges[index - i] - ranges[index - i - 1]) > next_distance_threshold ||
           abs(ranges[index + i] - ranges[index + i + 1]) > next_distance_threshold)
            return false;
    }
    return true;
}

void onNewLidarScan(const sensor_msgs::LaserScan &msg)
{
    if (!scan_t_1.empty())
    {
        if(!scan_t_2.empty())
        {
            size_t index_max = 0;
            float max_motion = 0;
            for (size_t i=2; i < msg.ranges.size() - 2; i++)
            {
                if(std::abs(scan_t_2[i] - scan_t_1[i]) < next_distance_threshold)
                {
                    float motion = std::abs(msg.ranges[i] - scan_t_1[i]);
                    if (isFlatWindow(msg.ranges, i, 5) &&
                        scan_t_1[i] > min_distance &&
                        msg.ranges[i] > min_distance &&
                        msg.ranges[i] < max_distance && 
                        motion > max_motion)
                    {
                        max_motion = motion;
                        index_max = i;
                    }
                }
            }
            if (max_motion >= distance_threshold)
            {
                proximity_sensing::Point pt;
                float angle = msg.angle_min + index_max * msg.angle_increment;
                pt.data.header.frame_id = msg.header.frame_id;
                pt.data.point.x = msg.ranges[index_max] * std::cos(angle);
                pt.data.point.y = msg.ranges[index_max] * std::sin(angle);
                pt.data.point.z = look_altitude;
                pt.priority.value = resource_management_msgs::MessagePriority::LOW;
                point_head_pub.publish(pt);
                last_move_time = ros::Time::now();
            }
            else if (ros::Time::now() - last_move_time >= ros::Duration(time_not_moving_uninteresting))
            {
                proximity_sensing::Point pt;
                pt.priority.value = resource_management_msgs::MessagePriority::VOID;
                point_head_pub.publish(pt);
                last_move_time = ros::Time::now(); // To avoid over publishing
            }
        }
        scan_t_2 = scan_t_1;
    }
    scan_t_1 = msg.ranges;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "proximity_sensing");
    ros::NodeHandlePtr nh(new ros::NodeHandle("~"));

    if(argc != 3)
    {
        std::cout << "[Proximity sensing] node should be launch as:]\n" <<
                     "rosrun proximity_sensing proximity_sensing /laser_topic /resource_manager/buffer_topic" << std::endl;
        return -1;
    }

    std::string laser_topic(argv[1]);
    std::string manager_topic(argv[2]);

    point_head_pub = nh->advertise<proximity_sensing::Point>(manager_topic, 10, true);
    ros::Subscriber lidar_sub = nh->subscribe(laser_topic, 1, onNewLidarScan);

    std::cout << "Proximity sensing is running with:" << std::endl <<
                 "\t-laser topic: " << laser_topic << std::endl <<
                 "\t-manager topic: " << manager_topic << std::endl;

    ros::spin();
    return 0;
}
