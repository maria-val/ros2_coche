#include "rclcpp/rclcpp.hpp"
#include "c4_radar/radar_ars408.hpp"
#include "c4_radar/radar_srr208.hpp"
#include "visualization_msgs/msg/marker.hpp"

class RadarVisualization : public rclcpp::Node
{
  public:
    RadarVisualization() : Node("radar_visualization")
    {
      parsed_ars408_subscriber_ =  this->create_subscription<radar_msgs::msg::RadarMsg408>("radar_parser/ars408", 1000, std::bind(&RadarVisualization::draw_radar_parsed_msg_408, this, std::placeholders::_1));
      parsed_srr208_subscriber_ =  this->create_subscription<radar_msgs::msg::RadarMsg208>("radar_parser/srr208", 1000, std::bind(&RadarVisualization::draw_radar_parsed_msg_208, this, std::placeholders::_1));
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("radar_visualization",1000);
    }
  private:
    void draw_radar_parsed_msg_408(const radar_msgs::msg::RadarMsg408 &msgIn)
    {
      if(msgIn.num_of_objects > 0)
      {
        auto marker = visualization_msgs::msg::Marker();
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        rclcpp::Clock time;
        marker.header.stamp = time.now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "targets_408";
        marker.id = 0;
        marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::msg::Marker::ADD;

        for (int i=0;i<msgIn.num_of_objects;i++)
        {
          marker.id = msgIn.objects[i].obj_id;
          // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
          marker.pose.position.x = msgIn.objects[i].obj_long_displ;
          marker.pose.position.y = msgIn.objects[i].obj_lat_displ;;
          marker.pose.position.z = 0.5;
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;

          // Set the scale of the marker -- 1x1x1 here means 1m on a side
          marker.scale.x = 1.0;
          marker.scale.y = 1.0;
          marker.scale.z = 1.0;

          // Set the color -- be sure to set alpha to something non-zero!
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
          marker.color.a = 1.0;
          
          marker.lifetime = rclcpp::Duration::from_seconds(0.2);
          
          // Publish the marker        
          marker_publisher_->publish(marker);
        }
      } 
    }

    void draw_radar_parsed_msg_208(const radar_msgs::msg::RadarMsg208 &msgIn)
    {
      if (msgIn.num_of_tracks > 0)
      {
        auto marker = visualization_msgs::msg::Marker();
        // Set our initial shape type to be a cube
        uint32_t shape = visualization_msgs::msg::Marker::CYLINDER;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "map";
        rclcpp::Clock time;
        marker.header.stamp = time.now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "targets_208";
        marker.id = 0;
        marker.type = shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::msg::Marker::ADD;

        for (int i=0;i<msgIn.num_of_tracks;i++)
        {
            marker.id = msgIn.objects[i].track_id;
            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
            marker.pose.position.x = msgIn.objects[i].track_long_displ;
            marker.pose.position.y = msgIn.objects[i].track_lat_displ;
            marker.pose.position.z = 0.5;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // Set the scale of the marker -- 1x1x1 here means 1m on a side
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;

            // Set the color -- be sure to set alpha to something non-zero!
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.lifetime = rclcpp::Duration::from_seconds(0.2);
            
            // Publish the marker        
            marker_publisher_->publish(marker);
        }
      } 
    }

    rclcpp::Subscription<radar_msgs::msg::RadarMsg408>::SharedPtr parsed_ars408_subscriber_;
    rclcpp::Subscription<radar_msgs::msg::RadarMsg208>::SharedPtr parsed_srr208_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RadarVisualization>());
  rclcpp::shutdown();
  return 0;
}