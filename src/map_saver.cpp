#include <fstream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class AutosaveMapNode : public rclcpp::Node
{
public:
  AutosaveMapNode()
    : Node("autosave_map_node")
  {
    using namespace cv;
    using namespace nav_msgs::msg;
    
    map_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("map",0.05, std::bind(&AutosaveMapNode::autosavemap_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_; 

  void autosavemap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    using namespace cv;

    float resolution = msg->info.resolution;
    float origin_x = msg->info.origin.position.x;
    float origin_y = msg->info.origin.position.y;

    // Initialize pointer to data
    uint8_t *data = reinterpret_cast<uint8_t*>(msg->data.data());
    uint8_t cur_point = data[0];

    bool isMapEmpty = false;
    Mat im(msg->info.height, msg->info.width, CV_8UC1);

    // Constexpr variable declaration
    int kFreeCellCarto = 0;
    int kOccupiedLowCellCarto = 70;
    int kOccupiedHighCellCarto = 100;
    int kWhitePixel = 254;
    int kBlackPixel = 0;
    int kGrayPixel = 205;
    bool isMapWhite = false;

    std::ofstream myfile;
    myfile.open("map_values.txt");
    myfile << "Writing this to a file.\n";
    RCLCPP_INFO(this->get_logger(), "Map size: %zu", msg->data.size());

    // Transform the map in the same way the map_saver component does by populating the image matrix with the corresponding pixel values
    for (size_t i = 0; i < msg->data.size(); i++)
    {
      // Save the values in a text file
      myfile << static_cast<int>(data[i]) << " ";

      if (data[i] == kFreeCellCarto)
      {
        isMapWhite = true;
        im.data[i] = kWhitePixel;
      }
      else if (data[i] >= kOccupiedLowCellCarto && data[i] <= kOccupiedHighCellCarto)
      {
        im.data[i] = kBlackPixel;
      }
      else
      {
        im.data[i] = kGrayPixel;
      }

      // Verify if the map has values
      if (i != 0)
      {
        isMapEmpty = isMapEmpty || (data[i] != cur_point);
      }
      cur_point = data[i];
    }

    RCLCPP_INFO(this->get_logger(), "The map contains white pixels: %d", isMapWhite);
    myfile.close();

    // Sanity check
    if (!isMapEmpty)
    {
      RCLCPP_INFO(this->get_logger(), "Map is empty, ignoring update.");
    }

    flip(im, im, 0);  // Flip the image in the x-axis
    imwrite("map.pgm", im);

  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AutosaveMapNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
