#include "triangulation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false); // Enable intra-process communication if saver is in the same process
    auto node = std::make_shared<TriangulationNode>(options);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}