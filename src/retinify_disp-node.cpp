#include "retinify_disp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //     bool run_saver_in_process = false;
    // for (int i = 4; i < argc; ++i) {
    //     std::string a = argv[i];
    //     if (a == "--with-saver" || a == "--inproc-saver") {
    //         run_saver_in_process = true;
    //     }
    // }
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(false); // Enable intra-process communication if saver is in the same process
    auto node = std::make_shared<RetinifyDisparityNode>(options);


    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}