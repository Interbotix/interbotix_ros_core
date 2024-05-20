
#include <memory>
#include "interbotix_footswitch_driver/footswitch_driver.hpp"

// ACTION!="remove", KERNEL=="event[0-9]*", \
//    ENV{ID_VENDOR_ID}=="3553", \
//    ENV{ID_MODEL_ID}=="b001", \
//    ENV{LIBINPUT_IGNORE_DEVICE}="1"

namespace footswitch_driver
{

FootSwitch::FootSwitch(const rclcpp::NodeOptions & options)
: rclcpp::Node("footswitch_node")
{
  int res;

  // Initialize the hidapi library
  res = hid_init();

  // Open the device using the VID, PID
  handle_ = hid_open(vend_id, prod_id, NULL);
  if (!handle_) {
    RCLCPP_FATAL(get_logger(), "Unable to open device. Exiting...");
    hid_exit();
     ::exit(1);
  }

  // Set the hid_read() function to be non-blocking.
  hid_set_nonblocking(handle_, 1);

  pub_footswitch_state_ = create_publisher<FootswitchState>("state", 10);
  pub_footswitch_state_->publish(FootswitchState());

  tmr_update_state_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&FootSwitch::update_state, this));

  RCLCPP_INFO(get_logger(), "Initialized Footswitch driver.");
}

FootSwitch::~FootSwitch()
{
  RCLCPP_INFO(get_logger(), "Exiting Footswitch driver.");
  // Close the device
  if (handle_) {
    hid_close(handle_);
  }

  // Finalize the hidapi library
  hid_exit();
}

void FootSwitch::update_state()
{
  int res;
  unsigned char buf[65];
  int i;
  res = 0;
  i = 0;
  while (res == 0 && rclcpp::ok()) {
    res = hid_read(handle_, buf, sizeof(buf));
    if (res < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to read(): %ls\n", hid_error(handle_));
      break;
    }

    i++;
    if (i >= 100) {
      break;
    }

    get_clock()->sleep_for(std::chrono::milliseconds(1));
  }

  FootswitchState state;

  if (res > 0) {
    for (int i = 3; i<6; i++) {

      switch (buf[i]) {
        case 0:
          break;
        case 4:
          state.state[0] = true;
          break;
        case 5:
          state.state[1] = true;
          break;
        case 6:
          state.state[2] = true;
          break;

        default:
          break;
      }
    }

    state.header.stamp = now();
    pub_footswitch_state_->publish(state);
  }
}

}  // namespace footswitch_driver


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto footswitch_node = std::make_shared<footswitch_driver::FootSwitch>();
  rclcpp::spin(footswitch_node);

  return 0;
}
