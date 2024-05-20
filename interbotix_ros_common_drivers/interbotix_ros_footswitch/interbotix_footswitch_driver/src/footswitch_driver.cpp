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
  // Initialize the hidapi library
  hid_init();

  // Open the device using the VID, PID
  handle_ = hid_open(VEND_ID, PROD_ID, NULL);
  if (!handle_) {
    RCLCPP_FATAL(
      get_logger(),
      "Unable to open device with VID '%x' and PID '%x'. Exiting...",
      VEND_ID, PROD_ID);
    hid_exit();
    ::exit(1);
  }

  // Set the hid_read() function to be non-blocking.
  hid_set_nonblocking(handle_, 1);

  int update_rate;
  declare_parameter("update_rate", 10);
  get_parameter("update_rate", update_rate);

  update_period_ms_ = static_cast<int64_t>((1.0 / static_cast<float>(update_rate)) * 1'000);

  pub_footswitch_state_ = create_publisher<FootswitchState>("state", 10);

  tmr_update_state_ = create_wall_timer(
    std::chrono::milliseconds(update_period_ms_),
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
  int res = 0;
  unsigned char buf[65];
  int i = 0;

  // Continue to poll if no readings have been received
  while (res == 0 && rclcpp::ok()) {
    res = hid_read(handle_, buf, sizeof(buf));
    if (res < 0) {
      RCLCPP_ERROR(get_logger(), "Unable to read from footswitch: %ls", hid_error(handle_));
      break;
    }

    i++;
    // Break after update period
    if (i >= update_period_ms_) {
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(footswitch_driver::FootSwitch)
