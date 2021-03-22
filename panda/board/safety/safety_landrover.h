#define _TEST_TORQ_ 1
const int LANDROVER_MAX_STEER = 1023;
const int LANDROVER_MAX_RT_DELTA = 1023;         // max delta torque allowed for real time checks
const uint32_t LANDROVER_RT_INTERVAL = 250000;  // 250ms between real time checks
const int LANDROVER_MAX_RATE_UP = 25;
const int LANDROVER_MAX_RATE_DOWN = 30;
const int LANDROVER_MAX_TORQUE_ERROR = 800;    // max torque cmd in excess of torque motor

bool landrover_camera_detected = 0;             // is giraffe switch 2 high?
int landrover_rt_torque_last = 0;
int landrover_desired_torque_last = 0;
int landrover_cruise_engaged_last = 0;
uint32_t landrover_ts_last = 0;
struct sample_t landrover_torque_meas;         // last few torques measured

static int landrover_rx_hook(CAN_FIFOMailBox_TypeDef *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

#if 0
  // Measured eps torque
  if (addr == 544) {
    int torque_meas_new = ((GET_BYTE(to_push, 4) & 0x7U) << 8) + GET_BYTE(to_push, 5) - 1024U;

    // update array of samples
    update_sample(&landrover_torque_meas, torque_meas_new);
  }
#endif

  // enter controls on rising edge of ACC, exit controls on ACC off
  if (addr == 0x156) {
    int cruise_engaged = ((GET_BYTE(to_push, 4) & 0x40) >> 6) == 1;

    if (cruise_engaged && !landrover_cruise_engaged_last) {
      controls_allowed = 1;
    }
    if (!cruise_engaged) {
      controls_allowed = 0;
    }
    landrover_cruise_engaged_last = cruise_engaged;
  }

  // check if stock camera ECU is still online
  if ((bus == 0) && (addr == 0x28F)) {
    landrover_camera_detected = 1;
    controls_allowed = 0;
  }
  return 1;
}

static int landrover_tx_hook(CAN_FIFOMailBox_TypeDef *to_send) {

  int tx = 1;

  // If camera is on bus 1, then nothing can be sent
  if (landrover_camera_detected) {
    tx = 0;
  }

  int addr = GET_ADDR(to_send);

  // LKA STEER
  if (addr == 0x28F) {
    int desired_torque = ((GET_BYTE(to_send, 3) & 0x7U) << 8) + GET_BYTE(to_send, 4) - 1024U;
    uint32_t ts = TIM2->CNT;
    bool violation = 0;

    if (controls_allowed) {
      // *** global torque limit check ***
      violation |= max_limit_check(desired_torque, LANDROVER_MAX_STEER, -LANDROVER_MAX_STEER);

      // *** torque rate limit check ***
      //violation |= dist_to_meas_check(desired_torque, landrover_desired_torque_last,
        //&landrover_torque_meas, LANDROVER_MAX_RATE_UP, LANDROVER_MAX_RATE_DOWN, LANDROVER_MAX_TORQUE_ERROR);

      // used next time
      landrover_desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      violation |= rt_rate_limit_check(desired_torque, landrover_rt_torque_last, LANDROVER_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, landrover_ts_last);
      if (ts_elapsed > LANDROVER_RT_INTERVAL) {
        landrover_rt_torque_last = desired_torque;
        landrover_ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed  && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) {
      landrover_desired_torque_last = 0;
      landrover_rt_torque_last = 0;
      landrover_ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  // TODO: fix bug preventing the button msg to be fwd'd on bus 2

  // 1 allows the message through
  // tx = 1;    // if 1 then all packet pass, so, do not run else find steer angle.
#ifdef _TEST_TORQ_
  tx = 1;
#endif
  return tx;
}

static void landrover_init(int16_t param) {
  UNUSED(param);
  controls_allowed = 0;
  landrover_camera_detected = 0;
}

static int landrover_fwd_hook(int bus_num, CAN_FIFOMailBox_TypeDef *to_fwd) {

  int bus_fwd = -1;
  int addr = GET_ADDR(to_fwd);
  // forward CAN 0 -> 2 so stock LKAS camera sees messages
  if (bus_num == 0) {
    bus_fwd = 2;
  }
  // forward all messages from camera except LKAS_COMMAND and LKAS_HUD
  if ((bus_num == 2) && !landrover_camera_detected && (addr != 0x28F) && (addr != 0x1D8) && (addr != 0x3D4)) {
  // if (bus_num == 2) {
    bus_fwd = 0;
  }
  return bus_fwd;
}


const safety_hooks landrover_hooks = {
  .init = landrover_init,
  .rx = landrover_rx_hook,
  .tx = landrover_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  // .ignition = default_ign_hook,
  .fwd = landrover_fwd_hook,
};
