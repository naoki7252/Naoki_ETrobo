#include "app.h"

#include "device_io.h"
#include "etrc_info.h"
#include "driving.h"
#include "state_manager.h"

#if defined(MAKE_SIM)
static const bool kSimulator = true;
#else
static const bool kSimulator = false;
#endif

#if defined(MAKE_RIGHT)
static const bool kRcourse = true;
#else
static const bool kRcourse = false;
#endif

MotorIo* motor_io;
SensorIo* sensor_io;
Luminous* luminous;
Localize* localize;
WheelsControl* wheels_control;
StateManager* state_manager;

static void initialize() {
  motor_io = new MotorIo();
  sensor_io = new SensorIo();
  luminous = new Luminous(sensor_io);
  localize = new Localize(motor_io);
  wheels_control = new WheelsControl(motor_io);
  state_manager = new StateManager(wheels_control);
}

static void finalize() {
  delete state_manager;
  delete wheels_control;
  delete localize;
  delete luminous;
  delete sensor_io;
  delete motor_io;
}


void main_task(intptr_t unused) {
  initialize();
  sta_cyc(UPDATE_INFO_CYC);

  while (true) {
    if (sensor_io->touch_sensor_pressed_) break;
    tslp_tsk(10*1000U);
  }
  tslp_tsk(3000*1000U);

  sta_cyc(EXEC_ACTION_CYC);

  tslp_tsk(10*1000U);

  while (true) {
    if (sensor_io->back_button_pressed_) break;
    tslp_tsk(100*1000U);
  }

  stp_cyc(EXEC_ACTION_CYC);
  stp_cyc(UPDATE_INFO_CYC);
  finalize();
  ext_tsk();
}

void exec_action_task(intptr_t unused) {
  state_manager->TestRun();
  ext_tsk();
}

void update_info_task(intptr_t unused) {
  motor_io->Update();
  sensor_io->Update();
  luminous->Update();

  ext_tsk();
}
