#include "app.h"

#include "device_io.h"
#include "etrc_info.h"
#include "driving.h"
#include "game_play.h"
#include "state_manager.h"


MotorIo* motor_io;
SensorIo* sensor_io;
Luminous* luminous;
Localize* localize;
WheelsControl* wheels_control;
BingoAgent* bingo_agent;
StateManager* state_manager;

static void initialize() {
  motor_io = new MotorIo();
  sensor_io = new SensorIo();
  luminous = new Luminous(sensor_io);
  localize = new Localize(motor_io);
  wheels_control = new WheelsControl(motor_io);
  bingo_agent = new BingoAgent();
  state_manager = new StateManager(wheels_control, luminous);
}

static void finalize() {
  delete state_manager;
  delete bingo_agent;
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
  // tslp_tsk(3000*1000U);

  sta_cyc(EXEC_ACTION_CYC);

  tslp_tsk(10*1000U);

  while (true) {
    if (sensor_io->touch_sensor_pressed_) break;
    tslp_tsk(100*1000U);
  }

  stp_cyc(EXEC_ACTION_CYC);
  stp_cyc(UPDATE_INFO_CYC);
  finalize();
  ext_tsk();
}

void exec_action_task(intptr_t unused) {
  state_manager->Update();
  ext_tsk();
}

void update_info_task(intptr_t unused) {
  motor_io->Update();
  sensor_io->Update();
  luminous->Update();

  ext_tsk();
}
