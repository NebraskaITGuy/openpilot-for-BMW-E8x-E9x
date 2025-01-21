from cereal import car
from openpilot.selfdrive.car import DT_CTRL, apply_dist_to_meas_limits, apply_hysteresis
from openpilot.selfdrive.car.bmw import bmwcan
from openpilot.selfdrive.car.bmw.bmwcan import SteeringModes, CruiseStalk
from openpilot.selfdrive.car.bmw.values import CarControllerParams, CanBus, BmwFlags
from openpilot.selfdrive.car.interfaces import CarControllerBase
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car.conversions import Conversions as CV

VisualAlert = car.CarControl.HUDControl.VisualAlert

# DO NOT CHANGE: Cruise control step size
CC_STEP = 1 # cruise single click jump - always 1 - interpreted as km or miles depending on DSC or DME set units
CRUISE_STALK_IDLE_TICK_STOCK = 0.2 # stock cruise stalk CAN frequency when stalk is not pressed is 5Hz
CRUISE_STALK_HOLD_TICK_STOCK = 0.05 # stock cruise stalk CAN frequency when stalk is pressed is 20Hz

CRUISE_STALK_SINGLE_TICK = CRUISE_STALK_IDLE_TICK_STOCK # we will send also at 5Hz in between stock messages to emulate single presses
CRUISE_STALK_HOLD_TICK = 0.01 # emulate held stalk, 100Hz makes stock messages be ignored

CRUISE_SPEED_HYST_GAP = CC_STEP * 0.6  # between >0.5 and <1 to avoid cruise speed toggling. More than 0.5 to add some phase lead
ACCEL_HYST_GAP = 0.05 # m/s^2

ACCEL_HOLD_MEDIUM = 0.4
DECEL_HOLD_MEDIUM = -0.6
ACCEL_HOLD_STRONG = 1.2
DECEL_HOLD_STRONG = -1.2

class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP):
    super().__init__(dbc_name, CP)
    self.flags = CP.flags
    self.min_cruise_speed = CP.minEnableSpeed
    self.cruise_units = None

    self.cruise_cancel = False  # local cruise control cancel
    self.cruise_enabled_prev = False
    # redundant safety check with the board
    self.apply_steer_last = 0
    self.last_cruise_rx_timestamp = 0 # stock cruise buttons
    self.last_cruise_tx_timestamp = 0 # openpilot commands
    self.tx_cruise_stalk_counter_last = 0
    self.rx_cruise_stalk_counter_last = -1
    self.cruise_speed_with_hyst = 0
    self.accel_with_hyst = 0
    self.accel_with_hyst_last = 0
    self.calc_desired_speed = 0

    self.cruise_bus = CanBus.PT_CAN
    if CP.flags & BmwFlags.DYNAMIC_CRUISE_CONTROL:
      self.cruise_bus = CanBus.F_CAN


    self.packer = CANPacker(dbc_name)


  def update(self, CC, CS, now_nanos):

    actuators = CC.actuators
    can_sends = []

    self.cruise_units = (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # steer command
    if self.flags & BmwFlags.STEPPER_SERVO_CAN:
      steer_error =  not CC.latActive and CC.enabled
      if not steer_error: # stop steer CAN tx if steering is unavailable (unless user cancels) #todo soft off when user didn't cancelled
        # *** apply steering torque ***
        if CC.enabled:
          new_steer = actuators.steer * CarControllerParams.STEER_MAX
          # explicitly clip torque before sending on CAN
          apply_steer = apply_dist_to_meas_limits(new_steer, self.apply_steer_last, CS.out.steeringTorqueEps,
                                             CarControllerParams.STEER_DELTA_UP, CarControllerParams.STEER_DELTA_DOWN,
                                             CarControllerParams.STEER_ERROR_MAX, CarControllerParams.STEER_MAX)
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.TorqueControl, apply_steer))
        else:
          apply_steer = 0
          can_sends.append(bmwcan.create_steer_command(self.frame, SteeringModes.Off))
        self.apply_steer_last = apply_steer

    # debug
    if CC.enabled and (self.frame % 10) == 0: #slow print
      frame_number = self.frame
      print(f"Steering req: {actuators.steer}, Speed: {CS.out.vEgo}, Frame number: {frame_number}")

    self.cruise_enabled_prev = CC.enabled

    new_actuators = actuators.as_builder()
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX
    new_actuators.steerOutputCan = self.apply_steer_last

    new_actuators.speed = self.calc_desired_speed
    new_actuators.accel = speed_err_req

    self.frame += 1
    return new_actuators, can_sends
