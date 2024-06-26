package frc.robot.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.ShamLib.HID.CommandFlightStick;

public class RealControllerBindings implements ControllerBindings {

  private final CommandXboxController operatorController =
      new CommandXboxController(Constants.Controller.OPERATOR_CONTROLLER_ID);
  private final CommandFlightStick leftFlightStick =
      new CommandFlightStick(Constants.Controller.LEFT_FLIGHT_STICK_ID);
  private final CommandFlightStick rightFlightStick =
      new CommandFlightStick(Constants.Controller.RIGHT_FLIGHT_STICK_ID);

  @Override
  public double getDriveXValue() {
    return -leftFlightStick.getY();
  }

  @Override
  public double getDriveYValue() {
    return -leftFlightStick.getX();
  }

  @Override
  public double getDriveTurnValue() {
    return -rightFlightStick.getRawAxis(0);
  }

  @Override
  public Trigger tuningIncrement() {
    return operatorController.povUp();
  }

  @Override
  public Trigger tuningDecrement() {
    return operatorController.povDown();
  }

  @Override
  public Trigger tuningStop() {
    return operatorController.a();
  }

  @Override
  public Trigger resetGyro() {
    return leftFlightStick.topLeft();
  }

  @Override
  public Trigger traverse() {
    return operatorController.a();
  }

  // TODO: This wasn't integrated super well in the 2024 season
  @Override
  public Trigger xShape() {
    return ControllerBindings.super.xShape();
  }

  @Override
  public void setRumble(double rumbleValue) {
    operatorController.getHID().setRumble(RumbleType.kBothRumble, rumbleValue);
  }
}
