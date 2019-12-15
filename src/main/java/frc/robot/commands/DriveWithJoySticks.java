/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;


public class DriveWithJoySticks extends Command {
  private DriveTrain driveTrain;
  private Joystick leftStick;
  private Joystick rightStick;
  public DriveWithJoySticks(DriveTrain driveTrain, Joystick leftStick, Joystick rightStick) {
    this.driveTrain = driveTrain;
    this.leftStick = leftStick;
    this.rightStick = rightStick;

    requires(this.driveTrain);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    double left = leftStick.getY();
    double right = rightStick.getY();
    driveTrain.setSpeed(left, right);
    // double yAxis = Robot.oi.driveController.getY(Hand.kLeft);
    // double xAxis = Robot.oi.driveController.getX(Hand.kRight);
    // Robot.driveTrain.setSpeed(yAxis, xAxis);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
