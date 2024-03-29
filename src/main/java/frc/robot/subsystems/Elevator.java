/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.ElevatorState;

public class Elevator extends MainSubsystem {
  private WPI_TalonSRX elevator;
  private ElevatorState state;
  private final double kElevatorTolerance = 100.0;
  private HashMap <ElevatorState, Integer> elevatorSetpoints = new HashMap<ElevatorState, Integer>();
  private Command defaultCommand;

  public Elevator(){
    elevator = new WPI_TalonSRX(Constants.kElevatorID);
    elevator.configFactoryDefault();
    elevator.setNeutralMode(NeutralMode.Brake);
    elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    elevator.configNeutralDeadband(0.001, Constants.kTimeoutMs);
    elevator.setSensorPhase(false);
    elevator.setInverted(false);
    elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    elevator.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);
    elevator.configNominalOutputForward(0, Constants.kTimeoutMs);
    elevator.configNominalOutputReverse(0, Constants.kTimeoutMs);
		elevator.configPeakOutputForward(1, Constants.kTimeoutMs);
    elevator.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    elevator.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		elevator.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		elevator.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		elevator.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    elevator.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
    elevator.configMotionCruiseVelocity(Constants.kCruiseVelocity, Constants.kTimeoutMs);
    elevator.configMotionAcceleration(Constants.kAcceleration, Constants.kTimeoutMs);
    elevator.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

    populateSetpoints();
  }
  public void outputTelemetry(){
    SmartDashboard.putString("Elevator State", this.getState().toString());
    SmartDashboard.putNumber("Elevator Pos", this.getEncoderPos());
  }
  public boolean isMotionMagicDone(){
    return Math.abs(this.getClosedLoopTarget() - this.getEncoderPos()) < kElevatorTolerance; 
  }

  public double getClosedLoopTarget(){
    return elevator.getClosedLoopTarget();
  }

  public double getEncoderPos(){
    return elevator.getSelectedSensorPosition();
  }
  public void setMotionMagicPosition(double position) {
    elevator.set(ControlMode.MotionMagic, position);
  }
  public void setState(ElevatorState state){
    this.state = state;
  }
  public int getSetpoint(ElevatorState state){
    setState(state);
    return elevatorSetpoints.get(state);
  }
  public void stop(){
    elevator.stopMotor();
  }
  public ElevatorState getState(){
    return state;
  }
  private void populateSetpoints(){
    elevatorSetpoints.put(ElevatorState.ZERO, 0);
    elevatorSetpoints.put(ElevatorState.LEVEL_1, Constants.kLevel1);
    elevatorSetpoints.put(ElevatorState.LEVEL_2, Constants.kLevel2);
    elevatorSetpoints.put(ElevatorState.LEVEL_3, Constants.kLevel3);
  }

  @Override
  public void initDefaultCommand() {
    
  }
}
