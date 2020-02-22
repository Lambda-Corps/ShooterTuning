/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonSRX m_top, m_bottom, m_conveyor, m_longIntake, m_IndexerIntake;
  /**
   * Creates a new ExampleSubsystem.
   */
  public Shooter() {
    // These CAN IDs were only chosen to match the 2020 robot, they can be any thing
    // that matches.
    m_top = new TalonSRX(9);
    m_bottom = new TalonSRX(8);
    m_conveyor = new TalonSRX(2);
    m_longIntake = new TalonSRX(11);
    m_IndexerIntake = new TalonSRX(10);
    // set talons to factory deafault
    m_top.configFactoryDefault();
    m_bottom.configFactoryDefault();
    m_conveyor.configFactoryDefault();
    m_IndexerIntake.configFactoryDefault();
    m_longIntake.configFactoryDefault();

    // config top motor
    m_top.setInverted(false);
    m_top.setSensorPhase(false);
    m_top.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);

    // config bottom motor
    m_bottom.setInverted(false);
    m_bottom.follow(m_top);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configureVelocityPID(double kp, double ki, double kd, double kf) {
    // Each run of the command (while in tuning test mode) should reconfigure the 
    // talon PID slots with the appropriate PID values.
    m_top.config_kI(0, kp);
    m_top.config_kI(0, ki);
    m_top.config_kD(0, kd);
    m_top.config_kF(0, kf);
  }

  public void velocityPID(double m_setpoint, double m_tolerance) {
    m_top.set(ControlMode.Velocity, m_setpoint);
  }

  public void stopMotors() {
    m_top.set(ControlMode.PercentOutput, 0);
    m_conveyor.set(ControlMode.PercentOutput, 0);
    m_IndexerIntake.set(ControlMode.PercentOutput, 0);
    m_longIntake.set(ControlMode.PercentOutput, 0);
  }

  public void startConveyor(double speed) {
    m_conveyor.set(ControlMode.PercentOutput, speed);
  }

  public void startIntake(double speed){
    m_longIntake.set(ControlMode.PercentOutput, speed);
  }

  public void startIndexer(double speed){
    m_IndexerIntake.set(ControlMode.PercentOutput, speed);
  }

}
