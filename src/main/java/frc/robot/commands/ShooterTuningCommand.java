/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

/*
 * This command is designed to be used for any sort of PID calibration that
 * we need to do.  It can be used with a little tweaking to be MotionMagic,
 * MotionProfile, Velocity, or Position.
 * 
 * Feel free to copy the source file to make a new one, or to extend this one
 * and make it your own. This example is using the drive train for the base 
 * subsystem, but can be used with many subsystem by changing the argument type.
 * 
 * The command will use Shuffleboard to put all of it's values that will be used
 * to tune the pid itself.  Then, on robot init, this Command can be sent to the 
 * Dashboard as well for tuning.
 */
public class ShooterTuningCommand extends CommandBase {
  private double m_setpoint, m_tolerance, m_conveyorSpeed, m_conveyorDelay;
  private Timer cmdTimer;

  private final Shooter m_shooter;
  private final ShuffleboardTab m_myTab;
  private NetworkTableEntry m_kpEntry, m_kiEntry, m_kdEntry, m_kfEntry, m_spEntry,m_conveyorEntry, m_conveyorDelayEntry;
  /**
   * Creates a new PIDTuningCommand.
   */
  public ShooterTuningCommand( Shooter shooter ) {
    cmdTimer = new Timer(); // used to delay the conveyor if necessary

    // This sets up all the shuffleboard components needed for testing on the PID Tuning Tab
    m_myTab = Shuffleboard.getTab("PID Tuning");

    // The general process to tune the velocity PID is as follows:
    // 1) Determine (with Phoenix Tuner) what is the RPM for the desired shot,
    //    note the PercentOutput required to generate that speed.
    //    this is your kF value you will use for the rest of the tuning.
    // 2) Start with a small kP, and 0 for kI and kD values.  A starting kP 
    //    of 1 is reasonable.
    // 3) If the kP value is too large there will be some semi-violent-ish 
    //    recovery on the motor (which you can see visually, or in the graph
    //    on Phoenix Tuner), if the kP value is too small, the second and
    //    subsequent shots will not be the proper trajectory (meaning the motor
    //    output is not recovering quickly enough with the PID). Increase/Decrease
    //    by doubling (or halving) it's value until the subsequent shot trajectories
    //    match the first.
    // 4) If for some reason the recover from kP looks "jerky" or not smooth, then 
    //    try introducing the kD value.  Typically a starting kD value is 5x - 10x kP.
    //
    //    For a velocity PID, it's unlikely you'll need anything but kF and kD.
    m_kpEntry = m_myTab.add("kP", 0).withPosition(1, 0).getEntry();
    m_kiEntry = m_myTab.add("kI", 0 ).withPosition(2, 0).getEntry();
    m_kdEntry = m_myTab.add("kD", 0 ).withPosition(3, 0).getEntry();
    m_kfEntry = m_myTab.add("kF", 0 ).withPosition(0, 0).getEntry();
    m_spEntry = m_myTab.add("Set Point", 0 ).withPosition(4, 0).getEntry();

    // Conveyor motor goes backward to get the balls where we want them.
    // Add in a conveyor delay method in case the shooter needs time to ramp up.
    m_conveyorEntry = m_myTab.add("Conveyor Speed", -.75).withPosition(6, 0).getEntry();
    m_conveyorDelayEntry = m_myTab.add("Conveyor Delay", 2).withPosition(7, 0).getEntry();

    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Grab the relevant values for the PID control from Shuffleboard and set the 
    // motor controller configuration accordingly
    double kp = m_kpEntry.getDouble(0);
    double ki = m_kiEntry.getDouble(0);
    double kd = m_kdEntry.getDouble(0);
    double kf = m_kfEntry.getDouble(0);
    m_setpoint = m_spEntry.getDouble(0);
    m_conveyorSpeed = m_conveyorEntry.getDouble(0);
    m_conveyorDelay = m_conveyorDelayEntry.getDouble(2.0);

    m_shooter.configureVelocityPID(kp, ki, kd, kf);

    cmdTimer.reset();
    cmdTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive the shooter motors, as well as the conveyor to start the 
    m_shooter.velocityPID(m_setpoint, m_tolerance);
    if( cmdTimer.get() >= m_conveyorDelay){
      // Turn on the conveyor motor after the delay has been meet.
      m_shooter.startConveyor(m_conveyorSpeed);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Shut the motors off by driving in Percent
    m_shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command ends after 10 seconds, adjust as needed but it's probably enough.
    // It can also just be canceled by clicking the button on Shuffleboard.
    return cmdTimer.get() > 10.0;
  }
}
