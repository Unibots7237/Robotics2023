// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivebaseSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public DrivebaseSubsystem() {}

  public WPI_TalonSRX frontrighttalon = new WPI_TalonSRX(Constants.frontrighttalonport);
  public WPI_TalonSRX frontlefttalon = new WPI_TalonSRX(Constants.frontlefttalonport);
  public WPI_VictorSPX backrightvictor = new WPI_VictorSPX(Constants.backrightvictorport);
  public WPI_VictorSPX backleftvictor = new WPI_VictorSPX(Constants.backleftvictorport);

  MotorControllerGroup right = new MotorControllerGroup(frontrighttalon, backrightvictor);
  MotorControllerGroup left = new MotorControllerGroup(frontlefttalon, backleftvictor);

  DifferentialDrive mdrive = new DifferentialDrive(right, left);

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public void teleopDrive(double move, double turn) {

    // SmartDashboard.putNumber("Left Encoder", Robot.m_robotContainer.drivebasesub.encoderLeft.getQuadraturePosition());
   //  SmartDashboard.putNumber("Right Encoder", Robot.m_robotContainer.drivebasesub.encoderRight.getQuadraturePosition());

     if (Math.abs(move) <= .05) {
         move = 0;
     }
     if (Math.abs(turn) <= .05) {
         turn = 0;
     }

     SmartDashboard.putNumber("move", move);
     SmartDashboard.putNumber("turn", turn);
     mdrive.arcadeDrive(turn, move);
 }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
