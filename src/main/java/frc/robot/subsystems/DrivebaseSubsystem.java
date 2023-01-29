// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivebaseSubsystem extends SubsystemBase {

  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public WPI_TalonSRX frontrighttalon = new WPI_TalonSRX(Constants.frontrighttalonport);
  public WPI_TalonSRX frontlefttalon = new WPI_TalonSRX(Constants.frontlefttalonport);
  public WPI_VictorSPX backrightvictor = new WPI_VictorSPX(Constants.backrightvictorport);
  public WPI_VictorSPX backleftvictor = new WPI_VictorSPX(Constants.backleftvictorport);

  MotorControllerGroup right = new MotorControllerGroup(frontrighttalon, backrightvictor);
  MotorControllerGroup left = new MotorControllerGroup(frontlefttalon, backleftvictor);

  DifferentialDrive mdrive = new DifferentialDrive(right, left);

  public DrivebaseSubsystem() {
    gyro.reset();
  }

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

  //this returns false when its done because in the autonomous command its checking if its still running or not
  public boolean turnXdegrees(double degree) {
    if (Math.abs(gyro.getAngle() - degree) < 0.25) {
      return false;
    }
    if (Math.abs(gyro.getAngle() - degree) < 1) {
      if (degree < 180) {
        left.set(Constants.autoadjustturn);
        right.set(-Constants.autoadjustturn);
      } 
      if (degree > 180) {
        left.set(Constants.autoadjustturn);
        right.set(Constants.autoadjustturn);
      }
    } else {
      if (degree == 180 ) {
        right.set(Constants.autoturnspeed);
        left.set(-Constants.autoturnspeed);
      }
      if (degree < 180 ) {
        left.set(Constants.autoturnspeed);
        right.set(-Constants.autoturnspeed);
      }
      if (degree > 180) {
        right.set(Constants.autoturnspeed);
        left.set(-Constants.autoturnspeed);
      }
    }
    return true;
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
