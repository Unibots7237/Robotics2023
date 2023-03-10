// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivebaseSubsystem extends SubsystemBase {

  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  public static AHRS gyroNAVX = new AHRS(I2C.Port.kOnboard);

  public WPI_TalonSRX frontrighttalon = new WPI_TalonSRX(Constants.frontrighttalonport);
  public WPI_TalonSRX frontlefttalon = new WPI_TalonSRX(Constants.frontlefttalonport);
  public WPI_VictorSPX backrightvictor = new WPI_VictorSPX(Constants.backrightvictorport);
  public WPI_VictorSPX backleftvictor = new WPI_VictorSPX(Constants.backleftvictorport);

  MotorControllerGroup right = new MotorControllerGroup(frontrighttalon, backrightvictor);
  MotorControllerGroup left = new MotorControllerGroup(frontlefttalon, backleftvictor);

  DifferentialDrive mdrive = new DifferentialDrive(right, left);

  public SensorCollection encoderLeft = new SensorCollection(frontlefttalon);
  public SensorCollection encoderRight = new SensorCollection(frontrighttalon);

  public DrivebaseSubsystem() {
    gyro.reset();
    encoderLeft.setQuadraturePosition(0, 0);
    encoderRight.setQuadraturePosition(0, 0);
  }

  public void teleopDrive(double move, double turn) {

     if (Math.abs(move) <= .05) {
         move = 0;
     }
     if (Math.abs(turn) <= .05) {
         turn = 0;
     }

     SmartDashboard.putNumber("Left Encoder", encoderLeft.getQuadraturePosition());
     SmartDashboard.putNumber("Right Encoder", encoderRight.getQuadraturePosition());
     SmartDashboard.putNumber("Gyroscope", gyro.getAngle());
     SmartDashboard.putNumber("NAVX Pitch", gyroNAVX.getPitch());
     SmartDashboard.putNumber("D-PAD number", RobotContainer.xboxcontroller.getPOV());
     mdrive.arcadeDrive(turn, move);
  }

  //this returns false when its done because in the autonomous command its checking if its still running or not
  public boolean turnXdegrees(double x) {

    boolean turningClockWise = false;
    double degree = gyro.getAngle();

    if (x > 180) {
      x = 360-x;
      turningClockWise = true;
    }
    if (Math.abs(degree) - x < .75 && Math.abs(degree) - x > -.75) {
        return false;
    } 
    else if (turningClockWise) {
        left.set(Constants.autoturnspeed);
        right.set(Constants.autoturnspeed);
    }
    else {
        left.set(-Constants.autoturnspeed);
        right.set(-Constants.autoturnspeed);
      } 

    return true;
  }

  //returns false when its done
  //uses gyro to auto-balance
  public boolean DriveXInches(double inches) {
    double rotations = inches * Constants.rotationsPerInch;

    boolean increaseLeftSpeed = false;
    boolean increaseRightSpeed = false;

    System.out.println(gyro.getAngle());


    if ((encoderRight.getQuadraturePosition()- Math.abs(rotations)) > 150) {
      if (rotations > 0) {
        if (gyro.getAngle() >= .5) {
          increaseRightSpeed = true;
        }
        if (gyro.getAngle() <= -.5) {
          increaseLeftSpeed = true;
        }
      }
      if (rotations < 0) {
        if (gyro.getAngle() >= .5) {
          increaseLeftSpeed = true;
        }
        if (gyro.getAngle() <= -.5) {
          increaseRightSpeed = true;
        }
      }
    }

    if (rotations > 0) {
      if ((encoderLeft.getQuadraturePosition()*-1) < rotations) {
        if (increaseLeftSpeed) {
          left.set((Constants.autonomousdrivespeed+0.05));
        } else{
          left.set(Constants.autonomousdrivespeed);
        }
      } 
      if ((encoderRight.getQuadraturePosition()) < rotations) {
        if (increaseRightSpeed) {
          right.set(-(Constants.autonomousdrivespeed + .05));
        } else{
          right.set(-Constants.autonomousdrivespeed);
        }
      } 

      if ((encoderRight.getQuadraturePosition()) >= rotations &&
          (encoderLeft.getQuadraturePosition()*-1) >= rotations) {
          return false;
      }
    } else {
      if ((encoderLeft.getQuadraturePosition()*-1) > rotations) {
        if (increaseLeftSpeed) {
          left.set(-(Constants.autonomousdrivespeed+0.05));
        } else{
          left.set(-Constants.autonomousdrivespeed);
        }
      } 
      if ((encoderRight.getQuadraturePosition()) > rotations) {
        if (increaseRightSpeed) {
          right.set((Constants.autonomousdrivespeed+0.05));
        } else{
          right.set(Constants.autonomousdrivespeed);
        }
      } 

      if ((encoderRight.getQuadraturePosition()) <= rotations &&
          (encoderLeft.getQuadraturePosition()*-1) <= rotations) {
          return false;
      }
    }

    return true;
  } 


  private boolean timerStart = false;
  private Timer timer = new Timer();
  //up is positive, pointing down is negative
  public boolean autoBalanceOnStation() {
    if (gyroNAVX.getPitch() < 5 && gyroNAVX.getPitch() > -5) {
      return false;
    }
    //its pointing up, meaning it needs to move forward to balance on the charging station
    if (gyroNAVX.getPitch() > 5) {
      right.set(-Constants.balancespeed);
      left.set(Constants.balancespeed);
    }
    //now its pointing down so it needs to go back
    if (gyroNAVX.getPitch() < -5) {
      right.set(Constants.balancespeed);
      left.set(-Constants.balancespeed);    
    }

    if (gyroNAVX.getPitch() <2 && gyroNAVX.getPitch() >-2) {
      if (!timerStart) {
        timerStart = true;
        timer.start();
      }
      if (timer.hasElapsed(1)) {
        left.set(0);
        right.set(0);
        frontrighttalon.setNeutralMode(NeutralMode.Brake);
        backrightvictor.setNeutralMode(NeutralMode.Brake);
        frontlefttalon.setNeutralMode(NeutralMode.Brake);
        backleftvictor.setNeutralMode(NeutralMode.Brake);
        return false;
      }
    }

    return true;
  }
}
