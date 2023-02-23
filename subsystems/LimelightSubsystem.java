// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Limelight extends SubsystemBase {
  public static void limelightStart(){
    //turn on and blink
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(2);
    
    //set important settings
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
  }
  public static void limelightRun(){
    //collect important data
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    
    System.out.println("Hello");
    //check whether any valid targets, then act accordingly
  limelightRun();
  }
}
