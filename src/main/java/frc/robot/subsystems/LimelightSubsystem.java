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

public class LimelightSubsystem extends SubsystemBase {
  public static void limelight(){
    //set important settings
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("stream").setNumber(0);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry tv = table.getEntry("tv");
    
    double y_offset_angle = ty.getDouble(0);
    double x_offset_angle = tx.getDouble(0);
    double any_target_double = tv.getDouble(0);
    double distance_to_target = (Constants.h2 - Constants.h1) / (Math.tan(3.14159*y_offset_angle/180));
    boolean any_target = any_target_double==1 ? true : false;

    SmartDashboard.putNumber("x_offset_angle", y_offset_angle);
    SmartDashboard.putNumber("y_offset_angle", x_offset_angle);
    SmartDashboard.putBoolean("any_targets?", any_target);
    SmartDashboard.putNumber("distance_to_target", distance_to_target);
  }
}