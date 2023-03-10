// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.RobotContainer;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class LimelightCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private static DrivebaseSubsystem drivebasesub;
    private GrabberSubsystem grabsub;
    private ElevatorSubsystem elevatorsub;
    private ArmSubsystem armsub;
    private LimelightSubsystem limelightsub;
    private LimelightSubsystem m_subsystem;

    public LimelightCommand(DrivebaseSubsystem drivebasesubsystem, GrabberSubsystem grabbersubsystem, ElevatorSubsystem elevatorsubsystem, ArmSubsystem armsubsystem, LimelightSubsystem limelightsubsystem) {
        drivebasesub = drivebasesubsystem;
        grabsub = grabbersubsystem;
        elevatorsub = elevatorsubsystem;
        armsub = armsubsystem;
        limelightsub = limelightsubsystem;
        addRequirements(drivebasesub, grabsub, elevatorsub, armsub, limelightsub);
    }

    public static boolean turn_to_target(double x_offset_angle) {
        boolean doingTurn;
        if (x_offset_angle > 0.75) {
            doingTurn = drivebasesub.turnXdegrees(-x_offset_angle);
            if (!doingTurn) {
                return false;
            }
        }
        if (x_offset_angle < -0.75) {
            doingTurn = drivebasesub.turnXdegrees(x_offset_angle);
            if (!doingTurn) {
                return false;
            }
        }
        return true;
    }
    public static Void limelightOn(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        return null;
    }
    public static Void limelightOff(){
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
        return null;
    }
}