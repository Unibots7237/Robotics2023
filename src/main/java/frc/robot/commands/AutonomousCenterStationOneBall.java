// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonomousCenterStationOneBall extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private DrivebaseSubsystem drivebasesub;
  private GrabberSubsystem grabsub;
  private ElevatorSubsystem elevatorsub;
  private ArmSubsystem armsub;
  private LimelightSubsystem limelightsub;

  private Timer timer = new Timer();
  
  boolean doingFirstMove = false;
  boolean doingSecondMove = false;
  boolean doingThirdMove = false;
  boolean doingFourthMove = false;
  boolean doingFifthMove = false;
  boolean doingSixthMove = false;
  boolean doingSeventhMove = false;
  boolean doingEightMove = false;
  boolean doingNinthMove = false;

  int currentMove = 1;
  int lastMove = 9;

  public AutonomousCenterStationOneBall(DrivebaseSubsystem subsystem, GrabberSubsystem grabbersubsystem, ElevatorSubsystem elevatorsubsystem, ArmSubsystem armsubsystem, LimelightSubsystem limelightsubsystem) {
    drivebasesub = subsystem;
    grabsub = grabbersubsystem;
    elevatorsub = elevatorsubsystem;
    armsub = armsubsystem;
    limelightsub = limelightsubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, grabsub, armsub, elevatorsub, limelightsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentMove = 1;
    drivebasesub.gyro.reset();
    drivebasesub.encoderLeft.setQuadraturePosition(0, 0);
    drivebasesub.encoderRight.setQuadraturePosition(0, 0);
  }

  public void resetEncoderGyro() {
    drivebasesub.encoderLeft.setQuadraturePosition(0,0);
    drivebasesub.encoderRight.setQuadraturePosition(0,0);
    drivebasesub.gyro.reset();
    drivebasesub.gyroNAVX.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

  if (currentMove == 1) {
    doingFirstMove = elevatorsub.raiseToLevel(Constants.ThirdElevatorHeight);
    if (!doingFirstMove) {
      currentMove++; 
      resetEncoderGyro();
    }
  }
  if (currentMove == 2) {
    armsub.currentArmTargetEncoder = Constants.thirdLevelArmHeight;
    armsub.armRaised = true;
    if (!doingSecondMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 3) {
    doingThirdMove = drivebasesub.DriveXInches(12);
    if (!doingThirdMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 4) {
    grabsub.RetractGrabbers();
    currentMove++;
  }
  if (currentMove == 5) {
    doingFifthMove = drivebasesub.DriveXInches(-12);
    if (!doingFifthMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 6) {
    doingSixthMove = elevatorsub.raiseToLevel(Constants.SecondElevatorHeight);
    if (!doingSixthMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 7) {
    doingSeventhMove = armsub.raiseUnraiseArm(Constants.startingArmHeight);
    if (!doingSeventhMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 8) {
    doingEightMove = drivebasesub.DriveXInches(-15); //x = 65
    if (!doingEightMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  if (currentMove == 9) {
    doingNinthMove = drivebasesub.autoBalanceOnStation();
    if (!doingNinthMove) {
      currentMove++;
      resetEncoderGyro();
    }
  }
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
