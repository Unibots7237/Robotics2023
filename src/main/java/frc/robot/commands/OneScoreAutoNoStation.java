// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class OneScoreAutoNoStation extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem drivebasesub;
  private final GrabberSubsystem grabsub;
  private final ElevatorSubsystem elevatorsub;

  boolean doingFirstMove = false;
  boolean doingSecondMove = false;
  boolean doingThirdMove = false;
  boolean doingFourthMove = false;

  int currentMove = 1;
  int lastMove = 4;

  public OneScoreAutoNoStation(DrivebaseSubsystem subsystem, GrabberSubsystem grabberSubsystem, ElevatorSubsystem elvsub) {
    drivebasesub = subsystem;
    grabsub = grabberSubsystem;
    elevatorsub = elvsub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem, grabberSubsystem);
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

    //raise the elevator to the third level
    //lift the arm/grabber
    //move forward ~12 inches
    //open the arm

    if (currentMove == 1) {
      doingFirstMove = elevatorsub.raiseToLevel(Constants.ThirdElevatorHeight);
      if (!doingFirstMove) {
        currentMove++; 
        resetEncoderGyro();
      }
    }
    if (currentMove == 2) {
      doingSecondMove = drivebasesub.DriveXInches(12);
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
