// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DrivebaseSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutonomousRightOfStation1BallPad extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DrivebaseSubsystem drivebasesub;

  boolean doingFirstMove = false;
  boolean doingSecondMove = false;
  boolean doingThirdMove = false;
  boolean doingFourthMove = false;
  boolean doingFifthMove = false;
  boolean doingSixthMove = false;

  int currentMove = 1;
  int lastMove = 6;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousRightOfStation1BallPad(DrivebaseSubsystem subsystem) {
    drivebasesub = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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

    //score the point
    //reverse 200 inches
    //turn 90 right (so 270 in the method)
    //move forward ~50 inches
    //turn 90 left (so 90)
    //move forward ~50 inches
    //start auto gyro adjust

    if (currentMove == 1) {
      doingFirstMove = drivebasesub.DriveXInches(-15); //-200
      if (!doingFirstMove) {
        currentMove++; 
        resetEncoderGyro();
      }
    }
    if (currentMove == 2) {
      doingSecondMove = drivebasesub.turnXdegrees(270);
      if (!doingSecondMove) {
        currentMove++;
        resetEncoderGyro();
      }
    }
    if (currentMove == 3) {
      doingThirdMove = drivebasesub.DriveXInches(15); //50
      if (!doingThirdMove) {
        currentMove++;
        resetEncoderGyro();
      }
    }
    if (currentMove == 4) {
      doingFourthMove = drivebasesub.turnXdegrees(90);
      if (!doingFourthMove) {
        currentMove++;
        resetEncoderGyro();
      }
    }
    if (currentMove == 5) {
      doingFifthMove = drivebasesub.DriveXInches(15); //80
      if (!doingFifthMove) {
        currentMove++;
        resetEncoderGyro();
      }
    }
    if (currentMove == 6) {
      doingSixthMove = drivebasesub.autoBalanceOnStation();
      if (!doingSixthMove) {
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
