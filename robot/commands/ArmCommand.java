// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmCommand  extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;

  boolean debounce = false;
  boolean raisingArm  = false;
  boolean droppingArm = false;
  
  public ArmCommand(ArmSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double move = RobotContainer.xboxcontroller.getRightY();
    if (!debounce) {
      if (move > 0.05) {
        m_subsystem.moveArm(move);
      }
      if (move < -0.05) {
        m_subsystem.moveArm(move);
      }
    }

    if (raisingArm) {
      raisingArm = m_subsystem.raiseUnraiseArm(Constants.raisedArmHeight);
      if (!raisingArm) {
        debounce = false;
      }
    }
    if (droppingArm) {
      droppingArm = m_subsystem.raiseUnraiseArm(Constants.raisedArmHeight);
      if (!droppingArm) {
        debounce = false;
      }
    }

    if (!debounce) {
      if (RobotContainer.xboxcontroller.getRightStickButton()) {
        if (!m_subsystem.armRaised) {
          raisingArm = true;
          debounce = true;
          m_subsystem.armRaised = true;
        }
        if (m_subsystem.armRaised) {
          droppingArm = true;
          debounce = true;
          m_subsystem.armRaised = false;
        }
      }
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
