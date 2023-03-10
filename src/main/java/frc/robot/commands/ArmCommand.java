// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArmCommand  extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ArmSubsystem m_subsystem;
  private final ElevatorSubsystem elevatorsub;

  boolean debounce = false;
  boolean raisingArm  = false;
  boolean droppingArm = false;

  public boolean autonomousRaiseHoldArm = false;
  
  public ArmCommand(ArmSubsystem subsystem, ElevatorSubsystem elesub) {
    m_subsystem = subsystem;
    elevatorsub = elesub;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    double currentArmEncoder = -1*(m_subsystem.armneomotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Arm neo encoder", currentArmEncoder);
    SmartDashboard.putNumber("Target arm neo encoder", m_subsystem.currentArmTargetEncoder);


    double move = RobotContainer.xboxcontroller.getRightY();
    move = move/5;

    /* 
    if (move > .1) {
      move = .5; 
    }
    if (move < -.1) {
      move = -.15;
    }*/
    if (move < 0.05 && move > -0.05) {
      move = 0;
    }


    if (move == 0) {
      if (currentArmEncoder > 6.75) {
        m_subsystem.armneomotor.set(-0.05);
      }
      if (currentArmEncoder < 6.75) {
        m_subsystem.armneomotor.set(-0.025);
      }
      /* 
      if (currentArmEncoder < (m_subsystem.currentArmTargetEncoder)) {
        m_subsystem.armneomotor.set(0.05);
      }
      if (currentArmEncoder > (m_subsystem.currentArmTargetEncoder)) {
      }
      */
    }
    


    if (move != 0) {
      m_subsystem.currentArmTargetEncoder = (m_subsystem.armneomotor.getEncoder().getPosition())*-1;
      m_subsystem.armneomotor.set(move);
    }
    

    if (!debounce) {
      if (-elevatorsub.rightneomotor.getEncoder().getPosition() >= 5) {
      
      } else{
        //m_subsystem.moveArm(0);
      }
    }

    /* 
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

    if ((move < 0.05) && (move > -.05)) {
      if (-elevatorsub.rightneomotor.getEncoder().getPosition() >= 5) {
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
    }
    */
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
