// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ElevatorCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem m_subsystem;
  private final ArmSubsystem armsub;

  private final Timer timer = new Timer();
  private boolean debounce = false;
  private boolean currentlyMovingElevator = false;

  private boolean movingfirst = false;
  private boolean movingsecond = false;
  private boolean movingthird = false;

  private boolean movingelefirst = false;
  private boolean movingelesecond = false;
  private boolean movingelethird = false;

  private boolean movingarmThird = false;
  private boolean movingArmSecond = false;
  private boolean movingArmFirst = false;

  private double elevatorMaintainEncoder;

  
  public ElevatorCommand(ElevatorSubsystem subsystem, ArmSubsystem arms) {
    m_subsystem = subsystem;
    armsub = arms;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  //THIS ALSO CONTROLS THE ARMS WHEN USING THE DPAD BUTTONS. I AM TOO LAZY TO MAKE A COMMAND FOR IT SO IM LEAVING IT HERE
  @Override
  public void execute() {
    SmartDashboard.putNumber("elevator encoder", -(m_subsystem.rightneomotor.getEncoder().getPosition()));
    SmartDashboard.putNumber("target elevator encoder", elevatorMaintainEncoder);
     
    if (!debounce) {
      if (RobotContainer.xboxcontroller.getRightBumper()) {
        m_subsystem.spinNeoMotors(true);
        if (-(m_subsystem.rightneomotor.getEncoder().getPosition()) >= Constants.maxElevatorHeight) {
          elevatorMaintainEncoder = -(m_subsystem.rightneomotor.getEncoder().getPosition());
          m_subsystem.neos.stopMotor();
        }
      } else if(RobotContainer.xboxcontroller.getLeftBumper()) {
        m_subsystem.spinNeoMotors(false);
        if (armsub.armneomotor.getEncoder().getPosition() > Constants.maxMoveElevatorDownArmEncoder) {
        }
      } else{
        /* 
        if (-(m_subsystem.rightneomotor.getEncoder().getPosition()) > elevatorMaintainEncoder) {
          m_subsystem.neos.stopMotor();
        }
        if (-(m_subsystem.rightneomotor.getEncoder().getPosition()) < elevatorMaintainEncoder) {
          m_subsystem.spinNeoMotors(true);
        }*/
        m_subsystem.neos.set(-.03);
      } 
    }

    //dpad right = third level
    //dpad up  = second level
    //dpad left = first level

    if (!debounce) {
      movingthird = false;
      movingsecond = false;
      movingfirst = false;
    }

    if (debounce) {
      if (movingthird) {
        if (movingelethird) {
          movingelethird = this.m_subsystem.raiseToLevel(Constants.ThirdElevatorHeight);
        }
        if (movingarmThird) {
          //movingarmThird = this.armsub.raiseUnraiseArm(Constants.thirdLevelArmHeight);
        }
        if (!movingelethird && !movingarmThird) {
          debounce = false;
          m_subsystem.neos.stopMotor();
          armsub.armneomotor.stopMotor();
        }
      }
      if (movingsecond) {
        if (movingelesecond) {
          movingelesecond = this.m_subsystem.raiseToLevel(Constants.SecondElevatorHeight);
        }
        if (movingArmSecond) {
          //movingArmSecond = this.armsub.raiseUnraiseArm(Constants.secondLevelArmHeight);
        }
        if (!movingsecond && !movingArmSecond) {
          debounce = false;
          m_subsystem.neos.stopMotor();
          armsub.armneomotor.stopMotor();
        }
      }
      if (movingfirst) {
        if (movingelefirst) {
          movingelefirst = this.m_subsystem.raiseToLevel(Constants.FirstElevatorHeight);
        }
        if (movingArmFirst) {
          //movingArmFirst = this.armsub.raiseUnraiseArm(Constants.startingArmHeight);
        }
        if (!movingfirst && movingArmFirst) {
          debounce = false;
          m_subsystem.neos.stopMotor();
          armsub.armneomotor.stopMotor();
        }
      }
    }

    if (!debounce) {
      if (armsub.armneomotor.getEncoder().getPosition() > Constants.maxMoveElevatorDownArmEncoder) {
        if (RobotContainer.xboxcontroller.getPOV() == Constants.DPADRight) {
          movingthird = true;
          movingarmThird = true;
          movingelethird = true;
          debounce = true; 
        } else if(RobotContainer.xboxcontroller.getPOV() == Constants.DPADUp){
          movingArmSecond = true;
          movingelesecond = true;
          movingsecond = true;
          debounce = true;
        } else if (RobotContainer.xboxcontroller.getPOV() == Constants.DPADLeft) {
          movingfirst = true;
          movingArmFirst = true;
          movingelefirst = true;
          debounce = true;
        } else if (RobotContainer.xboxcontroller.getPOV() == Constants.DPADDown) {
          movingthird = false;
          movingarmThird = false;
          movingelethird = false;
          movingsecond = false;
          movingArmSecond = false;
          movingelesecond = false;
          movingfirst = false;
          movingArmFirst = false;

          debounce = false;
          m_subsystem.neos.stopMotor();
          armsub.armneomotor.stopMotor();
        }
      }
    }

    /* 
    if (RobotContainer.xboxcontroller.getPOV() == 180) {
      m_subsystem.rightneomotor.getEncoder().setPosition(0);
    }
    */
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
