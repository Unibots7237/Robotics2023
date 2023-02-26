// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


public class ArmSubsystem extends SubsystemBase {

  public CANSparkMax armneomotor = new CANSparkMax(Constants.armneomotor, MotorType.kBrushless);


   public ArmSubsystem() {
    armneomotor.getEncoder().setPosition(Constants.startingArmHeight);
   }

   public void moveArm(double move) {
    armneomotor.set(move);
   }

  public boolean raiseUnraiseArm(double height) {
    double encoder = armneomotor.getEncoder().getPosition();

    boolean goingDown = false;
    if (height < encoder) {goingDown = true;}

    if (goingDown) {
      armneomotor.set(-Constants.armMoveDownSpeed);
      if (encoder <= height+0.5) {
        armneomotor.stopMotor();
        armneomotor.set(0);
        return false;
      }
    }
    if (!goingDown) {
      armneomotor.set(Constants.armMoveDownSpeed);
      if (encoder <= height-0.5) {
        armneomotor.stopMotor();
        armneomotor.set(0);
        return false;
      }
    }

    return true;
  }   

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
