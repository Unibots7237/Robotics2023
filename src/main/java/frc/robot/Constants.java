// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  /*for the motors */
  public static final int frontrighttalonport = 1;
  public static final int frontlefttalonport = 2;
  public static final int backrightvictorport = 4;
  public static final int backleftvictorport = 3;

  public static final int leftneomotor = 9;
  public static final int rightneomotor = 10;

  public static final int armneomotor = 11;

  public static int solenoidforward = 0;
  public static int solanoidreverse = 1;

  public static final int xboxcontroller = 0;

  public static final double elevatormotorspeedUP = 0.45;
  public static final double elevatormotorspeedDOWN = .1;
  public static final double autoturnspeed = 0.35;
  public static final double autoadjustturn = 0.2;

  public static final double autonomousdrivespeed = .25;
  public static final double balancespeed = .4;

  public static final double rotationsPerInch = 75;


  //for the elevator
  public static final double FirstElevatorHeight = 5;
  public static final double SecondElevatorHeight = 30;
  public static final double ThirdElevatorHeight = 60;
  public static final double maxElevatorHeight = 90;
  public static final double startingElevatorHeight = 0;

  public static final double startingArmHeight = 0;
  public static final double raisedArmHeight = 50;

  public static final double thirdLevelArmHeight = 40;
  public static final double firstLevelArmHeight = 10; //this is just the limit for when the grabbers can open or close
  public static final double secondLevelArmHeight = 25;

  public static final double armMoveUpSpeed = .05;
  public static final double armMoveDownSpeed = .05;

  public static final double maxMoveElevatorDownArmEncoder = 10;

  public static final double h1 = 5;
  public static final double h2 = 15.13;

  //dpad numbers
  public static final int DPADLeft = 270;
  public static final int DPADUp = 0;
  public static final int DPADRight = 90;
  public static final int DPADDown = 180;
}
