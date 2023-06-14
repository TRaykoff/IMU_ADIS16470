// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.ADIS16470_3Axis.IMUAxis;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // CONFIG
  private boolean kUSE_SHUFFLEBOARD = true;

  // Gyro
  ADIS16470_3Axis gyro = new ADIS16470_3Axis(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY);

  // Shuffleboard
  private ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("ADIS16470");

  private GenericEntry yawOut = shuffleboardTab.add("Yaw (deg)", 0.0).getEntry();
  private GenericEntry pitchOut = shuffleboardTab.add("Pitch (deg)", 0.0).getEntry();
  private GenericEntry rollOut = shuffleboardTab.add("Roll (deg)", 0.0).getEntry();

  private GenericEntry xAccel = shuffleboardTab.add("X Velo", 0.0).getEntry();
  private GenericEntry yAccel = shuffleboardTab.add("Y Velo", 0.0).getEntry();
  private GenericEntry zAccel = shuffleboardTab.add("Z Velo", 0.0).getEntry();

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    // Update Shuffleboard
    if (kUSE_SHUFFLEBOARD) {
      yawOut.setDouble(gyro.getAngle(IMUAxis.kYaw));
      pitchOut.setDouble(gyro.getAngle(IMUAxis.kPitch));
      rollOut.setDouble(gyro.getAngle(IMUAxis.kRoll));

      xAccel.setDouble(gyro.getAccelerationX());
      yAccel.setDouble(gyro.getAccelerationY());
      zAccel.setDouble(gyro.getAccelerationZ());
    } else {
      System.out.println(String.format("Angle: %s, %s, %s; Velo: %s, %s, %s",
        gyro.getAngle(gyro.getYawAxis()),
        gyro.getAngle(gyro.getPitchAxis()),
        gyro.getAngle(gyro.getRollAxis()),
        gyro.getAccelerationX(),
        gyro.getAccelerationY(),
        gyro.getAccelerationZ()
      ));
    }
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
