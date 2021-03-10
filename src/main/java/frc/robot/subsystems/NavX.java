/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
//import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class NavX extends SubsystemBase {
  /**
   * Creates a new NavX.
   */
  //private final DifferentialDriveOdometry m_Odometry;

  AHRS ahrs = new AHRS(SPI.Port.kMXP);

  public NavX() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public float getYaw() {

    return ahrs.getYaw();

  }

  public float getRoll() {

    return ahrs.getRoll();

  }

  public float getPitch() {

    return ahrs.getPitch();

  }

  public float getDisplacementX() {

    return ahrs.getDisplacementX();

  }

  public float getDisplacementY() {

    return ahrs.getDisplacementY();

  }

  public float getDisplacementZ() {

    return ahrs.getDisplacementZ();

  }

  public float getCurrentDegrees() {

    return ahrs.getCompassHeading();

  }

  public Rotation2d getRotation2d()
  {
    return ahrs.getRotation2d();
  }
  public void resetYaw() {

    ahrs.reset();

  }

  public void resetDisplacement() {

    resetDisplacement();

  }

}
