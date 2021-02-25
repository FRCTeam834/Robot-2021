/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.RobotContainer;

public class DriveForwardDistance extends CommandBase {
  /**
   * Creates a new DriveForwardDistance.
   */
  double distance, encoderStart, speed;
  boolean finished;
  public DriveForwardDistance(double speed, double dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.driveTrain);
    distance = dist;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveTrain.resetEncoderPosition();
    //encoderStart = RobotContainer.driveTrain.getRightEncoderValue();
    RobotContainer.driveTrain.setDrive(-speed, -speed);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance <= (-RobotContainer.driveTrain.getRightEncoderValue() * 3.0015)) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
    RobotContainer.driveTrain.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
