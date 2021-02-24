/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveBackwardsDistance extends CommandBase {
  /**
   * Creates a new DriveBackwardsDistance.
   */
  double distance, encoderStart;
  boolean finished;
  public DriveBackwardsDistance(double dist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    distance = dist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    Robot.driveTrain.resetEncoderPosition();
    Robot.driveTrain.setDrive(.2, .2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance <= (Robot.driveTrain.getRightEncoderValue() *3.015)) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = false;
    Robot.driveTrain.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
