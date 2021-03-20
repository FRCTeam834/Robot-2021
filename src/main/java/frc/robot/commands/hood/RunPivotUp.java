/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;

public class RunPivotUp extends CommandBase {
  /**
   * Creates a new RunClimberUp.
   */
  boolean finished = false;

  public RunPivotUp() {
    addRequirements(Robot.gimbalLock);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check to make sure that the limit switch hasn't been triggered
    if (Robot.gimbalLock.getLimitSwitch()) {
      Robot.gimbalLock.stop();
      Robot.gimbalLock.resetEncoder();
      finished = true;
    }
    else {
      Robot.gimbalLock.tiltUp(ShooterConstants.SHOOTER_ANGLE_INCREMENT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
