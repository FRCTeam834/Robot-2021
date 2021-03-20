/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class HoodHome extends CommandBase {
  /**
   * Creates a new HoodHome.
   */
  boolean finished = false;

  public HoodHome() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.gimbalLock);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.gimbalLock.setSpeed(.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Check to see if the hood has hit it's home position
    if (Robot.gimbalLock.getLimitSwitch()) {

      // Stop motor and set the home position
      Robot.gimbalLock.stop();
      Robot.gimbalLock.resetEncoder();

      // Finished should be set last in case of interrupt
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.gimbalLock.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
