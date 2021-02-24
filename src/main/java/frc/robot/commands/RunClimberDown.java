/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunClimberDown extends CommandBase {
  /**
   * Creates a new RunClimberDown.
   */
  boolean finished;
  public RunClimberDown() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    if(!Robot.climber.getLimitBottom()) {
    Robot.climber.down(Constants.CLIMBER_MOTOR_SPEED);
    } else {
    finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Robot.climber.getLimitBottom() == true) {
      finished = true;
      Robot.climber.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.climber.stop();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
