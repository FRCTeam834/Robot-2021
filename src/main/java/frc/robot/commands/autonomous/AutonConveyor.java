/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AutonConveyor extends CommandBase {
  /**
   * Creates a new AutonConveyor.
   */
  double time, speed, timeStart, currentTime, timeElapsed;
  boolean isFinished;
  public AutonConveyor() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 3;
    speed = .75;
    timeStart = System.currentTimeMillis();
    isFinished = false;
  }

  public void initialize(double t, double s) {
    time = t;
    speed = s;
    Robot.conveyor.start(speed);
    timeStart = System.currentTimeMillis();
    isFinished = false;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = System.currentTimeMillis();
    timeElapsed = currentTime - timeStart;
    
    if (timeElapsed < time) {
      isFinished = false;
    }
    else {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isFinished = false;
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
