/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ConveyorConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class EmptyShooterNoVision extends CommandBase {
  /**
   * Creates a new EmptyShooterNoVision.
   */
  double time, timeStart;
  boolean finished;
  public EmptyShooterNoVision() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.shooter, Robot.conveyor);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time = 2;
    timeStart = System.currentTimeMillis();
    finished = false;
    Robot.shooter.getMotor().setVoltage(ShooterConstants.S_WHEEL_VOLTAGE);
    Robot.conveyor.start(ConveyorConstants.AUTON_CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(((System.currentTimeMillis()-timeStart)/1000) > time) {
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.shooter.stop();
    Robot.conveyor.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
