/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.snapto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SnapToGoal extends CommandBase {
  /**
   * Creates a new SnapToGoal.
   */
  double lMotor, rMotor, angle;
  boolean finished;
  public SnapToGoal(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain, Robot.navX);   
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
  
    lMotor = 1;
    rMotor = -1;
    Robot.driveTrain.setDrive(.5, -.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(Robot.navX.getYaw()) >= (angle-15)) { // spin slower once close for percision 
      Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
      if(Math.abs(Robot.navX.getYaw()) >= (angle-5)) {
        //now the that we are facing angle, we can gg ez stop spinning
        finished = true;
      }
     } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setDrive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
