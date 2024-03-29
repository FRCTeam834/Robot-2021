/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.snapto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;

public class SnapTo0 extends CommandBase {
  /**
   * Creates a new SnapTo0.
   */
  boolean finished;
  double motorPower;

  public SnapTo0() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    //lMotor = -1; 
    //rMotor = 1;
    //Robot.driveTrain.setDrive(-0.5, 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Math.abs(Robot.navX.getYaw()) / 30 > 1) {
      motorPower = 1;
    }
    else {
      motorPower = Math.abs(Robot.navX.getYaw()) / 30;
    }

    //start turning robot in correct direction
    if (Robot.navX.getYaw() < 0) { //if robot to left of 0, turn right
      Robot.driveTrain.setDrive(-0.25 * motorPower, 0.25 * motorPower);
    } else if (Robot.navX.getYaw() >= 0) { //if robot to right of 0, turn left
      Robot.driveTrain.setDrive(0.25 * motorPower, -0.25 * motorPower);
    } 

    if(Math.abs(Robot.navX.getYaw()) <= 15) { // spin slower once close for percision
      //Robot.driveTrain.setDrive(lMotor*.25, rMotor*.25);
      if(Math.abs(Robot.navX.getYaw()) <= 3) {
        //now the that we are facing 0, we can gg ez stop spinning
        finished = true;
      }
     } 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driveTrain.setDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //finished;
  }
}
