/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.createdclasses.Goal;
import frc.robot.subsystems.GimbalLock;

public class LookAtGoalY extends CommandBase {
  /**
   * Creates a new LookAtGoalY.
   */

  private GimbalLock g = Robot.gimbalLock;
  private double angleRequirement = 0;
  private boolean isFinished = false;
  private Goal goal;
  private boolean over = false;

  public LookAtGoalY() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.gimbalLock);
    addRequirements(Robot.EVSNetworkTables);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    g.stop();

    isFinished = false;

    try {

      if (Robot.EVSNetworkTables.getGoalArray().get(0).size() != 0) {

        goal = new Goal(Robot.EVSNetworkTables.getGoalArray().get(0));

      } else {

        System.out.println("No Goal Found");
        isFinished = true;

      }

    } catch (Exception e) {

    }

    double distance = goal.getDistance() * Math.sin(g.getEncoder()) / 12; //The 5 is a placeholder for shooter angle
    double v = ShooterConstants.S_WHEEL_SPEED * ShooterConstants.WHEEL_CIRCUMFERENCE;
    double phi = g.getEncoder();
    double a = Math.pow(distance - ShooterConstants.SHOOTER_MOUTH_WIDTH * Math.sin(phi), 2) / (v);
    double b = distance - ShooterConstants.SHOOTER_MOUTH_WIDTH * Math.sin(phi);
    double c = 98.25 / 12 - ShooterConstants.SHOOTER_MOUTH_WIDTH * Math.cos(phi);
    angleRequirement = 2 * (Math.atan(b - Math.sqrt(-Math.pow(a, 2) + Math.pow(b, 2) + Math.pow(c, 2) / (a + c))));
    if (angleRequirement < 0)
      angleRequirement = 2 * (Math.atan(b + Math.sqrt(-Math.pow(a, 2) + Math.pow(b, 2) + Math.pow(c, 2) / (a + c))));
    angleRequirement = angleRequirement % (2 * Math.PI);
    angleRequirement -= Math.PI/2;
    if(angleRequirement < g.getEncoder()) over = false;
    if(angleRequirement > g.getEncoder()) over = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (g.getEncoder() < angleRequirement + ShooterConstants.ANGLE_TOLERANCE ) {
      g.tiltDown(ShooterConstants.GIMBAL_MULTIPLIER * (angleRequirement - g.getEncoder()));
    } else if (g.getEncoder() > angleRequirement - ShooterConstants.ANGLE_TOLERANCE) {
      g.tiltUp(ShooterConstants.GIMBAL_MULTIPLIER * (angleRequirement - g.getEncoder()));
    } else if (g.getEncoder() > angleRequirement - ShooterConstants.ANGLE_TOLERANCE || g.getEncoder() < angleRequirement + ShooterConstants.ANGLE_TOLERANCE) {
      g.stop();}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }

}
