// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous.irahAutons;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Import the robot
import frc.robot.Robot;
// Import robot container
import frc.robot.RobotContainer;
//import frc.robot.commands.MoveToPosition; (Figure out replacement)
// Parameters class
import frc.robot.Constants;
import frc.robot.commands.snapto.SnapToGoal;
import frc.robot.commands.autonomous.*;
import edu.wpi.first.wpilibj.util.Units;

//This is the auton for the Infinite Recharge at Home Auton Challenge; Part B

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Beeline extends SequentialCommandGroup {
  /** Creates a new PathB. */
  public Beeline() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    Starting position: D1
    *detect distance to nearest ball(either d5 or d6) if distance = 1.2m its red path if = 1.5m its blue path*
    */
    int angle1 = 0;
    int angle2 = 0;
    int angle3 = 0;
    int angle4 = 0;
    // tbh we don't know what the distances are in
    double distance1 = 0;
    double distance2 = 0;
    double distance3 = 0;
    double distance4 = 0;

    // Red Path
    if(RobotContainer.ultrasonicSensor.withinRange(Units.inchesToMeters(90), Units.inchesToMeters(10))) {
      // Turn 45 degrees to the left; Move 84.85 in Forwards
        angle1 = 45; distance1 = Units.inchesToMeters(84.85);

      // Turn 90 degrees to the right; Move 84.85 in Forwards
        angle2 = -45; distance2 = Units.inchesToMeters(84.85);

      // Turn 90 degrees to the left; Move 84.85 in Forwards
        angle3 = 45; distance3 = Units.inchesToMeters(84.85);

      // Turn 45 degrees to the right; Move 130 in Forwards
        angle4 = 0; distance4 = Units.inchesToMeters(130);

      // Makes lights red because red path
        Robot.lights = Constants.LAVA_RAINBOW;
    }
    // Blue Path
    else if(RobotContainer.ultrasonicSensor.withinRange(Units.inchesToMeters(120), Units.inchesToMeters(10))) {
      // Move Forwards 120 in
        angle1 = 0; distance1 = Units.inchesToMeters(120);

      // Turn 45 degrees to the left; Move 84.85 in Forwards
        angle2 = 45; distance2 = Units.inchesToMeters(84.85);

      // Turn 90 degrees to the right; Move 84.85 in Forwards
        angle3 = -45; distance3 = Units.inchesToMeters(84.85);

      // Turn 45 degrees to the left; Move 40 in Forwards
        angle4 = 0; distance4 = Units.inchesToMeters(40);

      // LED
        Robot.lights = Constants.SKY_BLUE;

    }
    else{
        Robot.lights = Constants.STROBE_RED;
    }

    addCommands(new SnapToGoal(angle1), new DriveAndIntake(distance1), new SnapToGoal(angle2), new DriveAndIntake(distance2), new SnapToGoal(angle3), new DriveAndIntake(distance3), new SnapToGoal(angle4), new DriveAndIntake(distance4));

  }
}