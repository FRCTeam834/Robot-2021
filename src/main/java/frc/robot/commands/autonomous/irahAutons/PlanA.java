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
import frc.robot.Constants.LEDConstants;
import frc.robot.commands.snapto.SnapToGoal;
import frc.robot.commands.autonomous.*;
import edu.wpi.first.wpilibj.util.Units;

//This is the auton for the Infinite Recharge at Home Auton Challenge; Part B

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlanA extends SequentialCommandGroup {
  /** Creates a new PathB. */
  public PlanA() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    /*
    Starting position: D1
    *detect distance to nearest ball(either d5 or d6) if distance = 1.2m its red path if = 1.5m its blue path*
    */
    double angle1 = 0;
    double angle2 = 0;
    double angle3 = 0;
    double angle4 = 0;
    // tbh we don't know what the distances are in
    double distance1 = 0;
    double distance2 = 0;
    double distance3 = 0;
    double distance4 = 0;

    // Red Path
    if(Robot.ultrasonic.withinRange(Units.inchesToMeters(60), Units.inchesToMeters(10))) {
      // Move 60 in Forwards
        angle1 = 0; distance1 = Units.inchesToMeters(60);

      // Turn 26.565 degrees to the right; Move 67.08 in Forwards
        angle2 = -26.565; distance2 = Units.inchesToMeters(67.08);

      // Turn 98.13 degrees to the left; Move 94.868 in Forwards
        angle3 = 71.565; distance3 = Units.inchesToMeters(94.868);

      // Turn 71.565 degrees to the right; Move 160 in Forwards
        angle4 = 0; distance4 = Units.inchesToMeters(160);

      // Makes lights red because red path
        Robot.lights = LEDConstants.LAVA_RAINBOW;
    }
    // Blue Path
    else if(Robot.ultrasonic.withinRange(Units.inchesToMeters(240), Units.inchesToMeters(10))) {
      // Turn  degrees to the right; Move Forwards 161.555 in
        angle1 = -21.801; distance1 = Units.inchesToMeters(161.555);

      // Turn 93.365 degrees to the left; Move 94.868 in Forwards
        angle2 = 71.565; distance2 = Units.inchesToMeters(94.868);

      // Turn 98.13 degrees to the right; Move 67.08 in Forwards
        angle3 = -26.565; distance3 = Units.inchesToMeters(67.08);

      // Turn 26.565 degrees to the left; Move 70 in Forwards
        angle4 = 0; distance4 = Units.inchesToMeters(70);

      // LED
        Robot.lights = LEDConstants.SKY_BLUE;

    }
    else{
        //Robot.lights = LEDConstants.STROBE_RED;
    }

    addCommands(new SnapToGoal(angle1), new DriveAndIntake(distance1), new SnapToGoal(angle2), new DriveAndIntake(distance2), new SnapToGoal(angle3), new DriveAndIntake(distance3), new SnapToGoal(angle4), new DriveAndIntake(distance4));

  }
}