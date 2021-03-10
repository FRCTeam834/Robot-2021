// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands.autonomous.irahAutons;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.subsystems.DriveTrain;

//This is the auton for the Bounce Path auton challenge

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InelasticCollision extends SequentialCommandGroup {
  /** Creates a new Bounce. */
  public InelasticCollision(DriveTrain driveTrain) {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(AutonConstants.ksVolts,
    AutonConstants.kvVoltSecondsPerMeter, AutonConstants.kaVoltSecondsSquaredPerMeter), AutonConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(AutonConstants.kMaxSpeedMetersPerSecond,
    AutonConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutonConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

            Trajectory SlalomTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(Units.feetToMeters(5.96), Units.feetToMeters(2.75),  Rotation2d.fromDegrees(44.578)),
List.of(
   new Translation2d(Units.feetToMeters(8.623), Units.feetToMeters(6.132)),
   new Translation2d(Units.feetToMeters(15.984), Units.feetToMeters(6.767)),
   new Translation2d(Units.feetToMeters(21.344), Units.feetToMeters(6.175)),
   new Translation2d(Units.feetToMeters(25.156), Units.feetToMeters(3.473)),
   new Translation2d(Units.feetToMeters(26.557), Units.feetToMeters(5.48)),
   new Translation2d(Units.feetToMeters(23.852), Units.feetToMeters(5.995)),
   new Translation2d(Units.feetToMeters(20.139), Units.feetToMeters(3.422)),
   new Translation2d(Units.feetToMeters(11.508), Units.feetToMeters(3.139))
),
new Pose2d(Units.feetToMeters(4.205), Units.feetToMeters(7.487),  Rotation2d.fromDegrees(161.015)),
config);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      driveTrain.commandForTrajectory(SlalomTrajectory, false)
    );
  }
}
