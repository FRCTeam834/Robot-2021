
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
  CANSparkMax leftDrive1 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, CANSparkMax.MotorType.kBrushless);
  CANSparkMax leftDrive2 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, CANSparkMax.MotorType.kBrushless);
  CANSparkMax leftDrive3 = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_3, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightDrive1 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightDrive2 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, CANSparkMax.MotorType.kBrushless);
  CANSparkMax rightDrive3 = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_3, CANSparkMax.MotorType.kBrushless);

  SpeedControllerGroup leftDriveGroup = new SpeedControllerGroup(leftDrive1, leftDrive2, leftDrive3);
  SpeedControllerGroup rightDriveGroup = new SpeedControllerGroup(rightDrive1, rightDrive2, rightDrive3);

  DifferentialDrive dDrive = new DifferentialDrive(leftDriveGroup, rightDriveGroup);

  Joystick l = new Joystick(0);
  Joystick r = new Joystick(1);
  Rotation2d robotAngle = RobotContainer.navX.getRotation2d();
  DifferentialDriveOdometry dDriveOdometry = new DifferentialDriveOdometry(robotAngle);

  public DriveTrain() {

    leftDriveGroup.setInverted(Constants.LEFT_DRIVE_INVERTED);
    rightDriveGroup.setInverted(Constants.RIGHT_DRIVE_INVERTED);
    resetOdometry(new Pose2d());
    

  }

  public void resetEncoderPosition() {

    leftDrive1.getEncoder().setPosition(0);
    leftDrive2.getEncoder().setPosition(0);
    leftDrive3.getEncoder().setPosition(0);
    rightDrive1.getEncoder().setPosition(0);
    rightDrive2.getEncoder().setPosition(0);
    rightDrive3.getEncoder().setPosition(0);
    leftDrive1.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
    leftDrive2.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
    leftDrive3.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
    rightDrive1.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
    rightDrive2.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);
    rightDrive3.getEncoder().setPositionConversionFactor(Constants.DRIVE_CONVERSION_FACTOR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("Turnt: " + Rotation2d.fromDegrees(RobotContainer.navX.getYaw()));
    // System.out.println("Pose: " + dDriveOdometry.getPoseMeters());
    dDriveOdometry.update(robotAngle, leftDrive1.getEncoder().getPosition(), rightDrive1.getEncoder().getPosition());
    SmartDashboard.putString("Pose", dDriveOdometry.getPoseMeters().toString());
    SmartDashboard.putString("Angle", RobotContainer.navX.getRotation2d().toString());

  }

  public void resetOdometry(Pose2d pose2d) {
    resetEncoderPosition();
    dDriveOdometry.resetPosition(pose2d, RobotContainer.navX.getRotation2d());
  }

  public void leftDrive(double speed) {

    leftDriveGroup.set(speed);

  }

  public void rightDrive(double speed) {

    rightDriveGroup.set(speed);

  }

  public void setDrive(double lSpeed, double rSpeed) {

    leftDriveGroup.set(lSpeed);
    rightDriveGroup.set(rSpeed);

  }

  public CANEncoder getEncoders() {

    return leftDrive1.getEncoder();

  }

  public double getLeftEncoderValue() {

    double returnValue = (leftDrive1.getEncoder().getPosition() + leftDrive2.getEncoder().getPosition()
        + leftDrive3.getEncoder().getPosition()) / 3;
    return returnValue;

  }

  public double getRightEncoderValue() {

    double returnValue = (rightDrive1.getEncoder().getPosition() + rightDrive2.getEncoder().getPosition()
        + rightDrive3.getEncoder().getPosition()) / 3;
    return returnValue;

  }

  public void stop() {

    leftDriveGroup.set(0);
    rightDriveGroup.set(0);

  }

  public void setDriveWithMultiplier(double multiplier) {

    if (Robot.driveInverted == false) {
      setDrive(l.getY() * multiplier, r.getY() * multiplier);
    } else {
      setDrive((-r.getY() * multiplier), (-l.getY() * multiplier));
    }

  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDrive1.getEncoder().getVelocity(),
        rightDrive1.getEncoder().getVelocity());

  }

  public Pose2d getPose() {

    return dDriveOdometry.getPoseMeters();

  }

  public Joystick getJoystickL() {

    return l;

  }

  public Joystick getJoystickR() {

    return r;

  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    System.out.println(leftVolts);
    System.out.println(rightVolts);
    leftDriveGroup.setVoltage(-leftVolts * .05);
    rightDriveGroup.setVoltage(-rightVolts * .05);
    dDrive.feed();

  }

  public Command commandForTrajectory(Trajectory trajectory, Boolean initPose) {
    resetEncoderPosition();
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, RobotContainer.driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics, RobotContainer.driveTrain::getWheelSpeeds, new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        RobotContainer.driveTrain::tankDriveVolts, RobotContainer.driveTrain);
    // Run path following command, then stop at the end.
    if (initPose) {
      var reset = new InstantCommand(() -> RobotContainer.driveTrain.resetOdometry(trajectory.getInitialPose()));
      return reset.andThen(ramseteCommand.andThen(() -> RobotContainer.driveTrain.tankDriveVolts(0, 0)));
    } else {
      return ramseteCommand.andThen(() -> this.tankDriveVolts(0, 0));
    }
  }
}