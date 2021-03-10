/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /*
     * CAN ID# Motor Name 1 Left Drive 1 --------------------------- 2 Left Drive 2
     * --------------------------- 3 Left Drive 3 --------------------------- 4
     * Right Drive 4 --------------------------- 5 Right Drive 5
     * --------------------------- 6 Right Drive 6 --------------------------- 7
     * Intake --------------------------- 8 Indexer --------------------------- 9
     * Pivot --------------------------- 10 Shooter --------------------------- 11
     * Climber --------------------------- 12 Wheel --------------------------- 13
     * PDP ---------------------------
     */


    public static final class ShooterConstants {
        public static final int SHOOTER_MOTOR_PORT = 10; // CAN ID# //SHOOTER MOTOR
        public static final int SHOOTER_PIVOT_MOTOR_PORT = 9; // CAN ID# //NEEDS TO BE ADDED STILL
        public static final boolean SHOOTER_INVERTED = true; // Going the wrong way?
        public static final boolean SHOOTER_PIVOT_INVERTED = true; // Going the wrong way?
        public static final double SHOOTER_PIVOT_SPEED = 0.1;

        public static final double S_BACKSPIN_RATIO = 4 / 7;
        public static final double S_WHEEL_SPEED = 3600 / 60; // rps
        public static final double S_WHEEL_VOLTAGE = 11.6; // Measure pls
        public static final double S_PROPORTIONAL_CONSTANT = 6;
        public static final double S_INTEGRAL_CONSTANT = 5;
        public static final double S_DERIVATIVE_CONSTANT = 4;

        public static final double AUTON_SHOOTER_SPEED = .7;
        public static final double ROBOT_HEIGHT = 31.75 / 12;
        public static final double WHEEL_CIRCUMFERENCE = 5 * Math.PI / 12;
        public static final double SHOOTER_MOUTH_WIDTH = 6.25 / 12; // Currently a placeholder best be in feet
        public static final int GIMBAL_LOCK_PORT1 = 6; // Encoder DIO Port# Get from SRX?
        public static final int GIMBAL_LOCK_PORT2 = 9; // Encoder DIO Port# Get from SRX?
        public static final double GIMBAL_MULTIPLIER = 5; // Measure
        public static final int HOOD_GEAR_RATIO = 18 / 66;
        public static final int HOOD_LIMIT_SWITCH_PORT = 9;
        public static final double ANGLE_TOLERANCE = 2 * Math.PI / 120;
    }
    

    // Drivetrain Constants
    public static final class DrivetrainConstants
    {
        public static final int LEFT_DRIVE_MOTOR_1 = 1; // CAN ID #
        public static final int LEFT_DRIVE_MOTOR_2 = 2; // CAN ID#
        public static final int LEFT_DRIVE_MOTOR_3 = 3; // CAN ID#
        public static final int RIGHT_DRIVE_MOTOR_1 = 4; // CAN ID#
        public static final int RIGHT_DRIVE_MOTOR_2 = 5; // CAN ID#
        public static final int RIGHT_DRIVE_MOTOR_3 = 6; // CAN ID#
        public static final boolean LEFT_DRIVE_INVERTED = true; // Going the wrong way?
        public static final boolean RIGHT_DRIVE_INVERTED = false; // Going the wrong way?
        public static final double DRIVE_CONVERSION_FACTOR = 0;
        public static final double DRIVE_ENCODER_MULTIPLIER = 0.00086340382; // multiply this by distance to get required
                                                                              // encoder change
    }

    // Intake Constants
    public static final class IntakeConstants
    {
        public static final int INTAKE_MOTOR_PORT = 7; // CAN ID#
        public static final boolean INTAKE_INVERTED = false; // Going the wrong way?
    }

    //Conveyor Constants
    public static final class ConveyorConstants
    {
        public static final int CONVEYOR_MOTOR_PORT = 8; // CAN ID#
        public static final boolean CONVEYOR_INVERTED = true; // Going the wrong way?
        public static final int BALL_SENSOR_PORT = 0; // DIO Port# //Bottom Sensor
        public static final int EMPTY_SENSOR_PORT = 5; // DIO Port# //Top Sensor
        public static final double AUTON_CONVEYOR_SPEED = .5;
        
    }

    // Ultrasonic Constants
    public static final class UltrasonicConstants
    {
        public static final int US_PING = 1; // DIO Port# // Ping(outgoing)
        public static final int US_ECHO = 2; // DIO Port$ // Echo(incoming)
    }


    //Auton Constants
    public static final class AutonConstants
    {
        public static final double ksVolts = .185;
        public static final double kvVoltSecondsPerMeter = 2.11;
        public static final double kaVoltSecondsSquaredPerMeter = .571;
        public static final double kPDriveVel = 1.82;
        public static final double kTrackwidthMeters = .64557;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(1);
        public static final double kMaxAccelerationMetersPerSecondSquared = Units.feetToMeters(1);
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
   
    // Vision Constants - Auton
    public static final class VisionAutonConstants
    {
        public static final double TOLERANCE = 10;
        public static final double SPEED_INDEX = .001;
        public static final double BALL_AREA = 1; // Measure these
        public static final double BALL_DISTANCE = 1;
        public static final double GOAL_HEIGHT = 110;
        public static final double GOAL_DISTANCE = 144; // 12 feet From release
    }

    // Command IDs for Driver Input
    // First Digit = Main Subsystem Number
    // 1 = Drivetrain
    // 2 = Conveyor
    // 3 = BallIntake
    // 4 = EVS
    // Second Digit = Command ID
    public static final class DriverInputConstants
    {
        public static final int DRIVE_MAX_SPEED_ID = 12;
        public static final int DRIVE_NORMAL_ID = 11;
        public static final int DRIVE_SLOW_SPEED_ID = 13;
        public static final int RUN_INTAKE_ID = 31;
        public static final int RUN_INTAKE_BACKWARDS_ID = 32;
        public static final int STOP_INTAKE_ID = 33;
        public static final int TOGGLE_VISION_ID = 41;
        public static final int AIM_AND_SHOOT_ID = 42;
    }
    

    /*
     * [Name Low-Speed High Speed]
     *
     */

    // public static String[][] driverInputArray = new String{{"Christian Piper",
    // "0.25", "0.75"}, {"Standard", "0",}};

     // LED Values (for style)
     public static final class LEDConstants
     {
        public static final double LAVA_RAINBOW = -.87;
        public static final double STROBE_RED = -.11;
        public static final double PARTY = -.43;
        public static final double PINK = .57;
        public static final double GLITTER_RAINBOW = -.89;
        public static final double OCEAN = -.95;
        public static final double WHITE_HB = .25;
        public static final double BLUE_VIOLET = .89;
        public static final double SKY_BLUE = .83;
     }
 
}
