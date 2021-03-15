
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.drive.*;
import frc.robot.commands.autonomous.irahAutons.*;
import frc.robot.commands.autonomous.AimAndShoot;
//import frc.robot.commands.autonomous.AimAndShoot;
import frc.robot.commands.autonomous.EmptyShooterNoVision;
import frc.robot.commands.autonomous.ShooterToSpeed;
import frc.robot.commands.autonomous.autons.TestAuto;
import frc.robot.commands.Conveyor.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.hood.*;
import frc.robot.commands.snapto.*;
import frc.robot.commands.vision.ToggleVision;
import frc.robot.subsystems.BallIntake;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.GimbalLock;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.EVSNetworkTables;
import frc.robot.subsystems.UltrasonicSensor;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new
  // ExampleCommand(m_exampleSubsystem);
  //Subsystems
  
  private final DriveNormal driveNormal = new DriveNormal();
  private final DriveSlowSpeed driveSlowSpeed = new DriveSlowSpeed();
  private final DriveMaxSpeed driveMaxSpeed = new DriveMaxSpeed();
  private final DriveInverted driveInverted = new DriveInverted();

  private final RunIntake runIntake = new RunIntake();
  private final RunIntakeBackwards runIntakeBackwards = new RunIntakeBackwards();
  private final StopIntake stopIntake = new StopIntake();

  private final RunShooter runShooter = new RunShooter();
  private final StopShooter stopShooter = new StopShooter();

  private final RunPivotUp runPivotUp = new RunPivotUp();
  private final RunPivotDown runPivotDown = new RunPivotDown();

  private final RunConveyor runConveyor = new RunConveyor();
  private final RunConveyorSensor runConveyorSensor = new RunConveyorSensor();
  private final RunConveyorBackward runConveyorBackward = new RunConveyorBackward();
  private final StopConveyor stopConveyor = new StopConveyor();

  private final ToggleVision toggleVision = new ToggleVision();
  private final AimAndShoot aimAndShoot = new AimAndShoot();
  private final ShooterToSpeed shooterToSpeed = new ShooterToSpeed();
  
  //private final SpinCP spinCP = new SpinCP();
  //private final SetCPColor setCPColor = new SetCPColor();
  
  private final DriveBackwardsDistance driveBackwardsDistance = new DriveBackwardsDistance(12); 
  private final EmptyShooterNoVision emptyShooterNoVision = new EmptyShooterNoVision();
  //private final ResetYaw resetYaw = new ResetYaw();
  private final SnapTo0 snapTo0 = new SnapTo0();
  private final SnapTo180 snapTo180 = new SnapTo180();
  private final DriveForwardDistance driveForwardDistance = new DriveForwardDistance(.25, 69);

  public final static TestAuto testAuto = new TestAuto(Robot.driveTrain);
  public final static SlalomIBarelyKnowEm slalom = new SlalomIBarelyKnowEm(Robot.driveTrain);
  public final static Beeline pathB = new Beeline();
  public final static PlanA pathA = new PlanA();
  public final static DoABarrelRoll barrelRoll = new DoABarrelRoll(Robot.driveTrain);
  public final static InelasticCollision bounce = new InelasticCollision(Robot.driveTrain);
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);
  private final XboxController xbox = new XboxController(2);
  private final Joystick launchpad = new Joystick(3);
  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  //Joystick Buttons
  private final JoystickButton
  // Left Joystick
  lJoystick1 = new JoystickButton(leftJoystick, 1), lJoystick2 = new JoystickButton(leftJoystick, 2),
      lJoystick3 = new JoystickButton(leftJoystick, 3), lJoystick4 = new JoystickButton(leftJoystick, 4),
      lJoystick5 = new JoystickButton(leftJoystick, 5), lJoystick6 = new JoystickButton(leftJoystick, 6),
      lJoystick7 = new JoystickButton(leftJoystick, 7), lJoystick8 = new JoystickButton(leftJoystick, 8),
      lJoystick9 = new JoystickButton(leftJoystick, 9), lJoystick10 = new JoystickButton(leftJoystick, 10),
      lJoystick11 = new JoystickButton(leftJoystick, 11),

      // Right Joystick
      rJoystick1 = new JoystickButton(rightJoystick, 1), rJoystick2 = new JoystickButton(rightJoystick, 2),
      rJoystick3 = new JoystickButton(rightJoystick, 3), rJoystick4 = new JoystickButton(rightJoystick, 4),
      rJoystick5 = new JoystickButton(rightJoystick, 5), rJoystick6 = new JoystickButton(rightJoystick, 6),
      rJoystick7 = new JoystickButton(rightJoystick, 7), rJoystick8 = new JoystickButton(rightJoystick, 8),
      rJoystick9 = new JoystickButton(rightJoystick, 9), rJoystick10 = new JoystickButton(rightJoystick, 10),
      rJoystick11 = new JoystickButton(rightJoystick, 11),

      // Arcade Buttons
      BGTL = new JoystickButton(launchpad, 7), BGTM = new JoystickButton(launchpad, 2),
      BGTR = new JoystickButton(launchpad, 4), BGML = new JoystickButton(launchpad, 1),
      BGMM = new JoystickButton(launchpad, 6), BGMR = new JoystickButton(launchpad, 3),
      BGBL = new JoystickButton(launchpad, 10), BGBM = new JoystickButton(launchpad, 9),
      BGBR = new JoystickButton(launchpad, 8);

  //Xbox Buttons
  private final JoystickButton xboxStart = new JoystickButton(xbox, 8), xboxBack = new JoystickButton(xbox, 7),
      xboxB = new JoystickButton(xbox, 2), xboxA = new JoystickButton(xbox, 1), xboxY = new JoystickButton(xbox, 4),
      xboxX = new JoystickButton(xbox, 3), xboxLB = new JoystickButton(xbox, 5), xboxRB = new JoystickButton(xbox, 6),
      xboxLJB = new JoystickButton(xbox, 9);

  // Triggers

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    autonChooser.addOption("Slalom", slalom);
    autonChooser.addOption("Path B", pathB);
    autonChooser.addOption("Path A", pathA);
    autonChooser.addOption("Barrel Race", barrelRoll);
    autonChooser.addOption("Bounce Path", bounce);
    autonChooser.addOption("Test Path", testAuto);
    SmartDashboard.putData("Auto Chooser", autonChooser);
  }


  public ArrayList<Object> getCommands() {

    ArrayList<Object> t = new ArrayList<Object>();
    //t.add(driveNormal);
    t.add(driveMaxSpeed);
    t.add(driveSlowSpeed);
    t.add(runIntake);
    t.add(runIntakeBackwards);
    t.add(stopIntake);
    t.add(toggleVision);
    //t.add(aimAndShoot);

    return t;

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*Once all commands are inlined, organize them by subsystem. Then, use the search function (second icon on the left) and search for uses 
    of the command. Ex. search "driveNormal" and make all places where that is used into an inlined command (oter than robot.java. robot.java is weird)
    This might be the only place some of these are used and thats ok. Once that's done we can delete the command files.
    */ 

    // Drive

    //drive slow
    lJoystick1.whenPressed(() -> Robot.driveTrain.setDriveWithMultiplier(0.25));
    
    //drive normal
    rJoystick1.whenPressed(() -> Robot.driveTrain.setDriveWithMultiplier(.5));

    //drive fast/max speed
    rJoystick2.whenPressed(() -> Robot.driveTrain.setDriveWithMultiplier(1));

    //drive inverted
    //lJoystick2.whenPressed(Robot.driveInverted); 

    // Shooter
    
    //Aiming and shooting autonomous routine
    //xboxA.whenPressed(aimAndShoot);

    //Start the shooter
    BGTL.toggleWhenPressed(runShooter);

    //Hood
    // BGTR.whileHeld(runPivotUp);
    // BGMR.whileHeld(runPivotDown);

    // Coveyor

    //Run conveyor
    xboxB.toggleWhenPressed(runConveyorSensor);

    //conveyor backwards
    BGMR.whenPressed(() -> Robot.conveyor.start(-.75));

    //stop conveyor
    BGMM.whenPressed(new InstantCommand(Robot.conveyor::stop, Robot.conveyor));

    //Intake
    
     //start intake
     xboxRB.whileHeld(() -> Robot.ballIntake.start(1.0));

    //reverse intake
    xboxLB.whenHeld(new InstantCommand(Robot.ballIntake::startBackwards, Robot.ballIntake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}