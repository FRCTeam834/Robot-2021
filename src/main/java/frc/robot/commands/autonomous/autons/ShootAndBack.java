/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomous.autons;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveBackwardsDistance;
import frc.robot.commands.ResetYaw;
import frc.robot.commands.SnapTo0;
import frc.robot.commands.autonomous.AimAndShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndBack extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndBack.
   */
  public ShootAndBack() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new AimAndShoot(), new SnapTo0(), new DriveBackwardsDistance(42));
  }
}
