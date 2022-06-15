// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.TiltMoveToReverseLimit;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetUpCameraShoot extends SequentialCommandGroup {
  /** Creates a new SetupForCameraShoot. */
  public SetUpCameraShoot(RevShooterSubsystem shooter, RevTiltSubsystem tilt,
      LimeLight ll) {
    {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(

          new SetShootSpeedSource(shooter, shooter.cameraSource),

          new UseVision(ll, true),

          new SelectSpeedAndTiltByDistance(shooter, tilt),

          new TiltMoveToReverseLimit(tilt),

          new PositionTilt(tilt, tilt.cameraCalculatedTiltPosition));

    }
  }
}
