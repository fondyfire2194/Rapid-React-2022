// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.PositionTiltToCamera;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.commands.Vision.UseVision;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

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

          new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720, true),

          new SetShootSpeedSource(shooter, shooter.cameraSource),

          new SelectSpeedAndTiltByDistance(shooter, tilt),

          new PositionTiltToCamera(tilt));

    }
  }
}
