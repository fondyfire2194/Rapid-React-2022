// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Tilt.PositionTiltToPreset;
import frc.robot.commands.Vision.SetUpLimelightForTarget;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetupPresetShootLocation extends SequentialCommandGroup {
  /** Creates a new SetupForShootLocation. */
  public SetupPresetShootLocation(RevShooterSubsystem shooter, RevTiltSubsystem tilt, RevTurretSubsystem turret,
      LimeLight ll, int shootLocation) {
    {
      // Add your commands in the addCommands() call, e.g.
      // addCommands(new FooCommand(), new BarCommand());
      addCommands(

          new SetUpLimelightForTarget(ll, PipelinesConstants.noZoom960720, true),

          new SetShootSpeedSource(shooter, shooter.presetSource),

          new SetPresetShootPositionSpeedTilt(shooter, tilt, ll, shootLocation),

          new PositionTiltToPreset(tilt));

    }
  }
}
