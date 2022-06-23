// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.LeftHideOppCargo;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftHidePRG extends ParallelRaceGroup {
  /** Creates a new RightPRGShoot3. */
  public LeftHidePRG(IntakesSubsystem intake, RevDrivetrain drive,
      CargoTransportSubsystem transport, RevShooterSubsystem shooter,
      LimeLight ll, Compressor comp, double[] data) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new ParallelCommandGroup(
            new SetShootSpeedSource(shooter, shooter.fromPreset),
            new SetPresetRPM(shooter, 900),
         new RunShooter(shooter)),

        // new CalculateTargetDistance(ll, shooter),
        // new SelectSpeedAndTiltByDistance(shooter, tilt),

        new LeftHideOppCargo(intake, drive, transport, shooter, ll));
  }
}
