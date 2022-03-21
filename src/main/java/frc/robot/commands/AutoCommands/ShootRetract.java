// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetPresetRPM;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootRetract extends SequentialCommandGroup {
  /** Creates a new ShootRetract. */

  public ShootRetract(RevDrivetrain drive, RevTiltSubsystem tilt, RevTurretSubsystem turret, LimeLight ll,
      CargoTransportSubsystem transport, RevShooterSubsystem shooter, IntakesSubsystem intake, Compressor comp,
      double[] data) {
    // Add your commands in the addCommands() call, e.g.

    // addCommands(new FooCommand(), new BarCommand());

    double pickUpRate = drive.pickUpRate;
    double positionRate = drive.positionRate;

    double driveRetractPosition = data[0];
    double shootPosition = data[1];
    double upperTiltAngle = data[2];
    double upperTurretAngle = data[3];
    double upperRPM = data[4];

    addCommands(

        new ParallelCommandGroup(

            new PositionTilt(tilt, upperTiltAngle),

            new SetShootSpeedSource(shooter, shooter.fromPreset),

            new SetPresetRPM(shooter, upperRPM)),

        new ParallelRaceGroup(

            new SequentialCommandGroup(new ShootCargo(shooter, transport, intake),

                new ShootCargo(shooter, transport, intake)),

            new RunShooter(shooter))

                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,

                    ll)),

        new PositionStraight(drive, driveRetractPosition, positionRate)
            .deadlineWith(new PositionHoldTiltTurret(tilt, turret,

                ll)));
  }
}
