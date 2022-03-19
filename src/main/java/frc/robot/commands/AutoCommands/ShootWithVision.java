// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.Common.AcquireTarget;
import frc.robot.commands.AutoCommands.Common.PositionHoldTiltTurret;
import frc.robot.commands.AutoCommands.Common.SelectSpeedAndTiltByDistance;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetShootSpeedSource;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Vision.CalculateTargetDistance;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootWithVision extends SequentialCommandGroup {
  /** Creates a new ShootWithVision. */
  public ShootWithVision(LimeLight ll, RevTiltSubsystem tilt, RevTurretSubsystem turret, RawContoursV2 rcv2,
      RevShooterSubsystem shooter, CargoTransportSubsystem transport, IntakesSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(new AcquireTarget(ll, tilt, turret, rcv2),

        new SetShootSpeedSource(shooter, shooter.fromCamera),

        new CalculateTargetDistance(ll, rcv2, tilt, turret, shooter),

        new SelectSpeedAndTiltByDistance(shooter, tilt),

        new PositionTilt(tilt, tilt.cameraCalculatedTiltPosition),

        new ParallelRaceGroup(

            new ShootCargo(shooter, transport, intake),

            new RunShooter(shooter))

                .deadlineWith(new PositionHoldTiltTurret(tilt, turret,

                    ll)));

  }
}