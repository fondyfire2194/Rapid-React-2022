// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMap;
import frc.robot.Vision.GetTarget;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.Common.AimAndLockTarget;
import frc.robot.commands.AutoCommands.Common.LRetPu;
import frc.robot.commands.CargoTransport.RunRollers;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.SetShootSpeed;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LRetPuShoot extends SequentialCommandGroup {
  double tiltAngle = FieldMap.leftTarmacTiltAngle;
  double turretAngle = FieldMap.leftTarmacTurretAngle;
  double driveToPosition = FieldMap.leftTarmacDriveToPosition;
  double pickUpRate = FieldMap.drivePickupRate;
  double mps = FieldMap.leftStartMPS;
  double intakeSpeed = FieldMap.intakeSpeed;

  /** Creates a new LRetPuShoot. */
  public LRetPuShoot(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight ll, RevShooterSubsystem shooter, RawContoursV2 rcv2, GetTarget target,
      CargoTransportSubsystem transport, Compressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(new LRetPu(intake, drive, turret, tilt),

        new AimAndLockTarget(ll, turret, tilt, rcv2, target),

        new SetShootSpeed(shooter, mps),

        new RunShooter(shooter), new RunRollers(transport),

        new ShootCargo(shooter, tilt, turret, ll, transport, drive, compressor, 1.)

            .deadlineWith(new PositionHoldTilt(tilt, ll), new PositionHoldTurret(turret, ll)));

  }

}
