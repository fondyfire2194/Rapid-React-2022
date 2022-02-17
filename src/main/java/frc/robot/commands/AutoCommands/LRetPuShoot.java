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
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntakeMotor;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LRetPuShoot extends SequentialCommandGroup {
  double tiltAngle = FieldMap.leftTarmacTiltAngle;
  double turretAngle = FieldMap.leftTarmacTurretAngle;
  double drivePosition = FieldMap.leftTarmacDrivePosition;
  double pickUpRate = FieldMap.drivePickupRate;

  /** Creates a new LRetPuShoot. */
  public LRetPuShoot(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight ll, RevShooterSubsystem shooter, RawContoursV2 rcv2, GetTarget target,
      CargoTransportSubsystem transport, Compressor compressor) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
        new ParallelCommandGroup(new ResetEncoders(drive), new ResetGyro(drive),
            new PickupMove(drive, drivePosition, pickUpRate),
            new PrepositionTiltAndTurret(tilt, turret, tiltAngle, turretAngle))
                .deadlineWith(new SetRearIntakeActive(intake),
                    new ActiveIntakeArmLower(intake),
                    new RunActiveIntakeMotor(intake, 0.75)),

        new ParallelCommandGroup(new StopActiveIntake(intake), new ActiveIntakeArmRaise(intake)),

        new AimAndLockTarget(ll, turret, tilt, rcv2, target),

        new ShootCargo(shooter, tilt, turret, ll, transport, drive, compressor, 1.));

  }

}
