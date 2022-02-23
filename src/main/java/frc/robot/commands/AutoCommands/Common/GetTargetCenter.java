// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.Shooter.WaitTiltTurretLocked;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTiltToVision;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class GetTargetCenter extends SequentialCommandGroup {
  /** Creates a new AimAndShoot. */

  public GetTargetCenter(LimeLight ll, RevTurretSubsystem turret, RevTiltSubsystem tilt,

      RawContoursV2 rcv2,  double[] data) {

    double tiltTargetPosition = data[0];

    double turretTargetPosition = data[1];

    addCommands(new ParallelCommandGroup(new PositionTiltToVision(tilt, ll, tiltTargetPosition),

        new PositionTurretToVision(turret, ll, turretTargetPosition)),

        new WaitTiltTurretLocked(tilt, turret, ll),

      //  new GetNumberOfContourValues(rcv2)

           // .deadlineWith(new PositionHoldTilt(tilt, ll),

           //     new PositionHoldTurret(turret, ll)),

        new CalculateTarget(rcv2).deadlineWith(new PositionHoldTilt(tilt, ll),

            new PositionHoldTurret(turret, ll)));

  }
}
