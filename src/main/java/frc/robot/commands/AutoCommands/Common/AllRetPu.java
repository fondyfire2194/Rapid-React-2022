// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Pref;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class AllRetPu extends SequentialCommandGroup {

  // double intakeSpeed = Pref.getPref("IntakeSpeed");
  double pickUpRate = Pref.getPref("dRPur");

  /** Creates a new LRetPuShoot. */
  public AllRetPu(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      CargoTransportSubsystem transport, double[] data) {
    // Use addRequirements() here to declare subsystem dependencies.
    double tiltAngle = data[0];
    double turretAngle = data[1];
    double drivePickupPosition = data[2];
    double driveForwardPosition = data[3];

    addCommands(

        new ParallelCommandGroup(new ResetEncoders(drive), new ResetGyro(drive)),

            new ParallelCommandGroup(new PrepositionTiltAndTurret(tilt, turret, tiltAngle, turretAngle),
            
            new PositionStraight(drive, drivePickupPosition, pickUpRate))

                .deadlineWith(new SetRearIntakeActive(intake),

                    new RunActiveIntake(intake, transport)),

        new StopActiveIntake(intake));

  }

}
