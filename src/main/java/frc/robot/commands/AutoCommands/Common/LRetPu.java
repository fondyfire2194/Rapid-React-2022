// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;


import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldMap;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntakeMotor;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.RobotDrive.PickupMove;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class LRetPu extends SequentialCommandGroup {

  double tiltAngle = FieldMap.leftTarmacTiltAngle;
  double turretAngle = FieldMap.leftTarmacTurretAngle;
  double driveToPosition = FieldMap.leftTarmacDriveToPosition;
  double pickUpRate = FieldMap.drivePickupRate;
  double mps = FieldMap.leftStartMPS;
  double intakeSpeed = FieldMap.intakeSpeed;

  /** Creates a new LRetPuShoot. */
  public LRetPu(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt) {
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(

        new ParallelCommandGroup(new ResetEncoders(drive), new ResetGyro(drive),

            new PickupMove(drive, driveToPosition, pickUpRate),

            new PrepositionTiltAndTurret(tilt, turret, tiltAngle, turretAngle))

                .deadlineWith(new SetRearIntakeActive(intake),

                    new ActiveIntakeArmLower(intake),

                    new RunActiveIntakeMotor(intake, intakeSpeed)),

        new ParallelCommandGroup(new StopActiveIntake(intake), new ActiveIntakeArmRaise(intake)));

  }

}
