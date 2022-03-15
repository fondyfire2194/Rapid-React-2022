// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Pref;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Turret.PositionHoldTurret;
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
                        CargoTransportSubsystem transport, LimeLight ll, double[] data) {
                // Use addRequirements() here to declare subsystem dependencies.
                double tiltAngle = data[0];
                double turretAngle = data[1];
                double drivePickupPosition = data[2];

                addCommands(

                                new ParallelCommandGroup(

                                                new SetFrontIntakeActive(intake, false),
                                                new ActiveIntakeArmLower(intake),
                                                new ResetEncoders(drive),
                                                new ResetGyro(drive)),

                                new ParallelRaceGroup(

                                                new PositionStraight(drive, drivePickupPosition,
                                                                pickUpRate),

                                                new RunActiveIntake(intake, transport))
                                                
                                                                .deadlineWith(new ParallelCommandGroup(

                                                                                new PositionHoldTilt(tilt),
                                                                                new PositionHoldTurret(turret, ll))));

        }

}
