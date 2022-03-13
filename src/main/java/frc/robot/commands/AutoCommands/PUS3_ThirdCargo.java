// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.Common.AcquireTarget;
import frc.robot.commands.AutoCommands.Common.LowerShoot;
import frc.robot.commands.AutoCommands.Common.PrepositionTiltAndTurret;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.SetRearIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.RobotDrive.PositionStraight;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.ResetGyro;
import frc.robot.commands.RobotDrive.TurnToAngleProfiled;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PUS3_ThirdCargo extends SequentialCommandGroup {
  /** Creates a new PUS3_ThirdCargo. */
  public PUS3_ThirdCargo(RevDrivetrain drive, IntakesSubsystem intake,

      CargoTransportSubsystem transport, RevTurretSubsystem turret,

      RevTiltSubsystem tilt, RevShooterSubsystem shooter, RawContoursV2 rcv2, LimeLight ll, Compressor compressor,
      double[] data) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double angleToThirdCargo = 22;
    final double distanceToThirdCargo = 11;
    final double pickUpSpeed = .5;
    final double tiltAngle = 5;
    final double turretAngle = 0;

    addCommands(new TurnToAngleProfiled(drive, angleToThirdCargo),

        new ParallelCommandGroup(new ResetEncoders(drive), new ResetGyro(drive),

            new PositionStraight(drive, distanceToThirdCargo, pickUpSpeed),

            new PrepositionTiltAndTurret(tilt, turret, tiltAngle, turretAngle))

                .deadlineWith(new SetRearIntakeActive(intake),

                    new ActiveIntakeArmLower(intake),

                    new RunActiveIntake(intake, transport)),

        new ParallelCommandGroup(new StopActiveIntake(intake), new ActiveIntakeArmRaise(intake),

            new AcquireTarget(ll, tilt, turret, rcv2, shooter, transport, intake, compressor),

            new ConditionalCommand(

                new RunShooter(shooter),

                new LowerShoot(turret, tilt, ll, shooter, transport, intake, compressor, data),

                () -> rcv2.isFound))

    );
  }
}
