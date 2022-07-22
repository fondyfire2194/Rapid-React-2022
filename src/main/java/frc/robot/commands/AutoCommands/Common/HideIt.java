// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Intakes.IntakeToShootPosition;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.RobotDrive.PositionToPickup;
import frc.robot.commands.RobotDrive.ResetEncoders;
import frc.robot.commands.RobotDrive.TurnToHideAngle;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HideIt extends SequentialCommandGroup {

  

  /** Creates a new HideIt. */
  public HideIt(RevDrivetrain drive, FondyFireTrajectory fftraj,

      Pose2d cargoPose, boolean reversed,

      IntakesSubsystem intake, CargoTransportSubsystem transport,

      RevTurretSubsystem turret, double shootAngle,

      RevShooterSubsystem shooter, double shootRPM) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double timeOut = 15;

    if (RobotBase.isSimulation())
      timeOut = 1;
    addCommands(

        sequence(

            new GetHideTurnAngle(drive, fftraj, cargoPose, reversed),

            new WaitCommand(.1)),

        sequence(

            new TurnToHideAngle(drive),

            new WaitCommand(.25),

            new ResetEncoders(drive),

            new WaitCommand(.1)),

        parallel(
          
            new PositionToPickup(drive, .25),

            new IntakeToShootPosition(intake, transport).withTimeout(timeOut),

            new PositionTurret(turret, shootAngle).withTimeout(timeOut)),

        new RunCargoOutShooter(shooter, intake, transport, shootRPM)

            .raceWith(new WaitCommand(3)));

  }
}
