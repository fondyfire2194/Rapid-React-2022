// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Hub;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.commands.CargoTransport.RunRollers;
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
public class AutoCenterShootMove extends ParallelRaceGroup {
  /** Creates a new AutoCenterShootMove. */
  public AutoCenterShootMove(RevShooterSubsystem shooter, RevDrivetrain drive, RevTurretSubsystem turret,
      RevTiltSubsystem tilt, LimeLight limelight, Compressor compressor, CargoTransportSubsystem transport,
      IntakesSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(

      new RunShooter(shooter),new RunRollers(transport),

        new AutoModeCenterPowerPort(shooter, turret, tilt, transport, drive, intake, limelight, compressor)

    );
  }
}
