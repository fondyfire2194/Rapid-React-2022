// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.RawContoursV2;
import frc.robot.commands.AutoCommands.Common.AllRetPu;
import frc.robot.commands.AutoCommands.Common.GetNumberOfContourValues;
import frc.robot.commands.AutoCommands.Common.LowerShoot;
import frc.robot.commands.AutoCommands.Common.UpperShoot;

import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;

public class AllRetPuShoot extends SequentialCommandGroup {

  /** Creates a new LRetPuShoot. */
  public AllRetPuShoot(IntakesSubsystem intake, RevDrivetrain drive, RevTurretSubsystem turret, RevTiltSubsystem tilt,
      LimeLight ll, RevShooterSubsystem shooter, RawContoursV2 rcv2,
      CargoTransportSubsystem transport, Compressor compressor,double[] data) {
    

    addCommands(new AllRetPu(intake, drive, turret, tilt, data),

       new GetNumberOfContourValues(rcv2),

       
        new ConditionalCommand(

            new UpperShoot(turret, tilt, ll, shooter, transport, compressor, data),

            new LowerShoot(turret, tilt, ll, shooter, transport, compressor, data),

            () -> rcv2.isFound));

  }

}
