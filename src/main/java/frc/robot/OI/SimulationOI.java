// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Vision.LimeLight;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/** Add your docs here. */
public class SimulationOI {

  public SimulationOI(RevTurretSubsystem turret, RevTiltSubsystem tilt, RevDrivetrain drive,
      RevShooterSubsystem shooter, CargoTransportSubsystem transport, Compressor compressor,
      LimeLight limelight, IntakesSubsystem intake,
      ClimberSubsystem climber,
      FondyFireTrajectory traj) {

   // final Field2d m_field = new Field2d();

    ShuffleboardTab sim = Shuffleboard.getTab("Simulation");

    sim.add("Field", drive.m_field2d).withPosition(1, 0).withSize(5, 4).withWidget("Field");

    sim.addNumber("TiltAngle", () -> tilt.getAngle()).withPosition(6, 0)
        .withWidget("Number Bar").withProperties(Map.of("min", 0, "max", 16));
    sim.addNumber("TiltOut", () -> tilt.getMotorOut()).withPosition(7, 0)
        .withWidget("Number Bar").withProperties(Map.of("min", -1, "max", 1));

    sim.addNumber("TurretAngle", () -> turret.getAngle()).withPosition(6, 1)
        .withWidget("Number Bar").withProperties(Map.of("min", -40, "max", 40));
    sim.addNumber("TurretOut", () -> turret.getMotorOut()).withPosition(7, 1)
        .withWidget("Number Bar").withProperties(Map.of("min", -1, "max", 1));

    sim.addNumber("ShootRPM", () -> shooter.getRPM()).withPosition(8, 0)
        .withWidget("Number Bar").withProperties(Map.of("min", 0, "max", 5000));
    sim.addNumber("TopRPM", () -> shooter.getTopRPM()).withPosition(8, 1)
        .withWidget("Number Bar").withProperties(Map.of("min", 0, "max", 1000));

    sim.addNumber("LowRollRPM", () -> transport.getLowerRPM()).withPosition(9, 0)
        .withWidget("Number Bar").withProperties(Map.of("min", 0, "max", 1000));

    sim.addNumber("FrontIntakeOut", () -> intake.getFrontMotor()).withPosition(6, 2)
        .withWidget("Number Bar").withProperties(Map.of("min", -1, "max", 1));

    sim.addBoolean("FrontIntakeSol", () -> intake.getFrontArmLowered()).withPosition(6, 3);

    sim.addNumber("RearIntakeOut", () -> intake.getRearMotor()).withPosition(7, 2)
        .withWidget("Number Bar").withProperties(Map.of("min", -1, "max", 1));

    sim.addBoolean("RearIntakeSol", () -> intake.getRearArmLowered()).withPosition(7, 3);

    sim.addNumber("ClimberOut", () -> climber.getMotorOut()).withPosition(8, 2)
        .withWidget("Number Bar").withProperties(Map.of("min", -1, "max", 1));

  }

}