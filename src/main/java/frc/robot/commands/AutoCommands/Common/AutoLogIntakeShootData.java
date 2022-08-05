// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;

public class AutoLogIntakeShootData extends CommandBase {
        /** Creates a new AutoLogData. */
        private IntakesSubsystem m_intake;
        private CargoTransportSubsystem m_transport;
        private RevShooterSubsystem m_shooter;
        private RevDrivetrain m_drive;
        private RevTiltSubsystem m_tilt;
        private double m_autoStartTime;

        NetworkTableEntry autotime = NetworkTableInstance.getDefault().getTable("autolog").getEntry("autotime");
        NetworkTableEntry cargoatshoot = NetworkTableInstance.getDefault().getTable("autolog").getEntry("cargatshoot");
        NetworkTableEntry is_shooting = NetworkTableInstance.getDefault().getTable("autolog").getEntry("isshooting");

        NetworkTableEntry rearIntakeActive = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("rearintakeactive");
        NetworkTableEntry frontIntakeMotor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("frontintakemotor");
        NetworkTableEntry frontIntakeSensor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("frontintakesensor");
        NetworkTableEntry frontIntakeArm = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("frontintakearm");
        NetworkTableEntry cargoatfront = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("cargoatfront");

        NetworkTableEntry rearIntakeMotor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("rearintake");
        NetworkTableEntry rearIntakeSensor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("rearintakesensor");
        NetworkTableEntry rearIntakeArm = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("rearintakearm");
        NetworkTableEntry cargoatrear = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("cargoatrear");

        NetworkTableEntry lowRollerMotor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("lowrollerrpm");
        NetworkTableEntry lowRollerMotorOut = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("lowrollerout");
        NetworkTableEntry lowRollerMotorAmps = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("lowrolleramps");
        NetworkTableEntry topRollerMotor = NetworkTableInstance.getDefault().getTable("autolog").getEntry("toproller");
        NetworkTableEntry leftshootermotor = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("leftshooterrpm");
        NetworkTableEntry autoTime = NetworkTableInstance.getDefault().getTable("autolog").getEntry("autotime");

        NetworkTableEntry robotPosition = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("robotposition");
        NetworkTableEntry calcDistance = NetworkTableInstance.getDefault().getTable("autolog").getEntry("calcdistance");
        NetworkTableEntry calcrpm = NetworkTableInstance.getDefault().getTable("autolog").getEntry("calcrpm");
        NetworkTableEntry calctiltangle = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("calctitlangle");

        NetworkTableEntry inAutoEnabled = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("inautoenabled");
        NetworkTableEntry inTeleopEnabled = NetworkTableInstance.getDefault().getTable("autolog")
                        .getEntry("inteleopenabled");

        public AutoLogIntakeShootData(IntakesSubsystem intake, CargoTransportSubsystem transport,
                        RevShooterSubsystem shooter,
                        RevDrivetrain drive, RevTiltSubsystem tilt) {
                // Use addRequirements() here to declare subsystem dependencies.
                m_intake = intake;
                m_transport = transport;
                m_shooter = shooter;
                m_drive = drive;
                m_tilt = tilt;
        }

        // Called when the command is initially scheduled.
        @Override
        public void initialize() {
                m_autoStartTime = Timer.getFPGATimestamp();

        }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {

                autoTime.setNumber(Timer.getFPGATimestamp() - m_autoStartTime);

                cargoatshoot.setBoolean(m_transport.getCargoAtShoot());
                is_shooting.setBoolean(m_shooter.isShooting);
                lowRollerMotor.setNumber(m_transport.getLowerRPM());
                lowRollerMotorOut.setNumber(m_transport.getLowerRoller());
                lowRollerMotorAmps.setNumber(m_transport.getLowerRollerMotorAmps());

                rearIntakeActive.setBoolean(!m_intake.useFrontIntake);

                cargoatfront.setBoolean(m_intake.getCargoAtFront());
                frontIntakeSensor.setNumber(m_intake.frontIntakeCargoDetect.getVoltage());
                frontIntakeMotor.setNumber(m_intake.getFrontMotor());
                frontIntakeArm.setBoolean(m_intake.getFrontArmLowered());

                cargoatrear.setBoolean(m_intake.getCargoAtRear());
                rearIntakeSensor.setNumber(m_intake.rearIntakeCargoDetect.getVoltage());
                rearIntakeMotor.setNumber(m_intake.getRearMotor());
                rearIntakeArm.setBoolean(m_intake.getRearArmLowered());

                topRollerMotor.setNumber(m_shooter.getTopRPM());
                leftshootermotor.setNumber(m_shooter.getRPM());

                robotPosition.setNumber(m_drive.getAverageDistance());

                calcDistance.setNumber(m_shooter.calculatedCameraDistance);
                calcrpm.setNumber(m_shooter.cameraCalculatedSpeed);
                calctiltangle.setNumber(m_tilt.cameraCalculatedTiltPosition);

                inAutoEnabled.setBoolean(DriverStation.isAutonomousEnabled());
                inTeleopEnabled.setBoolean(DriverStation.isTeleopEnabled());
        }

        // Called once the command ends or is interrupted.
        @Override
        public void end(boolean interrupted) {
        }

        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
                return Timer.getFPGATimestamp() - m_autoStartTime > .04 && DriverStation.isTeleopEnabled()
                                && !m_shooter.isShooting;
        }
}
