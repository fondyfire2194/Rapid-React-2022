// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;

public class ArcadeDrive extends CommandBase {
  private final RevDrivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier lambdas. This command does not terminate.
   *
   * @param drivetrain           The drivetrain subsystem on which this command
   *                             will run
   * @param xaxisSpeedSupplier   Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDrive(RevDrivetrain drivetrain, Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSuppplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSuppplier;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double tempx = m_xaxisSpeedSupplier.get();
    double tempRot = m_zaxisRotateSupplier.get();

    if (Math.abs(tempx) < .05)
      tempx = 0;

    if (Math.abs(tempRot) < .05)
      tempRot = 0;
   
    m_drivetrain.arcadeDrive(tempx, tempRot * Pref.getPref("ArcadeTurnKp"));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
