// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import frc.robot.Pref;
import frc.robot.subsystems.RevDrivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.Supplier;

public class ArcadeDriveVelocity extends CommandBase {
  private final RevDrivetrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;

  private final double maxSpeed = 3;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to
   * the speed supplier lambdas. This command does not terminate.
   *
   * @param drivetrain           The drivetrain subsystem on which this command
   *                             will run
   * @param xaxisSpeedSupplier   Lambda supplier of forward/backward speed
   * @param zaxisRotateSuppplier Lambda supplier of rotational speed
   */
  public ArcadeDriveVelocity(RevDrivetrain drivetrain, Supplier<Double> xaxisSpeedSupplier,
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

    double tempX = m_xaxisSpeedSupplier.get();
    double tempRot = m_zaxisRotateSupplier.get();

    if (Math.abs(tempX) < .05)
      tempX = 0;
    if (Math.abs(tempRot) < .05)
      tempRot = 0;

    double leftSpeed = maxSpeed * (tempX + tempRot / 2);

    double rightSpeed = maxSpeed * (tempX - tempRot / 2);

    double yawComp = m_drivetrain.getYaw() * Pref.getPref("dRStKp");

    leftSpeed -= yawComp;
    
    rightSpeed += yawComp;

    m_drivetrain.smartVelocityControlMetersPerSec(leftSpeed, rightSpeed);

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
