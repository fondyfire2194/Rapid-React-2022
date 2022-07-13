// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RobotDrive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RevDrivetrain;

public class RunFeedForward extends CommandBase {
  /** Creates a new RunAtMPS. */
  private RevDrivetrain m_drive;

  double minMPS = 0;

  double mpsIncrementPer20ms;

  double maxMPS = 2.5;

  double time = 3;

  private double mpsIncremementPerSecond;

  private boolean slowDown;

  private boolean m_minus;

  private boolean m_step;

  private double[] mpsStep = { .5, 1, 1.5, 2, 2.5, 3 };

  private int stepPointer;

  private double startTime;

  public RunFeedForward(RevDrivetrain drive, boolean minus, boolean step) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_minus = minus;
    m_step = step;

    addRequirements(m_drive);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.currentMPS = 0;
    m_drive.resetEncoders();
    m_drive.resetGyro();
    slowDown = false;

    if (!m_step) {
      mpsIncremementPerSecond = maxMPS / (time / 2);

      SmartDashboard.putNumber("MPSIPS", mpsIncremementPerSecond);

      mpsIncrementPer20ms = mpsIncremementPerSecond / 50;

    } else {
      stepPointer = 0;
      startTime = Timer.getFPGATimestamp();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_step) {
      if (!slowDown) {

        m_drive.currentMPS += mpsIncrementPer20ms;

        if (m_drive.currentMPS >= maxMPS)

          slowDown = true;

      }

      if (slowDown) {

        m_drive.currentMPS -= mpsIncrementPer20ms;

        if (m_drive.currentMPS < .1)

          m_drive.currentMPS = 0;
      }

    }

    else {

      m_drive.currentMPS = mpsStep[stepPointer];

      if (Timer.getFPGATimestamp() > startTime + 2

          && stepPointer < mpsStep.length - 1) {

        stepPointer++;

        startTime = Timer.getFPGATimestamp();
      }
      if (stepPointer > mpsStep.length - 1)
        m_drive.currentMPS = 0;

    }
    if (m_minus)

      m_drive.tankDriveWithFeedforward(-m_drive.currentMPS, -m_drive.currentMPS);

    else

      m_drive.tankDriveWithFeedforward(m_drive.currentMPS, m_drive.currentMPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.currentMPS = 0;
    m_drive.tankDriveWithFeedforward(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
