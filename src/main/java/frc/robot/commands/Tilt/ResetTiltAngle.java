// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tilt;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodedShooterConstants;
import frc.robot.subsystems.RevTiltSubsystem;

public class ResetTiltAngle extends CommandBase {
    /** Creates a new ResetTiltAngle. */
    private final RevTiltSubsystem m_tilt;
    private int loopCtr;

    public ResetTiltAngle(RevTiltSubsystem tilt) {
        // Use addRequirements() here to declare subsy limelightstem dependencies.
        m_tilt = tilt;
        addRequirements(m_tilt);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        loopCtr = 0;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        loopCtr++;
        m_tilt.resetAngle();
        m_tilt.targetAngle = HoodedShooterConstants.TILT_MAX_ANGLE;
        if (RobotBase.isReal())
            m_tilt.setSoftwareLimits();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return loopCtr > 0;

    }
}
