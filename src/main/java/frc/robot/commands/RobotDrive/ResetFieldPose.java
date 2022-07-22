/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.RobotDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ResetFieldPose extends InstantCommand {
   private final RevDrivetrain drive;
   private boolean m_reversed;

   public ResetFieldPose(RevDrivetrain drive, boolean reversed) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.drive = drive;
      m_reversed=reversed;

   }

   // Called when the command is initially scheduled.
   @Override
   public void initialize() {
      SmartDashboard.putString("PrePose", drive.mOdometry.getPoseMeters().toString());
      SmartDashboard.putNumber("HEAD", drive.getHeading());
      drive.resetEncoders();
      double angleAdd = 0;
      if (m_reversed)
         angleAdd = 180;

      Pose2d now = new Pose2d(drive.getX(), drive.getY(), Rotation2d.fromDegrees(angleAdd + drive.getHeading()));
      SmartDashboard.putNumber("HEAD2", now.getRotation().getDegrees());

      drive.resetPose(now);

   }
}
