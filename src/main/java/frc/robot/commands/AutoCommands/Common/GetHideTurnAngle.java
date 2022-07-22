// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.trajectories.FondyFireTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GetHideTurnAngle extends InstantCommand {
  private RevDrivetrain m_drive;
  private FondyFireTrajectory m_fftraj;
  private Pose2d m_cargoPose;
  private boolean m_reversed;

  private boolean showDebug;

  private double cargoFromHubCenter = Units.inchesToMeters(153);

  private double angleBetweenCargoCenters = 22.5;// degrees

  // 2×r× sin(c/2) chord of circle
  private double distanceBetweenCargo = 2 * cargoFromHubCenter
      * Math.sin(Units.degreesToRadians(angleBetweenCargoCenters / 2));

  public GetHideTurnAngle(RevDrivetrain drive, FondyFireTrajectory fftraj, Pose2d cargoPose, boolean reversed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drive = drive;
    m_fftraj = fftraj;
    m_cargoPose = cargoPose;
    m_reversed = reversed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // calculate x and y difference between own cargo and stop point
    // and get the kypotenuse
    double robotStartAngle = m_fftraj.leftCargo.getRotation().getDegrees();

    double xDiff = m_drive.getX() - m_cargoPose.getTranslation().getX();

    double yDiff = m_drive.getY() - m_cargoPose.getTranslation().getY();

    double hypDiff = Math.hypot(xDiff, yDiff);

    double robotFromHubCenter = cargoFromHubCenter + hypDiff;

    double angleOfChordPoints = (90 - angleBetweenCargoCenters / 2);

    double cargoToCargoTurnAngle = 180 - angleOfChordPoints;

    // a2=b2+c2−2⋅b⋅ccosα where b is cargo to cargo and c is new hypotenuse
    // law of cosines
    double distrobtooppcargo = Math.sqrt(Math.pow(distanceBetweenCargo, 2)
        + Math.pow(hypDiff, 2)
        - 2 * distanceBetweenCargo * hypDiff * Math.cos(Units.degreesToRadians(cargoToCargoTurnAngle)));

    // a/sin A = b/sin B so sin B = bsin A/a law of sines

    double suppangle = Units.radiansToDegrees(Math
        .asin(distanceBetweenCargo * Math.sin(Units.degreesToRadians(cargoToCargoTurnAngle)) / distrobtooppcargo));

    double robotTurnAngle = suppangle;// robot reverses

    double angleAdd = 0;
    if (m_reversed)
      angleAdd = 180;

    m_drive.setHideTurnAngle(robotTurnAngle - angleAdd);

    if (m_reversed)

      m_drive.setHidePickupDistance(-distrobtooppcargo);

    else

      m_drive.setHidePickupDistance(distrobtooppcargo);

    if (showDebug) {

      SmartDashboard.putNumber("hypdiff", hypDiff);

      SmartDashboard.putNumber("distbtwncargo", distanceBetweenCargo);

      SmartDashboard.putNumber("distrob2oppcargo", distrobtooppcargo);

      SmartDashboard.putNumber("C2CTA", cargoToCargoTurnAngle);

      SmartDashboard.putNumber("rfhc", robotFromHubCenter);

      SmartDashboard.putNumber("AOCP ", angleOfChordPoints);

      SmartDashboard.putNumber("RTA", robotTurnAngle);

      SmartDashboard.putNumber("RobStartAngle", robotStartAngle);

      SmartDashboard.putNumber("suppang", suppangle);
    }
  }
}
