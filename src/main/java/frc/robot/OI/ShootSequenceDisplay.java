// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.OI;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevShooterSubsystem;

/** Add your docs here. */
public class ShootSequenceDisplay implements Sendable {
    private CargoTransportSubsystem m_transport;
    private RevShooterSubsystem m_shooter;
    private IntakesSubsystem m_intake;

    public ShootSequenceDisplay(CargoTransportSubsystem transport, RevShooterSubsystem shooter,
            IntakesSubsystem intake) {
        m_transport = transport;
        m_shooter = shooter;
        m_intake = intake;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
            
        builder.setSmartDashboardType("ShootSequence");
        builder.addBooleanProperty("cargo_at_shoot", m_transport::getCargoAtShoot,
                null);
        builder.addDoubleProperty("low_roll_rpm", m_transport::getLowerRPM, null);
        builder.addBooleanProperty("cargo_is_blue", m_transport::getCargoIsBlue,
                null);
        builder.addBooleanProperty("cargo_is_red", m_transport::getCargoRed, null);
        builder.addBooleanProperty("cargo_wrong_color",
                m_transport::getCargoAllianceMisMatch, null);

        builder.addDoubleProperty("shooter_rpm", m_shooter::getRPM, null);
        builder.addDoubleProperty("top_roll_rpm", m_shooter::getTopRPM, null);
        builder.addBooleanProperty("shoot_at_speed", m_shooter::getShooterAtSpeed, null);
        builder.addBooleanProperty("is_shooting", m_shooter::getShooterIsShooting, null);
        builder.addBooleanProperty("top_roll_at_speed", m_shooter::getTopRollerAtSpeed, null);

        // builder.addBooleanProperty("cargo_at_front", m_intake::getCargoAtFront,
        // null);
        builder.addBooleanProperty("cargo_at_rear", m_intake::getCargoAtRear, null);
        // builder.addDoubleProperty("front_motor_out", m_intake::getFrontMotor, null);
        builder.addDoubleProperty("rear_motor_out", m_intake::getRearMotor, null);

    }
}
