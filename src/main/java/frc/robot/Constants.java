/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

   public static final double inchToMetersConversionFactor = 0.0254;
   public static final double MINIMUM_TURN_SPEED = 0;

   public static final class CANConstants {
      public static final int PDP = 1;

      public static final int DRIVETRAIN_LEFT_MASTER = 2;
      public static final int DRIVETRAIN_LEFT_FOLLOWER = 3;

      public static final int DRIVETRAIN_RIGHT_MASTER = 4;
      public static final int DRIVETRAIN_RIGHT_FOLLOWER = 5;

      public static final int LEFT_SHOOTER_MOTOR = 6;
      public static final int RIGHT_SHOOTER_MOTOR = 7;

      public static final int TURRET_ROTATE_MOTOR = 8;// turret

      public static final int TILT_MOTOR = 9;

      // talons

      public static final int REAR_INTAKE_MOTOR = 10;
      public static final int FRONT_INTAKE_MOTOR = 13;

      public static final int FRONT_ROLLER = 14;
      public static final int REAR_ROLLER = 12;

      // public static final int RIGHT_BELT_MOTOR = 11;

      public static final int CLIMB_MOTOR = 15;
      public static final int CP_TURN_MOTOR = 16;
   }

   public static class SolenoidConstants {
      public static final int CLIMBER_ARM_1 = 4;
      public static final int CLIMBER_ARM_2 = 5;
      public static final int CLIMBER_RATCHET_1 = 6;
      public static final int CLIMBER_RATCHET_2 = 7;

      public static final int FRONT_INTAKE_1 = 2;
      public static final int FRONT_INTAKE_2 = 3;

      public static final int REAR_INTAKE_1 = 0;
      public static final int REAR_INTAKE_2 = 1;

   }

   public static final class PDPConstants {

      // PDP slots
      public static final int DRIVETRAIN_LEFT_MOTOR_A_PDP = 1;
      public static final int DRIVETRAIN_LEFT_MOTOR_B_PDP = 2;
      public static final int DRIVETRAIN_RIGHT_MOTOR_A_PDP = 3;
      public static final int DRIVETRAIN_RIGHT_MOTOR_B_PDP = 4;
      public static final int ELEVATOR_MOTOR_A_PDP = 5;
      public static final int ELEVATOR_MOTOR_B_PDP = 6;
      public static final int SHOOTER_MOTOR_A_PDP = 2;
      public static final int SHOOTER_MOTOR_B_PDP = 3;

   }

   public static final class DriveConstants {

      /**
       * Neo brushless 4096 counts per rev Gearing 10 to 1 6" diameter wheels Neo
       * reports in revs so multiply rev by 4096 to get counts
       * 
       * 
       */
      // DIMENSIONS IN METERS
      // max speed = .4788 *(5700 /(60 *10.2 )) = 4.5 mps (14.5 fps)
      public static double WHEEL_DIAMETER = .1524;// 6"
      public static double WHEEL_CIRCUMFERENCE = .4788;// meters
      public static double METERS_PER_MOTOR_REV = 0.0467;// pi * diameter /geaar ratio
      // .0467 *5700/60 = 4.4 mps
      public static double NEO550_COUNTS_PER_REV = 4096;// not used
      public static double DRIVE_GEAR_RATIO = 10.25;

      public final static double WHEELBASE_WIDTH = .69;

      public static double ksVolts = .171;

      public static final double kPDriveVeSl = 1.15;

      public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            WHEELBASE_WIDTH);
      public static final boolean kGyroReversed = false;
      /**
       * 
       * Robot turn constants
       */
      public static double kTurnP = .05;
      public static double kTurnI = 0.;
      public static double kTurnD = 0.;
      public static double kMaxTurnRateDegPerS = 10.;
      public static double kMaxTurnAccelerationDegPerSSquared = 5.;
      public static double kTurnRateToleranceDegPerS = 1.;
      public static double kTurnToleranceDeg = 2.;

      /**
       * 
       * Position constants
       */
      public static double kPositionRateToleranceMetersPerS = 0.2;
      public static double kPositionToleranceMeters = 0.2;
      public static double kPositionI = 0.01;
      public static double kPositionP = .4;
      public static double kPositionD = .5;
      public static final double kMaxSpeedMetersPerSecond = 2.0;
      public static double kMaxPositionAccelerationMetersPerSSquared = 6;

      /**
       * 
       * Trajectory constants
       */

      public static double kMaxTrajectoryMetersPerSecond = 2;
      public static double kMaxTrajectoryAccelerationMetersPerSquared = 5.;
      public static double kPDriveVel = 5;
      public static double kPickupSpeedMetersPerSecond = 1;

      /**
       * Simulation parameters FAKE!!
       * 
       */
      public static double kvVoltSecondsPerMeter = 2;
      public static double kaVoltSecondsSquaredPerMeter = .208;
      public static final double kvVoltSecondsPerRadian = 3.0;
      public static final double kaVoltSecondsSquaredPerRadian = 0.3;

      public static final LinearSystem<N2, N2, N2> kDrivetrainPlant = LinearSystemId.identifyDrivetrainSystem(
            kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter, kvVoltSecondsPerRadian, kaVoltSecondsSquaredPerRadian);

      public static final DCMotor kDriveGearbox = DCMotor.getNEO(2);

   }

   public static final class ControlPanelConstants {

   }

   public static final class ClimberConstants {

   }

   public static class HoodedShooterConstants {

      public static final double MAX_SHOOTER_RPM = 5700;
      public static final double MAX_SHOOTER_MPS = 80;//

      /**
       * Tilt axis is a leadscrew driven through a 14T to 28T or 2:1 belt and a 10:1
       * gearbox = 20:1
       * 
       * The tilt has a mechanical base angle of around 60 degrees and a max angle of
       * around 90
       * 
       * Accurate angle calculations for target distance purposes are done based on
       * the tilt mechanical design
       * 
       * 
       * 
       * The calculated angle will be used for position display and closed loop
       * purposed. The shoot angle counts down as the hood moves up. The angle is
       * preset to max and the encoder is set to 0 on the bottom switch. Then the
       * encoder is subtracted from max so as it increases the angle decreases.
       * 
       * It is pretty much linear with turns.
       * 
       * Leadscrew has 122/20 or 6.1 turns from bottom limit switch
       * 
       * 
       */

      public static final double TILT_MIN_ANGLE = 1;
      public static final double TILT_MAX_ANGLE = 31.5;

      public static final double TILT_CLOSE_ANGLE = 22;
      public static final double TILT_MIDFIELD_ANGLE = 12;

      public static double TILT_HEIGHT = Units.inchesToMeters(26);

      public static double maxAngleChange = TILT_MAX_ANGLE - TILT_MIN_ANGLE;

      public static double maxLeadscrewTurns = 6.0;

      public static double maxMotorTurns = 120;

      public static double tiltRange = TILT_MAX_ANGLE - TILT_MIN_ANGLE;
      public static final double TILT_MID_ANGLE = TILT_MIN_ANGLE + ((TILT_MAX_ANGLE - TILT_MIN_ANGLE) / 2);
      public static double tiltDegreesPerRev = maxAngleChange / maxMotorTurns;// 30/120 = .25 degrees per rev

      /**
       * 20 revs of turret motor turns an 18 tooth pinion one time There are 228 teeth
       * in 360 degrees, so 1 tooth = 360/228 = 1.579 degrees So 18 teeth = 18 * 1.579
       * = 28.42 degrees and one motor rev is 1.421 degrees
       * 
       * At 6000 motor rpm = 100 rps the turret rotates at 1.421 * 100 = 142 deg per
       * With end to end travel of 200 degrees = 200/142 = 1.4 second
       * 
       * 
       * 
       */

      public static final double TURRET_MAX_ANGLE = 55;

      public static final double TURRET_MIN_ANGLE = -45;

      public static final double TURRET_DEG_PER_MOTOR_REV = 1.421;

   }

   public static final class IntakeConstants {

      public static final double REAR_SPEED = .5;

   }

   public static final class CargoTransportConstants {

      public static final double FRONT_PASS_SPEED = .5;
      public static final double FRONT_SHOOT_SPEED = .56;
      public static final double REAR_PASS_SPEED = -.5;
      public static final double REAR_SHOOT_SPEED = -.5;
      public static final double BELT_SPEED = -.5;

   }

   public static final class AutoConstants {

      public static final double kMaxAccelerationMetersPerSecondSquared = 10;

      // Reasonable baseline values for a RAMSETE follower in units of meters and
      // seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
   }

   public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final int kCoDriverControllerPort = 1;
      public static final int kSetupControllerPort = 3;
      public static final int kShootBoxControllerPort = 2;
   }

   public static final class FieldConstants {

      public static final double fieldWidth = Units.inchesToMeters(323);
      public static final double fieldLength = Units.inchesToMeters(626.25);
      public static final double initiationLine = Units.inchesToMeters(115);// meters from Alliance wall
      public static final double centerPowerPort = 2.4;// meters
      public static final double centerTrench = 0;
      public static final double trenchToSecondBall = 0;
      public static final double trenchToThirdBall = 0;

      public static final double trenchToUnderCP = 0;

      public static final double powerPortTargetCenterHeight = Units.inchesToMeters(84);
      public static final double PORT_CENTER_HEIGHT = Units.inchesToMeters(90);
      public static final double BASE_CAMERA_HEIGHT = Units.inchesToMeters(25.);
      public static final double SHOT_HEIGHT = PORT_CENTER_HEIGHT - BASE_CAMERA_HEIGHT;
      public static final double SHOT_HEIGHT_SQUARED = SHOT_HEIGHT * SHOT_HEIGHT;
      public static final double MAX_CAMERA_HEIGHT = Units.inchesToMeters(26.75);
      public static final double CARGO_DIAMETER = Units.inchesToMeters(7);
      public static final double INNER_PORT_DIAMETER = Units.inchesToMeters(13);
      public static final double INNER_PORT_DISTANCE = Units.inchesToMeters(29.25);
      public static final double OUTER_PORT_MAX_GAP = Units.inchesToMeters(30);
      public static final double MIN_BALL_CENTER_AT_OUTER = PORT_CENTER_HEIGHT - (OUTER_PORT_MAX_GAP / 2)
            + CARGO_DIAMETER / 2;
      public static final double CAMERA_BASE_ANGLE = 30.;
      public static final double CAMERA_MAX_ANGLE = 0;
      public static double cameraAngleRange = CAMERA_MAX_ANGLE - CAMERA_BASE_ANGLE;
      public static final double cameraHeightRange = MAX_CAMERA_HEIGHT - BASE_CAMERA_HEIGHT;
      public static double cameraHeightSlope = cameraHeightRange / HoodedShooterConstants.tiltRange;
      public static double cameraAngleSlope = cameraAngleRange / HoodedShooterConstants.tiltRange;
   }

   public static class PipelinesConstants {

      public static final int noZoomPipeline = 0;
      public static final int x2ZoomPipeline = 1;
      public static final int x3ZoomPipeline = 2;
      public static final int driverPipeline = 4;
      public static final int ledsOffPipeline = 8;

   }

   public static boolean logTilt;
   public static boolean logShoot;
   public static boolean logTurret;
   public static double kRobotPeriod = .02;
}
