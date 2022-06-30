package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants.ShooterRangeConstants;

public class FieldMap {

        // Field marks, for reference
        public static final double fieldWidth = Units.inchesToMeters(323);
        public static final double fieldLength = Units.inchesToMeters(629.25);

        // vision strips
        public static final double visionStripWidth = Units.inchesToMeters(2);
        public static final double visionStripLength = Units.inchesToMeters(5);
        public static final int numberOfVisionStrips = 16;
        public static final double angleStripToStrip = 360 / 16;// 22.5 degrees
        public static final double heightToBaseVisionStrip = Units.inchesToMeters(101.625);
        public static final double visionStripRingDiameter = Units.inchesToMeters(53.375);

        // each tarmac sees 4 vision strips right to left
        public static final double rightRightOfCenter = 67.5;
        public static final double right2RightOfCenter = 90;
        public static final double rightLeftOfCenter = 45;
        public static final double rightTwoLeftOfCenter = 22.5;

        public static final double leftRightOfCenter = -22.5;
        public static final double left2RightOfCenter = 0;
        public static final double leftLeftOfCenter = -45;
        public static final double leftTwoLeftOfCenter = -67.5;

        public static final int atHub = 0;
        public static final int tarmacLine = 1;

        // tarmacs
        // angles
        public static final double hubRotationToField = 66;// +/- 3 degrees information only

        // all angles are relative to hub

        public static final double rightTarmacCargoRightStartAngleToHub = 90;
        public static final double rightTarmacCargoCenterStartAngleToHub = 77;
        public static final double leftTarmacCargoStartAngleToHub = -135;

        public static final double leftTarmacLineFromHubCenter = Units.inchesToMeters(118.66);
        public static final double rightTarmacLineFromCargoCenter = Units.inchesToMeters(40.44);
        public static final double rightTarmacCornerFromCargoCenter = Units.inchesToMeters(15.56);
        public static final double rightTarmacRightCargoToFieldWall = Units.inchesToMeters(11.21);
        public static final double rightTarmacMidCargoCenterToTarmacMidEdge = Units.inchesToMeters(34.46);
        public static final double rightTarmacCargoMidCenterToTarmacCenterEdge = Units.inchesToMeters(34.46);

        public static final double rightTarmacCenterStartAngle = 0;

        // cargo

        public static final double cargoFieldRingDiameter = Units.inchesToMeters(153);
        public static final double cargoDiameter = Units.inchesToMeters(9.5);

        // Robot dimensions with bumpers
        public static final double robotWidth = Units.inchesToMeters(34.75);//.8826
        public static final double robotLength = Units.inchesToMeters(37.75);//.9588
        public static double frontIntakeAdder = Units.inchesToMeters(9);//.2286
        public static double rearIntakeAdder = Units.inchesToMeters(9);
        public static final double robotTiltPivotHeight = Units.inchesToMeters(40);

        // common values

        public static double intakeSpeed = .75;
        public static final double drivePickupRate = .5;
        public static final double shotDelay = .75;

        // left tarmac auto start upper

        // robot starts with rear bumper behind tarmac
        // perimeter and centered on target and cargo

        public static final double leftTarmacPickupPosition = Units.inchesToMeters(-50);
        public static final double leftTarmacShootPosition = Units.inchesToMeters(-30.);

        public static final double leftTarmacTiltUpperAngle = ShooterRangeConstants.tiltRange2;
        public static final double leftTarmacTurretUpperAngle = 0;
        public static double leftTarmacUpperRPM = 3750;

        public static final double[] leftTarmacData = { leftTarmacPickupPosition,
                        leftTarmacShootPosition, leftTarmacTiltUpperAngle,
                        leftTarmacTurretUpperAngle, leftTarmacUpperRPM

        };

        public static final double rightTarmacPickupPosition = Units.inchesToMeters(-41);
        public static final double rightTarmacShootPosition = Units.inchesToMeters(-35);

        public static final double rightTarmacTiltUpperAngle = 15;
        public static final double rightTarmacTurretUpperAngle = 20;
        public static double rightTarmacUpperRPM = 3750;

        public static final double[] rightTarmacData = { rightTarmacPickupPosition,
                        rightTarmacShootPosition, rightTarmacTiltUpperAngle,
                        rightTarmacTurretUpperAngle, rightTarmacUpperRPM

        };

        public static final double centerTarmacPickupPosition = Units.inchesToMeters(-51);
        public static final double centerTarmacShootPosition = Units.inchesToMeters(-35);

        public static final double centerTarmacTiltUpperAngle = 15;
        public static final double centerTarmacTurretUpperAngle = -17;
        public static double centerTarmacUpperRPM = 3750;

        public static final double[] centerTarmacData = { centerTarmacPickupPosition,
                        centerTarmacShootPosition, centerTarmacTiltUpperAngle,
                        centerTarmacTurretUpperAngle, centerTarmacUpperRPM,
        };

        public static final double[] shootLocation_0 = { 4, 1850 };// { 1, 3500 };// 40" at hub "lob"
        public static final double[] shootLocation_1 = { 8, 2150 };// { 6, 4000 };// 100" at tarmac line
        public static final double[] shootLocation_2 = { 14, 4500 };// 200" inner launch pad
        public static final double[] shootLocation_3 = { 0, 500 };// set by operator during test

        public static String[] shootLocationName = { "At Hub", "Tarmac Line", "Inner Launchpad", "TestSetup" };

        public static final String[] shootModeName = { "Throttle", "Camera", "Preset" };
}
