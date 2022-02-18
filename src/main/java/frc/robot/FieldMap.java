package frc.robot;

import edu.wpi.first.math.util.Units;

public class FieldMap {

        // Field marks, for reference
        public static final double fieldWidth = Units.inchesToMeters(323);
        public static final double fieldLength = Units.inchesToMeters(629.25);

        // vision strips
        public static final double visionStripWidth = Units.inchesToMeters(2);
        public static final double visionStripLength = Units.inchesToMeters(5);
        public static final int numberOfVisionStrips = 16;
        public static final double angleStripToStrip = 360 / 16;//22.5 degrees

        // each tarmac sees 4 vision strips right to left
        public static final double rightRightOfCenter = 67.5;                        
        public static final double right2RightOfCenter = 90;
        public static final double rightLeftOfCenter = 45;
        public static final double rightTwoLeftOfCenter = 22.5;

        public static final double leftRightOfCenter = -22.5;
        public static final double left2RightOfCenter = 0;
        public static final double leftLeftOfCenter = -45;
        public static final double leftTwoLeftOfCenter = -67.5;

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
        public static final double robotWidth = Units.inchesToMeters(34.75);
        public static final double robotLength = Units.inchesToMeters(37.75);
        public static double frontIntakeAdder = Units.inchesToMeters(.3);
        public static double rearIntakeAdder = Units.inchesToMeters(.3);
        public static final double robotTiltPivotHeight = Units.inchesToMeters(40);

        // common values

        public static double intakeSpeed = .75;
        public static final double drivePickupRate = .5;
        public static final double shotDelay = .75;

        // left tarmac auto start upper

        // robot starts with rear bumper behind tarmac
        // perimeter and centered on target cargo

        public static final double leftTarmacTiltAngle = 15;
        public static final double leftTarmacTurretAngle = 15;
        public static final double leftTarmacDriveToPosition = (cargoFieldRingDiameter / 2)
                        - leftTarmacLineFromHubCenter
                        - rearIntakeAdder + robotLength;
        public static final double tiltTargetPosition = 25;
        public static final double turretTargetPosition = 10;
        public static double leftStartMPS = 21;

        // right tarmac auto start upper
        public static final double rightTarmacTiltAngle = 15;
        public static final double rightTarmacTurretAngle = 15;
        public static final double rightTarmacDrivePosition = 2;
        public static final double rightTiltTargetPosition = 25;
        public static final double rightTurretTargetPosition = 10;

        public static double rightStartMPS = 21;

}
