/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.OI.HubVisionShuffleboard;
import frc.robot.OI.LLVisionShuffleboard;
import frc.robot.OI.SetUpAutoOI;
import frc.robot.OI.SetUpOI;
import frc.robot.Vision.AngleSolver;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.Vision.LimelightControlMode.StreamType;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.CargoTransport.ReleaseCargo;
import frc.robot.commands.CargoTransport.RunRollers;
import frc.robot.commands.CargoTransport.StopRollers;
import frc.robot.commands.Climber.ClimberArm;
import frc.robot.commands.Climber.RunClimber;
import frc.robot.commands.Intakes.RunActiveIntakeMotor;
import frc.robot.commands.Intakes.StartActiveIntake;
import frc.robot.commands.Intakes.StopIntakeMotors;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.ArcadeDriveVelocity;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.JogShooterVelocity;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShootCargo;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.TiltJog;
import frc.robot.commands.Tilt.TiltJogVelocity;
import frc.robot.commands.Tilt.TiltWaitForStop;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.TurretJog;
import frc.robot.commands.Turret.TurretJogVelocity;
import frc.robot.commands.Turret.TurretWaitForStop;
import frc.robot.commands.Vision.SetUpLimelightForDriver;
import frc.robot.commands.Vision.SetUpLimelightForNoVision;
import frc.robot.commands.Vision.SetVisionMode;
import frc.robot.subsystems.CargoTransportSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakesSubsystem;
import frc.robot.subsystems.RevDrivetrain;
import frc.robot.subsystems.RevShooterSubsystem;
import frc.robot.subsystems.RevTiltSubsystem;
import frc.robot.subsystems.RevTurretSubsystem;
import frc.robot.trajectories.FondyFireTrajectory;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
      // The robot's subsystems

      // The driver's controller
      public final Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
      public final XboxController codriverGamepad = new XboxController(OIConstants.kCoDriverControllerPort);
      public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);

      public final RevDrivetrain m_drive;

      public final IntakesSubsystem m_intakes;

      public final CargoTransportSubsystem m_transport;

      public final RevTurretSubsystem m_turret;

      public final RevTiltSubsystem m_tilt;

      public final RevShooterSubsystem m_shooter;

      public final ClimberSubsystem m_climber;

      public final RawContoursV2 m_rCV2;

     // public final AngleSolver m_as;

      public static boolean autoSelected;

      public SetUpOI m_setup;

      public SetUpAutoOI m_autoOi;

      public HubVisionShuffleboard m_hvis;

      public LLVisionShuffleboard m_llVis;

      public LimeLight m_limelight;

      public Compressor m_compressor;

      public FondyFireTrajectory m_trajectory;

      public VisionReferenceTarget m_vrt;

      public boolean clickUp;

      // Co driver gamepad

      // Setup gamepad LOGITECH
      JoystickButton codriverX = new JoystickButton(codriverGamepad, 1);
      JoystickButton codriverA = new JoystickButton(codriverGamepad, 2);
      JoystickButton codriverB = new JoystickButton(codriverGamepad, 3);
      JoystickButton codriverY = new JoystickButton(codriverGamepad, 4);
      JoystickButton codriverLeftTrigger = new JoystickButton(codriverGamepad, 5);
      JoystickButton codriverRightTrigger = new JoystickButton(codriverGamepad, 6);

      JoystickButton codriverBack = new JoystickButton(codriverGamepad, 9);
      JoystickButton codriverStart = new JoystickButton(codriverGamepad, 10);

      JoystickButton codriverLeftStick = new JoystickButton(codriverGamepad, 11);
      JoystickButton codriverRightStick = new JoystickButton(codriverGamepad, 12);

      public POVButton codriverUpButton = new POVButton(codriverGamepad, 0);
      public POVButton codriverRightButton = new POVButton(codriverGamepad, 90);
      public POVButton codriverDownButton = new POVButton(codriverGamepad, 180);
      public POVButton codriverLeftButton = new POVButton(codriverGamepad, 270);

      // Setup gamepad XBox 3
      JoystickButton setupA = new JoystickButton(setupGamepad, 1);
      JoystickButton setupB = new JoystickButton(setupGamepad, 2);
      JoystickButton setupX = new JoystickButton(setupGamepad, 3);
      JoystickButton setupY = new JoystickButton(setupGamepad, 4);
      JoystickButton setupLeftTrigger = new JoystickButton(setupGamepad, 5);
      JoystickButton setupRightTrigger = new JoystickButton(setupGamepad, 6);

      JoystickButton setupBack = new JoystickButton(setupGamepad, 7);
      JoystickButton setupStart = new JoystickButton(setupGamepad, 8);

      JoystickButton setupLeftStick = new JoystickButton(setupGamepad, 11);
      JoystickButton setupRightStick = new JoystickButton(setupGamepad, 12);

      public POVButton setupUpButton = new POVButton(setupGamepad, 0);
      public POVButton setupRightButton = new POVButton(setupGamepad, 90);
      public POVButton setupDownButton = new POVButton(setupGamepad, 180);
      public POVButton setupLeftButton = new POVButton(setupGamepad, 270);

      public POVButton driverUpButton = new POVButton(m_driverController, 0);
      public POVButton driverRightButton = new POVButton(m_driverController, 90);
      public POVButton driverDownButton = new POVButton(m_driverController, 180);
      public POVButton driverLeftButton = new POVButton(m_driverController, 270);
      public boolean isMatch;

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {

            // Preferences.removeAll();
            // Pref.deleteUnused();
            // Pref.addMissing();
            m_drive = new RevDrivetrain();
            m_transport = new CargoTransportSubsystem();
            // m_rearIntake = new RearIntakeSubsystem();
            // m_frontIntake = new FrontIntakeSubsystem();
            m_intakes = new IntakesSubsystem();

            m_shooter = new RevShooterSubsystem();
            m_turret = new RevTurretSubsystem();
            m_tilt = new RevTiltSubsystem();
            m_climber = new ClimberSubsystem();

            m_limelight = new LimeLight();
            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);
            m_limelight.useVision = false;

            m_rCV2 = new RawContoursV2(m_limelight);
            

           // m_as = new AngleSolver(m_rCV2,m_limelight);


            m_vrt = new VisionReferenceTarget(m_rCV2, m_limelight);

            m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

            // m_autoFactory = new AutoFactory(m_shooter, m_turret, m_tilt, m_transport,
            // m_robotDrive, m_limelight,
            // m_compressor, m_intake);

            m_trajectory = new FondyFireTrajectory(m_drive);

            m_tilt.setDefaultCommand(new PositionHoldTilt(m_tilt,
                        m_limelight));
            // // m_tilt.setDefaultCommand(new PositionHoldTiltTest(m_tilt));

            m_turret.setDefaultCommand(new PositionHoldTurret(m_turret,
                        m_limelight));

            // m_turret.setDefaultCommand(new TurretJog(m_turret, () ->
            // setupGamepad.getRawAxis(0) / 5, setupGamepad));

            isMatch = Pref.getPref("IsMatch") == 1.;

            m_setup = new SetUpOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor,
                        m_limelight, m_intakes, m_climber, m_trajectory, isMatch);

            m_autoOi = new SetUpAutoOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor, m_limelight,
                        m_intakes, m_climber, m_trajectory, m_rCV2,  isMatch);

            m_hvis = new HubVisionShuffleboard(m_limelight, m_rCV2, m_vrt, m_turret, m_tilt, m_shooter, isMatch);

            m_llVis = new LLVisionShuffleboard(m_limelight, m_rCV2,m_turret, m_tilt, m_shooter, isMatch);

            m_drive.setDefaultCommand(getArcadeDriveCommand());

            configureButtonBindings();

            LiveWindow.disableAllTelemetry();

            CommandScheduler.getInstance()
                        .onCommandInitialize(command -> System.out.println(command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> System.out.println(command.getName() + " has ended"));
            CommandScheduler.getInstance()
                        .onCommandInterrupt(command -> System.out.println(command.getName() + " was interrupted"));
            CommandScheduler.getInstance().onCommandInitialize(
                        command -> SmartDashboard.putString("CS", command.getName() + " is starting"));
            CommandScheduler.getInstance()
                        .onCommandFinish(command -> SmartDashboard.putString("CE", command.getName() + " has Ended"));
            CommandScheduler.getInstance().onCommandInterrupt(
                        command -> SmartDashboard.putString("CE", command.getName() + "was Interrupted"));

      }

      /**
       * Use this method to define your button->command mappings. Buttons can be
       * created by instantiating a {@link GenericHID} or one of its subclasses
       * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
       * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
       */
      private void configureButtonBindings() {

            /**
             * 
             * Driver Joystick assignments
             * 
             */

            new JoystickButton(m_driverController, 2).whileHeld(new StartActiveIntake(m_intakes, .75));

            new JoystickButton(m_driverController, 1)

                        .whileHeld(new ShootCargo(m_shooter, m_tilt, m_turret,
                                    m_limelight, m_transport, m_compressor, 100));

            new JoystickButton(m_driverController, 5).whenPressed(new RunShooter(m_shooter))
                        .whenPressed(new RunRollers(m_transport));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport))
                        .whenPressed(new StopRollers(m_transport))
                        .whenPressed(new SetUpLimelightForNoVision(m_limelight))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whenReleased(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle));

            new JoystickButton(m_driverController, 4).whenPressed(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whenPressed(new SetUpLimelightForNoVision(m_limelight));

            // new JoystickButton(m_driverController, 6).

            new JoystickButton(m_driverController, 7).whileHeld(new PositionTilt(m_tilt, m_tilt.tiltMinAngle))
                        .whenPressed(new PositionTurret(m_turret, 0))
                        .whileHeld(new SetUpLimelightForDriver(m_limelight))
                        .whenReleased(new PositionTilt(m_tilt, m_tilt.tiltMaxAngle))
                        .whenReleased(new SetVisionMode(m_limelight))
                        .whenReleased(new SetUpLimelightForNoVision(m_limelight));

            // new JoystickButton(m_driverController, 8)

            // new JoystickButton(m_driverController, 9)

            // new JoystickButton(m_driverController, 11).

            // Hold to shoot all
            new JoystickButton(m_driverController, 12).whileHeld(() -> m_shooter.shootAll())
                        .whenReleased(() -> m_shooter.shootOne());

            driverUpButton.whenPressed(() -> m_tilt.aimHigher());

            driverDownButton.whenPressed(() -> m_tilt.aimLower());

            driverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft());// shoot right

            driverRightButton.whenPressed(() -> m_turret.aimFurtherRight());// shoot left

            /**
             * co driver sets the tilt and turret for shoot positions
             * 
             * 
             */

            // codriverY.whenPressed(

            // codriverB.whenPressed

            // codriverX.whenPressed(

            // // move from trench behind control panel to trench shoot position, lock on
            // // target and start shooter

            // codriverA.whenPressed(new Trench4Macro(m_robotDrive, m_shooter, m_turret,
            // m_tilt, m_transport, m_intake,
            // m_limelight));

            // // climber

            codriverLeftTrigger

                        .whenPressed(() -> m_climber.unlockRatchet())

                        .whileHeld(getRunClimberMotorCommand(codriverGamepad))

                        .whenReleased(() -> m_climber.stopMotor())

                        .whenReleased(() -> m_climber.lockRatchet());

            // codriverRightTrigger

            codriverBack.whenPressed(new ClimberArm(m_climber, true));

            codriverStart.whenPressed(new ClimberArm(m_climber, false));

            codriverUpButton.whenPressed(() -> m_tilt.aimHigher());

            codriverDownButton.whenPressed(() -> m_tilt.aimLower());

            codriverLeftButton.whenPressed(() -> m_turret.aimFurtherLeft());// shoot right

            codriverRightButton.whenPressed(() -> m_turret.aimFurtherRight());// shoot left

            // codriverUpButton.whenPressed(new ChangeShooterSpeed(m_shooter, +1));
            // codriverDownButton.whenPressed(new ChangeShooterSpeed(m_shooter, -1));
            // codriverRightButton.whenPressed(new ChangeShooterSpeed(m_shooter, +2));
            // codriverLeftButton.whenPressed(new ChangeShooterSpeed(m_shooter, -2));
            /**
             * Setup gamepad is used for testing functions
             */

            // setupDownButton.whenPressed(new ParallelCommandGroup((new
            // PositionTiltToVision(m_tilt, m_limelight, 24)),
            // new PositionTurretToVision(m_turret, m_limelight, -32)));

            setupUpButton.whileHeld(() -> m_transport.runTopRollerMotor(.5))
                        .whileHeld(() -> m_transport.runLowerRollerMotor(.5))
                        .whenReleased(() -> m_transport.stopTopRollerMotor())
                        .whenReleased(() -> m_transport.stopLowerRollerMotor());

            setupLeftButton.whenPressed(new ReleaseCargo(m_transport));

            // setupBack.whenPressed(new LogHubTarget(m_rCV2, m_as, m_getTarget, m_tilt,
            // m_turret, m_limelight));

            setupX.whileHeld(getJogTiltVelocityCommand(setupGamepad))
                        .whileHeld(getJogTurretVelocityCommand(setupGamepad)).whenReleased(new TiltWaitForStop(m_tilt))
                        .whenReleased(new TurretWaitForStop(m_turret));

            setupY.whileHeld(getJogTiltCommand(setupGamepad)).whileHeld(getJogTurretCommand(setupGamepad))
                        .whenReleased(new TiltWaitForStop(m_tilt)).whenReleased(new TurretWaitForStop(m_turret));

            setupA.whileHeld(new RunActiveIntakeMotor(m_intakes, .75))
                        .whenReleased(new StopIntakeMotors(m_intakes));

            // setupLeftStick.

            // setupRightStick.

            setupBack.whenHeld(getJogShooterCommand())
                        .whenReleased(m_shooter::stop);

            setupRightButton.whenHeld(getJogShooterVelocityCommand())
                        .whenReleased(m_shooter::stop);

            LiveWindow.disableAllTelemetry();

      }

      /**
       * Use this to pass the autonomous command to the main {@link Robot} class.
       *
       * @return the command to run in autonomous
       */

      public Command getAutonomousCommand() {
            return null;

      }

      public Command getArcadeDriveCommand() {
            return new ArcadeDrive(m_drive, () -> -m_driverController.getY(), () -> m_driverController.getTwist());
      }

      public Command getArcadeDriveVelocityCommand() {
            return new ArcadeDriveVelocity(m_drive, () -> -m_driverController.getY(),
                        () -> m_driverController.getTwist());
      }

      public Command getDriveStraightCommand() {
            return new DriveStraightJoystick(m_drive, () -> -m_driverController.getY());

      }

      public Command getJogTurretCommand(XboxController gamepad) {
            return new TurretJog(m_turret, () -> gamepad.getRawAxis(0) / 10, gamepad);
      }

      public Command getJogTiltCommand(XboxController gamepad) {
            return new TiltJog(m_tilt, () -> -gamepad.getRawAxis(1) / 5, gamepad);
      }

      /**
       * Use to tune velocity loop for use inside vision position loops. Goal is to
       * correct 5 degrees of vision error in 1 second.
       * 
       * One tilt motor rev is .25 degrees. 11000rpm = 180 rps = 45 dps max
       * 
       * 5 degrees vision = 20 motor revs per second.,approx 10% full speed
       * 
       * 
       * 
       * One turret motor rev is 1.41 degrees. 11000 rpm = 180 rps = 250 dps
       * 
       * So 5 degrees in one second 5 approx 1/150 or 7% full speed
       * 
       * 
       * 
       * 
       * @param gamepad
       * @return
       */

      public Command getJogTurretVelocityCommand(XboxController gamepad) {
            return new TurretJogVelocity(m_turret, () -> -gamepad.getRawAxis(0) / 5, gamepad);
      }

      public Command getJogTiltVelocityCommand(XboxController gamepad) {
            return new TiltJogVelocity(m_tilt, () -> -gamepad.getRawAxis(1), gamepad);
      }

      public Command getJogShooterCommand() {

            return new JogShooter(m_shooter, () -> setupGamepad.getRawAxis(4));
      }

      public Command getJogShooterVelocityCommand() {
            return new JogShooterVelocity(m_shooter, () -> setupGamepad.getRawAxis(4));
      }

      public Command getRunClimberMotorCommand(XboxController gamepad) {

            return new RunClimber(m_climber, () -> codriverGamepad.getRawAxis(1));
      }

      public double getThrottle() {
            return (1 - m_driverController.getThrottle()) / 2;
      }

      public void checkCANDevices() {
            m_turret.checkCAN();
            m_tilt.checkCAN();
            m_intakes.checkFrontCAN();
            m_intakes.checkRearCAN();
            m_shooter.checkCAN();
            m_drive.checkCAN();
            m_transport.checkCAN();

      }

      public double[] getPDPInfo() {
            double temp[] = { 0, 0, 0, 0, 0 };
            temp[0] = m_shooter.getBatteryVoltage();
            temp[1] = m_shooter.getTemperature();
            temp[2] = m_shooter.getTotalEnergy() / 3600;
            temp[3] = m_shooter.getTotalPower();
            return temp;

      }

      public void checkLimits() {
            if (m_tilt.onMinusSoftwareLimit() || m_tilt.onPlusSoftwareLimit() || m_tilt.onMinusHardwarLimit()
                        || m_turret.onPlusSoftwareLimit() || m_turret.onMinusSoftwareLimit()
                        || DriverStation.isDisabled())
                  m_limelight.useVision = false;

      }

}