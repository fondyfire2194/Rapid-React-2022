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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.Constants.TiltConstants;
import frc.robot.OI.HubVisionShuffleboard;
import frc.robot.OI.LLVisionShuffleboard;
import frc.robot.OI.SetUpAutoOI;
import frc.robot.OI.SetUpOI;
import frc.robot.OI.SetUpPreRoundOI;
import frc.robot.OI.ShootSequenceDisplay;
import frc.robot.OI.Show_Hide_Screens;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.Vision.LimelightControlMode.StreamType;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.TurnLedsOnOff;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.AutoCommands.Common.AcquireTarget;
import frc.robot.commands.AutoCommands.Common.SetupForShootLocation;
import frc.robot.commands.CargoTransport.RunLowerRollerIntake;
import frc.robot.commands.CargoTransport.StopLowerRoller;
import frc.robot.commands.Climber.RunClimber;
import frc.robot.commands.Climber.StopClimber;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.Shooter.ChangeShooterSpeed;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.JogShooterVelocity;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.ShootCargo;
//import frc.robot.commands.Shooter.ShootTwoCargo;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Tilt.PositionHoldTilt;
import frc.robot.commands.Tilt.PositionTilt;
import frc.robot.commands.Tilt.TiltJog;
import frc.robot.commands.Tilt.TiltJogVelocity;
import frc.robot.commands.Tilt.TiltWaitForStop;
import frc.robot.commands.Turret.PositionHoldTurret;
import frc.robot.commands.Turret.PositionTurret;
import frc.robot.commands.Turret.PositionTurretIncremental;
import frc.robot.commands.Turret.PositionTurretToVision;
import frc.robot.commands.Turret.TurretJog;
import frc.robot.commands.Turret.TurretJogVelocity;
import frc.robot.commands.Turret.TurretWaitForStop;
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
      // public final XboxController setupGamepad = new
      // XboxController(OIConstants.kSetupControllerPort);

      public final RevDrivetrain m_drive;

      public final IntakesSubsystem m_intake;

      public final CargoTransportSubsystem m_transport;

      public final RevTurretSubsystem m_turret;

      public final RevTiltSubsystem m_tilt;

      public final RevShooterSubsystem m_shooter;

      public final ClimberSubsystem m_climber;

      public final RawContoursV2 m_rcv2;

      public static boolean autoSelected;

      public SetUpOI m_setup;

      public SetUpAutoOI m_autoOi;

      public SetUpPreRoundOI m_preOi;

      public HubVisionShuffleboard m_hvis;

      public LLVisionShuffleboard m_llVis;

      public Show_Hide_Screens m_sh;

      public LimeLight m_limelight;

      public Compressor m_compressor;

      public FondyFireTrajectory m_trajectory;

      public VisionReferenceTarget m_vrt;

      public ShootSequenceDisplay ssdisp;

      // Drive joystick

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

      public POVButton driverUpButton = new POVButton(m_driverController, 0);
      public POVButton driverRightButton = new POVButton(m_driverController, 90);
      public POVButton driverDownButton = new POVButton(m_driverController, 180);
      public POVButton driverLeftButton = new POVButton(m_driverController, 270);

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {
            // Preferences.removeAll();
            Pref.deleteUnused();
            Pref.addMissing();
            m_drive = new RevDrivetrain();
            m_transport = new CargoTransportSubsystem();

            m_intake = new IntakesSubsystem();

            m_shooter = new RevShooterSubsystem();
            m_turret = new RevTurretSubsystem();
            m_tilt = new RevTiltSubsystem();
            m_climber = new ClimberSubsystem();
            m_limelight = new LimeLight();

            m_shooter.setDefaultCommand(new JogShooter(m_shooter, () -> 0.));

            m_drive.setDefaultCommand(getArcadeDriveCommand());

            m_intake.setDefaultCommand((new StopActiveIntake(m_intake)));

            m_transport.setDefaultCommand(new StopLowerRoller(m_transport));

            m_tilt.setDefaultCommand(new PositionHoldTilt(m_tilt));

            m_turret.setDefaultCommand(new PositionHoldTurret(m_turret,
                        m_limelight));

            m_climber.setDefaultCommand(new StopClimber(m_climber));

            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);
            m_limelight.useVision = false;

            m_rcv2 = new RawContoursV2(m_limelight);

            m_vrt = new VisionReferenceTarget(m_rcv2, m_limelight);

            m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

            m_trajectory = new FondyFireTrajectory(m_drive);
            ssdisp = new ShootSequenceDisplay(m_transport, m_shooter, m_intake);
            // test configuration
            // Show_Hide_Screens.setStates(false, false, false, true);
            // test configuration with hub vision
            // Show_Hide_Screens.setStates(false, false, true, true);

            // competition configuration
            Show_Hide_Screens.setStates(true, false, false, false);

            // competition with hub vision configuration
            // Show_Hide_Screens.setStates(true, false,true, false);

            m_setup = new SetUpOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor,
                        m_limelight, m_intake, m_climber, m_trajectory);

            m_autoOi = new SetUpAutoOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor, m_limelight,
                        m_intake, m_climber, m_trajectory, m_rcv2, ssdisp);

            m_preOi = new SetUpPreRoundOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor, m_limelight,
                        m_intake, m_climber, m_trajectory, m_rcv2);

            m_hvis = new HubVisionShuffleboard(m_limelight, m_rcv2, m_vrt, m_turret, m_tilt, m_shooter,
                        m_transport, m_compressor);

            m_llVis = new LLVisionShuffleboard(m_limelight, m_rcv2, m_turret, m_tilt, m_shooter);

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

            new JoystickButton(m_driverController, 1)
                        .whileHeld(new RunActiveIntake(m_intake, m_transport))
                        .whenPressed(new TurnLedsOnOff(m_limelight, false))
                        .whenPressed(new RunLowerRollerIntake(m_transport, m_intake))
                        .whenReleased(new StopActiveIntake(m_intake));

            new JoystickButton(m_driverController, 2)
                        .whenPressed(new RunShooter(m_shooter))
                        .whenPressed(new TurnLedsOnOff(m_limelight, true))
                        .whenPressed(new ShootCargo(m_shooter, m_transport, m_intake));

            new JoystickButton(m_driverController, 6).whenPressed(new SetFrontIntakeActive(m_intake, true));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport))
                        .whenPressed(new TurnLedsOnOff(m_limelight, false))
                        .whenPressed(new StopLowerRoller(m_transport));

            new JoystickButton(m_driverController, 4).whenPressed(new SetFrontIntakeActive(m_intake, false));

            new JoystickButton(m_driverController, 5).whenPressed(new RunShooter(m_shooter))
                        .whenPressed(new TurnLedsOnOff(m_limelight, true));

            new JoystickButton(m_driverController, 7)

                        .whenPressed(new PositionTilt(m_tilt, TiltConstants.TILT_MIN_ANGLE))

                        .whenPressed(new PositionTurret(m_turret, 0));

            new JoystickButton(m_driverController, 8)

                        .whileHeld(new RunCargoOutShooter(m_shooter, m_intake, m_transport));

            new JoystickButton(m_driverController, 9)

                        .whenPressed(new SequentialCommandGroup(
                              new AcquireTarget(m_limelight, m_tilt, m_turret, m_rcv2), 
                              new TimeDelay(0.25), 
                              new PositionTurretToVision(m_turret, m_rcv2)));

            //new JoystickButton(m_driverController, 10)

            new JoystickButton(m_driverController, 11).whileHeld(

                        new RunClimber(m_climber, Pref.getPref("ClimbArmUp")));

            new JoystickButton(m_driverController, 12)

                        .whileHeld(new RunClimber(m_climber, Pref.getPref("ClimbArmDown")))

                        .whenPressed(new PositionTilt(m_tilt, TiltConstants.TILT_MIN_ANGLE))

                        .whenPressed(new PositionTurret(m_turret, 0));

            // close to hub
            driverUpButton.whenPressed(new SetupForShootLocation(m_shooter, m_tilt, m_turret, m_limelight, 0));
            // tarmac line
            driverDownButton.whenPressed(new SetupForShootLocation(m_shooter, m_tilt, m_turret, m_limelight, 1));// tarmac

            driverLeftButton.whenPressed(new TurnLedsOnOff(m_limelight, false));

            driverRightButton.whenPressed(new TurnLedsOnOff(m_limelight, true));

            codriverStart.whileHeld(new RunCargoOutShooter(m_shooter, m_intake, m_transport));

            // codriverX.

            codriverY.whileHeld(getJogTiltCommand(codriverGamepad)).whileHeld(getJogTurretCommand(codriverGamepad))
                        .whenReleased(new TiltWaitForStop(m_tilt)).whenReleased(new TurretWaitForStop(m_turret));

            codriverB.whenPressed(new SetFrontIntakeActive(m_intake, false));

            codriverUpButton.whenPressed(new ChangeShooterSpeed(m_shooter, +100));

            codriverDownButton.whenPressed(new ChangeShooterSpeed(m_shooter, -100));

            codriverRightButton.whenPressed(new ChangeShooterSpeed(m_shooter, +250));

            codriverLeftButton.whenPressed(new ChangeShooterSpeed(m_shooter, -250));

            codriverA.whenPressed(new SetFrontIntakeActive(m_intake, true));

            codriverLeftTrigger.whenPressed(new PositionTurretIncremental(m_turret, 1));

            codriverRightTrigger.whenPressed(new PositionTurretIncremental(m_turret, -1));

            // test allow low shoot speed
            codriverLeftStick.whenPressed(new SetupForShootLocation(m_shooter, m_tilt, m_turret, m_limelight, 3));

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

      public Command getDriveStraightCommand() {
            return new DriveStraightJoystick(m_drive, () -> -m_driverController.getY());

      }

      public Command getJogTurretCommand(XboxController gamepad) {
            return new TurretJog(m_turret, () -> gamepad.getRawAxis(0) / 10, gamepad);
      }

      public Command getJogTiltCommand(XboxController gamepad) {
            return new TiltJog(m_tilt, () -> -gamepad.getRawAxis(1) / 2, gamepad);
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

            return new JogShooter(m_shooter, () -> codriverGamepad.getRawAxis(4));
      }

      public Command getJogShooterVelocityCommand() {
            return new JogShooterVelocity(m_shooter, () -> codriverGamepad.getRawAxis(4));
      }

      public double getThrottle() {
            return (1 - m_driverController.getThrottle()) / 2;
      }

      public void checkCANDevices() {
            m_turret.checkCAN();
            m_tilt.checkCAN();
            m_intake.checkFrontCAN();
            m_intake.checkRearCAN();
            m_shooter.checkCAN();
            m_drive.checkCAN();
            m_transport.checkCAN();
            m_climber.checkCAN();

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
            if (m_tilt.onMinusSoftwareLimit() || m_tilt.onPlusSoftwareLimit() || m_tilt.onMinusHardwareLimit()
                        || m_turret.onPlusSoftwareLimit() || m_turret.onMinusSoftwareLimit()
                        || m_turret.onPlusHardwareLimit() || m_turret.onMinusHardwareLimit()
                        || DriverStation.isDisabled())
                  m_limelight.useVision = false;

      }

}