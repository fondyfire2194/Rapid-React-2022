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
import frc.robot.Constants.TiltConstants;
import frc.robot.OI.HubVisionShuffleboard;
import frc.robot.OI.LLVisionShuffleboard;
import frc.robot.OI.SetUpAutoOI;
import frc.robot.OI.SetUpOI;
import frc.robot.OI.SetUpPreRoundOI;
import frc.robot.OI.Show_Hide_Screens;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.Vision.LimelightControlMode.StreamType;
import frc.robot.Vision.RawContoursV2;
import frc.robot.Vision.VisionReferenceTarget;
import frc.robot.commands.AutoCommands.Common.SetShootPositionSpeedTilt;
import frc.robot.commands.CargoTransport.RunLowerRoller;
import frc.robot.commands.CargoTransport.RunLowerRollerIntake;
import frc.robot.commands.CargoTransport.StopLowerRoller;
import frc.robot.commands.Climber.RunClimber;
import frc.robot.commands.Intakes.ActiveIntakeArmLower;
import frc.robot.commands.Intakes.ActiveIntakeArmRaise;
import frc.robot.commands.Intakes.ReverseActiveIntakeMotor;
import frc.robot.commands.Intakes.RunActiveIntake;
import frc.robot.commands.Intakes.RunCargoOutShooter;
import frc.robot.commands.Intakes.SetFrontIntakeActive;
import frc.robot.commands.Intakes.StopActiveIntake;
import frc.robot.commands.Intakes.StopIntakeMotors;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.Shooter.ChangeShooterSpeed;
import frc.robot.commands.Shooter.JogShooter;
import frc.robot.commands.Shooter.JogShooterVelocity;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.Shooter.RunTopRoller;
import frc.robot.commands.Shooter.ShootCargo;
//import frc.robot.commands.Shooter.ShootTwoCargo;
import frc.robot.commands.Shooter.StopShoot;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Shooter.StopTopRoller;
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
  //    public final XboxController setupGamepad = new XboxController(OIConstants.kSetupControllerPort);

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

      // Setup gamepad XBox 3
      // JoystickButton setupA = new JoystickButton(setupGamepad, 1);
      // JoystickButton setupB = new JoystickButton(setupGamepad, 2);
      // JoystickButton setupX = new JoystickButton(setupGamepad, 3);
      // JoystickButton setupY = new JoystickButton(setupGamepad, 4);
      // JoystickButton setupLeftTrigger = new JoystickButton(setupGamepad, 5);
      // JoystickButton setupRightTrigger = new JoystickButton(setupGamepad, 6);

      // JoystickButton setupBack = new JoystickButton(setupGamepad, 7);
      // JoystickButton setupStart = new JoystickButton(setupGamepad, 8);

      // JoystickButton setupLeftStick = new JoystickButton(setupGamepad, 11);
      // JoystickButton setupRightStick = new JoystickButton(setupGamepad, 12);

      // public POVButton setupUpButton = new POVButton(setupGamepad, 0);
      // public POVButton setupRightButton = new POVButton(setupGamepad, 90);
      // public POVButton setupDownButton = new POVButton(setupGamepad, 180);
      // public POVButton setupLeftButton = new POVButton(setupGamepad, 270);

      public POVButton driverUpButton = new POVButton(m_driverController, 0);
      public POVButton driverRightButton = new POVButton(m_driverController, 90);
      public POVButton driverDownButton = new POVButton(m_driverController, 180);
      public POVButton driverLeftButton = new POVButton(m_driverController, 270);
      public boolean isMatch;

      /**
       * The container for the robot. Contains subsysems, OI devices, and commands.
       */
      public RobotContainer() {
            isMatch = Robot.getFMSConnected();
            // Preferences.removeAll();
            // Pref.deleteUnused();
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

            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);
            m_limelight.useVision = false;

            m_rcv2 = new RawContoursV2(m_limelight);

            m_vrt = new VisionReferenceTarget(m_rcv2, m_limelight);

            m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

            m_trajectory = new FondyFireTrajectory(m_drive);

            Show_Hide_Screens.setStates(true, false, false);

            m_setup = new SetUpOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor,
                        m_limelight, m_intake, m_climber, m_trajectory);

            m_autoOi = new SetUpAutoOI(m_turret, m_tilt, m_drive, m_shooter, m_transport, m_compressor, m_limelight,
                        m_intake, m_climber, m_trajectory, m_rcv2);

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
                        .whenPressed(new RunLowerRollerIntake(m_transport))
                        .whenReleased(new StopActiveIntake(m_intake));

            new JoystickButton(m_driverController, 2)
                        .whenPressed(new ShootCargo(m_shooter, m_transport, m_intake, false));

            new JoystickButton(m_driverController, 6).whenPressed(new SetFrontIntakeActive(m_intake, true));

            new JoystickButton(m_driverController, 3).whenPressed(new StopShoot(m_shooter, m_transport))
                        .whenPressed(new StopLowerRoller(m_transport));

            new JoystickButton(m_driverController, 4).whenPressed(new SetFrontIntakeActive(m_intake, false));

            new JoystickButton(m_driverController, 5).whenPressed(new RunShooter(m_shooter));

            new JoystickButton(m_driverController, 7)

                        .whenPressed(new PositionTilt(m_tilt, TiltConstants.TILT_MIN_ANGLE))

                        .whenPressed(new PositionTurret(m_turret, 0));

            new JoystickButton(m_driverController, 8)

                        .whileHeld(new RunCargoOutShooter(m_shooter, m_intake, m_transport));

            // new JoystickButton(m_driverController, 10).

            // new JoystickButton(m_driverController, 9)

            // new JoystickButton(m_driverController, 11)

            //
            // new JoystickButton(m_driverController, 12)

            driverUpButton.whenPressed(new SetShootPositionSpeedTilt(m_shooter, m_tilt, m_limelight, 0));// tarmac line upper hub

            driverDownButton.whenPressed(new SetShootPositionSpeedTilt(m_shooter, m_tilt, m_limelight, 1));// tarmac line lower hub

            driverLeftButton.whenPressed(new SetShootPositionSpeedTilt(m_shooter, m_tilt, m_limelight, 2));// against hub upper

            driverRightButton.whenPressed(new SetShootPositionSpeedTilt(m_shooter, m_tilt, m_limelight, 3));// against hub lower

            /**
             * co driver can empty robot
             */

            codriverStart.whileHeld(new RunCargoOutShooter(m_shooter, m_intake, m_transport));

            codriverX.whileHeld(getJogTiltVelocityCommand(codriverGamepad))
                        .whileHeld(getJogTurretVelocityCommand(codriverGamepad)).whenReleased(new TiltWaitForStop(m_tilt))
                        .whenReleased(new TurretWaitForStop(m_turret));

            codriverY.whileHeld(getJogTiltCommand(codriverGamepad)).whileHeld(getJogTurretCommand(codriverGamepad))
                        .whenReleased(new TiltWaitForStop(m_tilt)).whenReleased(new TurretWaitForStop(m_turret));

            codriverB.whenPressed(new SetFrontIntakeActive(m_intake, false));
                        
            codriverUpButton.whenPressed(new ChangeShooterSpeed(m_shooter, +100));
            codriverDownButton.whenPressed(new ChangeShooterSpeed(m_shooter, -100));
            codriverRightButton.whenPressed(new ChangeShooterSpeed(m_shooter, +250));
            codriverLeftButton.whenPressed(new ChangeShooterSpeed(m_shooter, -250));

            codriverA.whileHeld(new SetFrontIntakeActive(m_intake, true));

            // // climber

            // codriverLeftTrigger

            // .whenPressed(() -> m_climber.unlockRatchet())

            // .whileHeld(getRunClimberMotorCommand(codriverGamepad))

            // .whenReleased(() -> m_climber.stopMotor())

            // .whenReleased(() -> m_climber.lockRatchet());

            // codriverBack.whenPressed(new ClimberArm(m_climber, true));

         
      
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

            return new JogShooter(m_shooter, () -> codriverGamepad.getRawAxis(4));
      }

      public Command getJogShooterVelocityCommand() {
            return new JogShooterVelocity(m_shooter, () -> codriverGamepad.getRawAxis(4));
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
            m_intake.checkFrontCAN();
            m_intake.checkRearCAN();
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