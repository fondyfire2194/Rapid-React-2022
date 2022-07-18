/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PipelinesConstants;
import frc.robot.OI.TrajTestOI;
import frc.robot.Vision.LimeLight;
import frc.robot.Vision.LimelightControlMode.CamMode;
import frc.robot.Vision.LimelightControlMode.LedMode;
import frc.robot.Vision.LimelightControlMode.StreamType;
import frc.robot.commands.RobotDrive.AccelVolts;
import frc.robot.commands.RobotDrive.ArcadeDrive;
import frc.robot.commands.RobotDrive.DriveStraightJoystick;
import frc.robot.commands.RobotDrive.RunAtVolts;
import frc.robot.commands.RobotDrive.RunFeedForward;
import frc.robot.commands.RobotDrive.TrajSimMPS;
import frc.robot.commands.RobotDrive.TrajSimVolts;
import frc.robot.subsystems.RevDrivetrain;
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

  
      public static boolean autoSelected;



 //     public SetUpAutoOI m_autoOi;

     

 
      public TrajTestOI ttoi;

      // public Show_Hide_Screens m_sh;

      public LimeLight m_limelight;

      public Compressor m_compressor;

      public FondyFireTrajectory m_trajectory;

      // public ShootSequenceDisplay ssdisp;

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
          
            m_limelight = new LimeLight();

           
            if (RobotBase.isSimulation())

                  m_drive.setDefaultCommand(getArcadeDriveCommandSim());

            else
                  m_drive.setDefaultCommand(getArcadeDriveCommand());

           
            m_limelight.setCamMode(CamMode.kvision);
            m_limelight.setLEDMode(LedMode.kpipeLine);
            m_limelight.setStream((StreamType.kStandard));

            m_limelight.setPipeline(PipelinesConstants.ledsOffPipeline);
            m_limelight.setHorizontalOffset(0);
            m_limelight.useVision = false;

            // m_rcv2 = new RawContoursV2(m_limelight);

            // m_vrt = new VisionReferenceTarget(m_rcv2, m_limelight);

            m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

            m_trajectory = new FondyFireTrajectory(m_drive);
            // ssdisp = new ShootSequenceDisplay(m_transport, m_shooter, m_intake);
            ttoi = new TrajTestOI(m_drive, m_trajectory);
            // test configuration
            // Show_Hide_Screens.setStates(false, false,true);
            // test configuration with vision
            // Show_Hide_Screens.setStates(false, true, true);

          
           
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
                        .whileHeld(new RunAtVolts(m_drive, false));
           

            new JoystickButton(m_driverController, 2)
                        .whileHeld(new AccelVolts(m_drive));

             new JoystickButton(m_driverController, 6)
             .whileHeld(new RunFeedForward(m_drive, false));

            new JoystickButton(m_driverController, 3)
                        .whileHeld(new TrajSimVolts(m_drive, false));

            new JoystickButton(m_driverController, 4).whileHeld(new RunFeedForward(m_drive, false));

            new JoystickButton(m_driverController, 5)
                        .whileHeld(new TrajSimVolts(m_drive, true));

            new JoystickButton(m_driverController, 7)
                        .whileHeld(new TrajSimMPS(m_drive, false));

            new JoystickButton(m_driverController, 8)

                        .whileHeld(new TrajSimMPS(m_drive, true));

            // new JoystickButton(m_driverController, 9).whenPressed(

                    

            // new JoystickButton(m_driverController, 10).whenPressed(

                       

            // new JoystickButton(m_driverController, 11).whileHeld(

                    

            // new JoystickButton(m_driverController, 12)

                        

            // // close to hub
            // driverUpButton.whenPressed
            // // tarmac line
            // driverDownButton.whenPressed

            // driverLeftButton.whenPressed

                        

            // driverRightButton

                     
            // co driver gamepad

            // codriverStart.whenPressed

            // codriverX.whileHeld

            // codriverY.whileHeld

            // codriverB.whenPressed

            // codriverUpButton.whenPressed

            // codriverDownButton.whenPressed

            // codriverRightButton.whenPressed

            // codriverLeftButton.whenPressed

            // codriverA.whenPressed

            // codriverLeftTrigger.whenPressed

            // codriverRightTrigger.whenPressed

            // // test allow low shoot speed
            // codriverLeftStick.whenPressed

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

      public Command getArcadeDriveCommandSim() {
            return new ArcadeDrive(m_drive, () -> -codriverGamepad.getRawAxis(3) / 2,
                        () -> codriverGamepad.getRawAxis(2) * .75);
      }

      public Command getDriveStraightCommand() {
            return new DriveStraightJoystick(m_drive, () -> -m_driverController.getY());

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

      // public Command getJogTurretVelocityCommand() {

      //       return new TurretJogVelocity(m_turret, () -> codriverGamepad.getRawAxis(0) / 2);
      // }

      // public Command getJogTiltVelocityCommand() {

      //       return new TiltJogVelocity(m_tilt, () -> -codriverGamepad.getRawAxis(1) / 2);
      // }

      // public Command getJogShooterCommand() {

      //       return new JogShooter(m_shooter, () -> codriverGamepad.getRawAxis(3));
      // }

      // public Command getJogShooterVelocityCommand() {
      //       return new JogShooterVelocity(m_shooter, () -> codriverGamepad.getRawAxis(4));
      // }

      public double getThrottle() {
            return (1 - m_driverController.getThrottle()) / 2;
      }

      public void checkCANDevices() {
          
            m_drive.checkCAN();
           

      }

      public double[] getPDPInfo() {
            double temp[] = { 0, 0, 0, 0, 0 };
            // temp[0] = m_shooter.getBatteryVoltage();
            // temp[1] = m_shooter.getTemperature();
            // temp[2] = m_shooter.getTotalEnergy() / 3600;
            // temp[3] = m_shooter.getTotalPower();
            return temp;

      }

      // public void checkLimits() {
      //       if (m_tilt.onMinusSoftwareLimit() || m_tilt.onPlusSoftwareLimit() || m_tilt.onMinusHardwareLimit()
      //                   || m_turret.onPlusSoftwareLimit() || m_turret.onMinusSoftwareLimit()
      //                   || m_turret.onPlusHardwareLimit() || m_turret.onMinusHardwareLimit()
      //                   || DriverStation.isDisabled())
      //             m_limelight.useVision = false;

      // }

}
