// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.AutonFlags;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.background.*;
import frc.robot.robots.RobotIdentifier;
import frc.robot.subsystems.*;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;
import frc.robot.vision.PixyCamHelper;

import java.util.Arrays;
import java.util.logging.Level;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DioIDs.kRobotID1;
import static frc.robot.Constants.DioIDs.kRobotID2;
import static frc.robot.Constants.Shooter.kDefaultVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.auton.AutonRoutine.DO_NOTHING;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

    public static RobotIdentifier sCurrentRobot;
    public static Drivetrain sDrivetrain;
    public static Shooter sShooter;
    public static Intake sIntake;
    public static Conveyor sConveyor;
    public static ProMicro sProMicro;

    private static SendableChooser<AutonRoutine> mAutonChooser;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        sCurrentRobot = RobotIdentifier.findByInputs(new DigitalInput(kRobotID1).get(), new DigitalInput(kRobotID2).get());
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Current robot is " + sCurrentRobot.name());

        populateShuffleboard();

        sDrivetrain = new Drivetrain();
        CommandScheduler.getInstance().setDefaultCommand(sDrivetrain, new DriveCommand());

        // Only instantiate subsystems/autons which require them for Infinite Recharge/Game Changers robots
        // to maintain backwards compatibility with DeepSpace
        if (sCurrentRobot == RobotIdentifier.PRACTICE_GAME_CHANGERS
                || sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS) {
            sShooter = new Shooter();
            CommandScheduler.getInstance().setDefaultCommand(sShooter, new ShooterCommand());

            sIntake = new Intake();
            CommandScheduler.getInstance().setDefaultCommand(sIntake, new IntakeCommand());

            sConveyor = new Conveyor();
            CommandScheduler.getInstance().setDefaultCommand(sConveyor, new ConveyorCommand());

            sProMicro = new ProMicro();
            CommandScheduler.getInstance().setDefaultCommand(sProMicro, new ProMicroCommand());

            mAutonChooser = new SendableChooser<>();
            Arrays.stream(AutonRoutine.values()).forEach(n -> mAutonChooser.addOption(n.name(), n));
            mAutonChooser.setDefaultOption(DO_NOTHING.name(), DO_NOTHING);
            SmartDashboard.putData("Auton Selector", mAutonChooser);
        }

        LimelightHelper.setLEDMode(false);
    }

    private void populateShuffleboard() {
        if (kIsInTuningMode) {
            SmartDashboard.putNumber(kDrivetrainLeftVelocityPKey, sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP());
            SmartDashboard.putNumber(kDrivetrainRightVelocityPKey, sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP());
            SmartDashboard.putNumber(kShooterMeasurementPeriodKey, 1);
            SmartDashboard.putNumber(kShooterMeasurementWindowKey, 1);
            SmartDashboard.putNumber(kShooterTuningSetpointRawUnitsKey, kDefaultVelocityRawUnits);
            SmartDashboard.putNumber(kAutoAlignTurnPKey, sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getP());
            SmartDashboard.putNumber(kAutoAlignTurnIKey, sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getI());
            SmartDashboard.putNumber(kAutoAlignTurnDKey, sCurrentRobot.getCurrentRobot().getDrivetrainTurnPID().getD());
        }
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */

    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

        SmartDashboard.putString("PixyCam Galactic Search Determination",
                PixyCamHelper.getGalacticSearchDetermination().name());
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        sDrivetrain.configureControllersAuton();

        AutonFlags.getInstance().setIsInAuton(true);

        LimelightHelper.setLEDMode(false);

        // Auton routines do not work with DeepSpace robots due to subsystem requirements
        if (sCurrentRobot == RobotIdentifier.PRACTICE_GAME_CHANGERS
                || sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS) {
            DebuggingLog.getInstance().getLogger().log(Level.INFO, "Selected autonomous description: "
                    + mAutonChooser.getSelected().getDescription());

            mAutonChooser.getSelected().getCommandGroup().schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    /**
     * This function is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        AutonFlags.getInstance().setIsInAuton(false);

        LimelightHelper.setLEDMode(false);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    /**
     * This function is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {

    }

    /**
     * This function is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
    }

    /**
     * This function is called once when test mode is enabled.
     */
    @Override
    public void testInit() {

    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
