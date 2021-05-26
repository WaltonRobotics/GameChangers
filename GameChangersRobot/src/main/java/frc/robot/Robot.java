// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auton.AutonFlags;
import frc.robot.auton.AutonRoutine;
import frc.robot.commands.auton.SetIntakeToggle;
import frc.robot.commands.auton.shootingChallenges.InterstellarAccuracyRoutine;
import frc.robot.commands.auton.shootingChallenges.PowerPortRoutine;
import frc.robot.commands.background.*;
import frc.robot.commands.background.driveMode.ArcadeDrive;
import frc.robot.commands.background.driveMode.CurvatureDrive;
import frc.robot.commands.background.driveMode.DriveMode;
import frc.robot.commands.background.driveMode.TankDrive;
import frc.robot.robots.RobotIdentifier;
import frc.robot.subsystems.*;
import frc.robot.utils.DebuggingLog;
import frc.robot.vision.LimelightHelper;

import java.util.Arrays;
import java.util.logging.Level;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.ContextFlags.kIsInfiniteRecharge;
import static frc.robot.Constants.DioIDs.kRobotID1;
import static frc.robot.Constants.DioIDs.kRobotID2;
import static frc.robot.Constants.DriverPreferences.kNormalScaleFactor;
import static frc.robot.Constants.DriverPreferences.kTurboScaleFactor;
import static frc.robot.Constants.Field.kPowerPortScoringZonePose;
import static frc.robot.Constants.Shooter.kDefaultVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.sRunInterstellarRoutineButton;
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
    public static Turret sTurret;
    public static ProMicro sProMicro;
    public static SendableChooser<SequentialCommandGroup> mShootingChallengeChooser;
    public static SendableChooser<DriveMode> sDriveModeChooser;
    private static SendableChooser<AutonRoutine> mAutonChooser;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        sCurrentRobot = RobotIdentifier.findByInputs(new DigitalInput(kRobotID1).get(), new DigitalInput(kRobotID2).get());
        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Current robot is " + sCurrentRobot.name());

        SmartDashboard.putString(kRobotIdentifierKey, sCurrentRobot.name());

        sDrivetrain = new Drivetrain();
        sShooter = new Shooter();
        sIntake = new Intake();
        sConveyor = new Conveyor();
        sTurret = new Turret();
        sProMicro = new ProMicro();

        CommandScheduler.getInstance().setDefaultCommand(sDrivetrain, new DriveCommand());

        // Only instantiate subsystems/autons which require them for Infinite Recharge/Game Changers robots
        // to maintain backwards compatibility with DeepSpace
        if (sCurrentRobot == RobotIdentifier.PRACTICE_GAME_CHANGERS
                || sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS) {
            CommandScheduler.getInstance().setDefaultCommand(sShooter, new ShooterCommand());
//            CommandScheduler.getInstance().setDefaultCommand(sIntake, new IntakeCommand());
            CommandScheduler.getInstance().setDefaultCommand(sConveyor, new ConveyorCommand());

            if (sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS) {
                CommandScheduler.getInstance().setDefaultCommand(sTurret, new TurretCommand());
            }

            CommandScheduler.getInstance().setDefaultCommand(sProMicro, new ProMicroCommand());
        }

        mAutonChooser = new SendableChooser<>();
        Arrays.stream(AutonRoutine.values()).forEach(n -> mAutonChooser.addOption(n.name(), n));
        mAutonChooser.setDefaultOption(DO_NOTHING.name(), DO_NOTHING);
        SmartDashboard.putData("Auton Selector", mAutonChooser);

        mShootingChallengeChooser = new SendableChooser<>();
        mShootingChallengeChooser.setDefaultOption("Do Nothing", new SequentialCommandGroup(
                new SetIntakeToggle(false),
                new InstantCommand(() -> sDrivetrain.resetPose(kPowerPortScoringZonePose))
        ));
//            mShootingChallengeChooser.addOption("Interstellar Accuracy Challenge", new InterstellarAccuracyRoutine());
        mShootingChallengeChooser.addOption("Power Port Challenge", new PowerPortRoutine());

        SmartDashboard.putData("Shooting Challenge Selector", mShootingChallengeChooser);

        populateShuffleboard();

        LimelightHelper.setLEDMode(kIsInTuningMode);

        if (!kIsInfiniteRecharge) {
            sRunInterstellarRoutineButton.whenPressed(new InterstellarAccuracyRoutine());
        }
    }

    private void populateShuffleboard() {
        if (kIsInTuningMode) {
            SmartDashboard.putNumber(kDrivetrainLeftVelocityPKey, sDrivetrain.getLeftVelocityPID().getP());
            SmartDashboard.putNumber(kDrivetrainRightVelocityPKey, sDrivetrain.getRightVelocityPID().getP());
            SmartDashboard.putNumber(kDrivetrainTuningOpenLoopRampRateKey, sDrivetrain.getConfig().kOpenLoopRampRate);
            SmartDashboard.putNumber(kShooterMeasurementPeriodKey, 1);
            SmartDashboard.putNumber(kShooterMeasurementWindowKey, 1);
            SmartDashboard.putNumber(kShooterTuningSetpointRawUnitsKey, kDefaultVelocityRawUnits);
            SmartDashboard.putNumber(kIntakeIntakingDutyCycleKey, sIntake.getConfig().kIntakeDutyCycle);
            SmartDashboard.putNumber(kTurnToAnglePKey, sDrivetrain.getTurnProfiledPID().getP());
            SmartDashboard.putNumber(kTurnToAngleIKey, sDrivetrain.getTurnProfiledPID().getI());
            SmartDashboard.putNumber(kTurnToAngleDKey, sDrivetrain.getTurnProfiledPID().getD());
            SmartDashboard.putNumber(kConveyorBallCountKey, sConveyor.getConfig().kIRSensorFlickeringTimeSeconds);
        }

        SmartDashboard.putNumber(kNormalScaleFactorKey, kNormalScaleFactor);
        SmartDashboard.putNumber(kTurboScaleFactorKey, kTurboScaleFactor);
        SmartDashboard.putNumber(kCurvatureTurnSensitivityKey, kNormalScaleFactor);

        sDriveModeChooser = new SendableChooser<>();
        sDriveModeChooser.setDefaultOption("Tank", new TankDrive());
        sDriveModeChooser.addOption("Curvature", new CurvatureDrive());
        sDriveModeChooser.addOption("Arcade", new ArcadeDrive());
        SmartDashboard.putData("Drive Mode Selector", sDriveModeChooser);
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

//        new SequentialCommandGroup(
//                new ResetPose(sTestTrajectory),
//                new RamseteTrackingCommand(sTestTrajectory, false, true)
//        ).schedule();

//        new DrivetrainCharacterizationRoutine().schedule();

        LimelightHelper.setLEDMode(kIsInTuningMode);

        DebuggingLog.getInstance().getLogger().log(Level.INFO, "Selected autonomous description: "
                + mAutonChooser.getSelected().getDescription());

        mAutonChooser.getSelected().getCommandGroup().schedule();
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
        if (!kIsInfiniteRecharge) {
            mShootingChallengeChooser.getSelected().schedule();
        }

        AutonFlags.getInstance().setIsInAuton(false);

        LimelightHelper.setLEDMode(kIsInTuningMode);
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
