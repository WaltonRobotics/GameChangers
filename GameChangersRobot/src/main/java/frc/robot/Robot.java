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
import frc.robot.commands.teleop.ConveyorCommand;
import frc.robot.commands.teleop.DriveCommand;
import frc.robot.commands.teleop.IntakeCommand;
import frc.robot.commands.teleop.ShooterCommand;
import frc.robot.robots.RobotIdentifier;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.Arrays;

import static frc.robot.Constants.DioIDs.kRobotID1;
import static frc.robot.Constants.DioIDs.kRobotID2;
import static frc.robot.Constants.SmartDashboardKeys.kLeftVelocityPKey;
import static frc.robot.Constants.SmartDashboardKeys.kRightVelocityPKey;
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
    private static SendableChooser<AutonRoutine> autonChooser;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        sCurrentRobot = RobotIdentifier.findByInputs(new DigitalInput(kRobotID1).get(), new DigitalInput(kRobotID2).get());
        System.out.println("Current robot is " + sCurrentRobot.name());

        populateShuffleboard();

        sDrivetrain = new Drivetrain();
        sShooter = new Shooter();
        sIntake = new Intake();
        sConveyor = new Conveyor();

        CommandScheduler.getInstance().setDefaultCommand(sDrivetrain, new DriveCommand());
        CommandScheduler.getInstance().setDefaultCommand(sShooter, new ShooterCommand());
        CommandScheduler.getInstance().setDefaultCommand(sIntake, new IntakeCommand());
        CommandScheduler.getInstance().setDefaultCommand(sConveyor, new ConveyorCommand());

        CommandScheduler.getInstance().setDefaultCommand(sDrivetrain, new DriveCommand());
        autonChooser = new SendableChooser<>();
        Arrays.stream(AutonRoutine.values()).forEach(n -> autonChooser.addOption(n.name(), n));
        autonChooser.setDefaultOption(DO_NOTHING.name(), DO_NOTHING);
        SmartDashboard.putData("Auton Selector", autonChooser);
    }

    private void populateShuffleboard() {
        SmartDashboard.putNumber(kLeftVelocityPKey, sCurrentRobot.getCurrentRobot().getDrivetrainLeftVelocityPID().getP());
        SmartDashboard.putNumber(kRightVelocityPKey, sCurrentRobot.getCurrentRobot().getDrivetrainRightVelocityPID().getP());
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
        sDrivetrain.setupControllersAuton();

        AutonFlags.getInstance().setIsInAuton(true);

        autonChooser.getSelected().getCommandGroup().schedule();
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
