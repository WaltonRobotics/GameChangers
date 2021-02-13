// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auton.RamseteTrackingCommand;
import frc.robot.commands.teleop.DriveCommand;
import frc.robot.robots.RobotIdentification;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.DioIDs.kRobotId1;
import static frc.robot.Constants.DioIDs.kRobotId2;
import static frc.robot.Constants.SmartDashboardKeys.kLeftVelocityPKey;
import static frc.robot.Constants.SmartDashboardKeys.kRightVelocityPKey;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Drivetrain sDrivetrain;
  public static RobotIdentification sCurrentRobot;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    sCurrentRobot = RobotIdentification.findByInputs(new DigitalInput(kRobotId1).get(), new DigitalInput(kRobotId2).get());
    System.out.println("Current robot is " + sCurrentRobot.name());

    populateShuffleboard();

    sDrivetrain = new Drivetrain();

    CommandScheduler.getInstance().setDefaultCommand(sDrivetrain, new DriveCommand());
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
//    new SequentialCommandGroup(
//            new DriveStraight(6.6),
//            new WaitCommand(2),
//            new DriveStraight(6.5)
//    ).schedule();
    sDrivetrain.setupMotorsAuton();
    sDrivetrain.reset();
    sDrivetrain.resetPose(Paths.GalacticSearchPaths.redA.getInitialPose());

    new RamseteTrackingCommand(Paths.GalacticSearchPaths.redA, true, true).schedule();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
