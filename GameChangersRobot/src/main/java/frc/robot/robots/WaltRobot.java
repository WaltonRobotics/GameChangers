package frc.robot.robots;

import frc.robot.config.*;

/* Generic interface for all Walton robots and their characteristics. */
public interface WaltRobot {

    void populateShooterInterpolationMethods();

    DrivetrainConfig getDrivetrainConfig();

    ShooterConfig getShooterConfig();

    IntakeConfig getIntakeConfig();

    ConveyorConfig getConveyorConfig();

    TurretConfig getTurretConfig();

}
