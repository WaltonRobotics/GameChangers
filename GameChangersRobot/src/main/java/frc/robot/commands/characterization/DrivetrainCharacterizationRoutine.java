package frc.robot.commands.characterization;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.sDrivetrain;
import static frc.robot.Robot.sIntake;

public class DrivetrainCharacterizationRoutine extends CommandBase {

    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    private final NetworkTableEntry mRotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    private double mPriorAutospeed = 0;
    private final Number[] mNumberArray = new Number[10];

    @Override
    public void initialize() {
        addRequirements(sDrivetrain);
        addRequirements(sIntake);

        sDrivetrain.setupControllersAuton();
        sDrivetrain.reset();

        sIntake.setDeployed(true);
    }

    @Override
    public void execute() {
        System.out.println("Hello");
        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double leftPosition = sDrivetrain.getLeftPositionMeters();
        double leftRate = sDrivetrain.getLeftVelocityMetersPerSec();

        double rightPosition = sDrivetrain.getRightPositionMeters();
        double rightRate = sDrivetrain.getRightVelocityMetersPerSec();

        double battery = RobotController.getBatteryVoltage();
        double motorVolts = battery * Math.abs(mPriorAutospeed);

        double leftMotorVolts = motorVolts;
        double rightMotorVolts = motorVolts;

        // Retrieve the commanded speed from NetworkTables
        double autospeed = mAutoSpeedEntry.getDouble(0);
        mPriorAutospeed = autospeed;

        // command motors to do things
        sDrivetrain.setDutyCycles((mRotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);

        // send telemetry data array back to NT
        mNumberArray[0] = now;
        mNumberArray[1] = battery;
        mNumberArray[2] = autospeed;
        mNumberArray[3] = leftMotorVolts;
        mNumberArray[4] = rightMotorVolts;
        mNumberArray[5] = leftPosition;
        mNumberArray[6] = rightPosition;
        mNumberArray[7] = leftRate;
        mNumberArray[8] = rightRate;
        mNumberArray[9] = sDrivetrain.getHeading().getRadians();

        mTelemetryEntry.setNumberArray(mNumberArray);
    }

    @Override
    public void end(boolean interrupted) {
        sDrivetrain.setDutyCycles(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
