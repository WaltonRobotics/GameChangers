//package frc.robot.commands.characterization;
//
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.robots.RobotIdentifier;
//
//import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;
//import static frc.robot.Robot.*;
//
//// Use characterization project for faster dt and more precise data
//@Deprecated
//public class DrivetrainCharacterizationRoutine extends CommandBase {
//
//    private final NetworkTableEntry mAutoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
//    private final NetworkTableEntry mTelemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
//    private final NetworkTableEntry mRotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");
//    private final Number[] mNumberArray = new Number[10];
//
//    public DrivetrainCharacterizationRoutine() {
//        addRequirements(sDrivetrain);
//        addRequirements(sIntake);
//    }
//
//    @Override
//    public void initialize() {
//        sDrivetrain.configureControllersAuton();
//        sDrivetrain.reset();
//
//        if (sCurrentRobot == RobotIdentifier.COMP_GAME_CHANGERS
//                || sCurrentRobot == RobotIdentifier.PRACTICE_GAME_CHANGERS) {
//            sIntake.setDeployed(false);
//        }
//    }
//
//    @Override
//    public void execute() {
//        // Retrieve values to send back before telling the motors to do something
//        double now = Timer.getFPGATimestamp();
//
//        double leftPosition = sDrivetrain.getLeftPositionMeters();
//        double leftRate = sDrivetrain.getLeftVelocityMetersPerSec();
//
//        double rightPosition = sDrivetrain.getRightPositionMeters();
//        double rightRate = sDrivetrain.getRightVelocityMetersPerSec();
//
//        double battery = getBatteryVoltage();
//
//        double leftMotorVolts = sDrivetrain.getLeftVoltage();
//        double rightMotorVolts = sDrivetrain.getRightVoltage();
//
//        // Retrieve the commanded speed from NetworkTables
//        double autospeed = mAutoSpeedEntry.getDouble(0);
//
//        // command motors to do things
//        sDrivetrain.setDutyCycles((mRotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed);
//
//        // send telemetry data array back to NT
//        mNumberArray[0] = now;
//        mNumberArray[1] = battery;
//        mNumberArray[2] = autospeed;
//        mNumberArray[3] = leftMotorVolts;
//        mNumberArray[4] = rightMotorVolts;
//        mNumberArray[5] = leftPosition;
//        mNumberArray[6] = rightPosition;
//        mNumberArray[7] = leftRate;
//        mNumberArray[8] = rightRate;
//        mNumberArray[9] = sDrivetrain.getHeading().getRadians();
//
//        mTelemetryEntry.setNumberArray(mNumberArray);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        sDrivetrain.setDutyCycles(0, 0);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
//}
