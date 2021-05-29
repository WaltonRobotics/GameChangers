/**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 */

package dc;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Supplier;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon

public class Robot extends TimedRobot {

    // Robot identification
    private static final String robotIdentifierKey = "Robot Identifier";
    private final boolean input1;
    private final boolean input2;

    private final double encoderConstant;
    // Only for COMP_DEEP_SPACE
    private final Solenoid gearShiftSolenoid = new Solenoid(0);
    private Joystick leftStick;
    private Joystick rightStick;
    private DifferentialDrive drive;
    private CANSparkMax leftWheelsMaster;
    private CANSparkMax rightWheelsMaster;
    private Supplier<Double> leftEncoderPosition;
    private Supplier<Double> leftEncoderRate;
    private Supplier<Double> rightEncoderPosition;
    private Supplier<Double> rightEncoderRate;
    private Supplier<Double> gyroAngleRadians;

    private final NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
    private final NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
    private final NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

    private String data = "";

    private int counter = 0;
    private double startTime = 0;
    private double priorAutospeed = 0;

    private final double[] numberArray = new double[10];
    private final ArrayList<Double> entries = new ArrayList<Double>();

    public Robot() {
        super(.005);

        input1 = new DigitalInput(8).get();
        input2 = new DigitalInput(9).get();

        if (!input1 && input2) {
            // COMP_DEEP_SPACE
            SmartDashboard.putString(robotIdentifierKey, "COMP_DEEP_SPACE");
            encoderConstant = 1.0 / 36.797;
        } else if (input1 && !input2) {
            // PRACTICE_GAME_CHANGERS
            SmartDashboard.putString(robotIdentifierKey, "PRACTICE_GAME_CHANGERS");
            encoderConstant = 0.05984734;
        } else {
            // COMP_GAME_CHANGERS
            SmartDashboard.putString(robotIdentifierKey, "COMP_GAME_CHANGERS");
            encoderConstant = 1.0 / 17.011875;
        }

        LiveWindow.disableAllTelemetry();
    }
    // methods to create and setup motors (reduce redundancy)
    public CANSparkMax setupCANSparkMax(int port, Sides side, boolean inverted) {
        // create new motor and set neutral modes (if needed)
        // setup Brushless spark
        CANSparkMax motor = new CANSparkMax(port, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        motor.enableVoltageCompensation(12.0);

        // setup encoder if motor isn't a follower
        if (side != Sides.FOLLOWER) {


            CANEncoder encoder = motor.getEncoder();


            switch (side) {
                // setup encoder and data collecting methods

                case RIGHT:
                    // set right side methods = encoder methods


                    rightEncoderPosition = ()
                            -> encoder.getPosition() * -encoderConstant;
                    rightEncoderRate = ()
                            -> encoder.getVelocity() * -encoderConstant / 60.;

                    break;
                case LEFT:
                    leftEncoderPosition = ()
                            -> encoder.getPosition() * encoderConstant;
                    leftEncoderRate = ()
                            -> encoder.getVelocity() * encoderConstant / 60.;

                    break;
                default:
                    // probably do nothing
                    break;

            }

        }


        return motor;

    }

    @Override
    public void robotInit() {
        if (!isReal()) SmartDashboard.putData(new SimEnabler());

        leftStick = new Joystick(0);
        rightStick = new Joystick(1);

        // create left motor
        leftWheelsMaster = setupCANSparkMax(3, Sides.LEFT, true);

        CANSparkMax leftFollowerID4 = setupCANSparkMax(4, Sides.FOLLOWER, false);
        leftFollowerID4.follow(leftWheelsMaster, false);

        rightWheelsMaster = setupCANSparkMax(1, Sides.RIGHT, true);
        CANSparkMax rightFollowerID2 = setupCANSparkMax(2, Sides.FOLLOWER, false);
        rightFollowerID2.follow(rightWheelsMaster, false);
        drive = new DifferentialDrive(leftWheelsMaster, rightWheelsMaster);
        drive.setDeadband(0);

        //
        // Configure gyro
        //

        // Note that the angle from the NavX and all implementors of WPILib Gyro
        // must be negated because getAngle returns a clockwise positive angle
        AHRS navx = new AHRS(SPI.Port.kMXP);
        gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());

        // Shift down for COMP_DEEP_SPACE
        if (!input1 && input2) {
            gearShiftSolenoid.set(false);
        }

        // Set the update rate instead of using flush because of a ntcore bug
        // -> probably don't want to do this on a robot in competition
        NetworkTableInstance.getDefault().setUpdateRate(0.010);
    }

    @Override
    public void disabledInit() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        System.out.println("Robot disabled");
        drive.tankDrive(0, 0);
        // data processing step
        data = entries.toString();
        data = data.substring(1, data.length() - 1) + ", ";
        telemetryEntry.setString(data);
        entries.clear();
        System.out.println("Robot disabled");
        System.out.println("Collected : " + counter + " in " + elapsedTime + " seconds");
        data = "";
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void robotPeriodic() {
        // feedback for users, but not used by the control program
        SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
        SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
        SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
        SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
        SmartDashboard.putNumber("heading", gyroAngleRadians.get());
    }

    @Override
    public void teleopInit() {
        System.out.println("Robot in operator control mode");
    }

    @Override
    public void teleopPeriodic() {
        drive.tankDrive(-leftStick.getY(), -rightStick.getY());
    }

    @Override
    public void autonomousInit() {
        System.out.println("Robot in autonomous mode");
        startTime = Timer.getFPGATimestamp();
        counter = 0;
    }

    /**
     * If you wish to just use your own robot program to use with the data logging
     * program, you only need to copy/paste the logic below into your code and
     * ensure it gets called periodically in autonomous mode
     *
     * Additionally, you need to set NetworkTables update rate to 10ms using the
     * setUpdateRate call.
     */
    @Override
    public void autonomousPeriodic() {

        // Retrieve values to send back before telling the motors to do something
        double now = Timer.getFPGATimestamp();

        double leftPosition = leftEncoderPosition.get();
        double leftRate = leftEncoderRate.get();

        double rightPosition = rightEncoderPosition.get();
        double rightRate = rightEncoderRate.get();

        double battery = RobotController.getBatteryVoltage();

        // Effectively the same as 12.0 * priorAutospeed since voltage compensation is enabled
        double leftMotorVolts = Math.abs(leftWheelsMaster.getBusVoltage() * leftWheelsMaster.getAppliedOutput());
        double rightMotorVolts = Math.abs(rightWheelsMaster.getBusVoltage() * rightWheelsMaster.getAppliedOutput());

        // Retrieve the commanded speed from NetworkTables
        double autospeed = autoSpeedEntry.getDouble(0);
        priorAutospeed = autospeed;

        // command motors to do things
        drive.tankDrive(
                (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
                false
        );

        numberArray[0] = now;
        numberArray[1] = battery;
        numberArray[2] = autospeed;
        numberArray[3] = leftMotorVolts;
        numberArray[4] = rightMotorVolts;
        numberArray[5] = leftPosition;
        numberArray[6] = rightPosition;
        numberArray[7] = leftRate;
        numberArray[8] = rightRate;
        numberArray[9] = gyroAngleRadians.get();

        // Add data to a string that is uploaded to NT
        for (double num : numberArray) {
            entries.add(num);
        }
        counter++;
    }

    public enum Sides {
        LEFT,
        RIGHT,
        FOLLOWER
    }
}
