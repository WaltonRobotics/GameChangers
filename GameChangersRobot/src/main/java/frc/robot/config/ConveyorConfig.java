package frc.robot.config;

public class ConveyorConfig {

    // The time that the IR sensors flicker randomly after changing states
    public boolean kIsFrontConveyorControllerInverted;
    public boolean kIsBackConveyorControllerInverted;
    public double kIRSensorFlickeringTimeSeconds;
    public double kNudgeTimeSeconds;
    public double kFrontConveyorNudgeVoltage;
    public double kBackConveyorNudgeVoltage;
    public double kFrontConveyorFeedVoltage;
    public double kBackConveyorFeedVoltage;
    public double kFrontConveyorIntakeDutyCycle;
    public double kBackConveyorIntakeDutyCycle;
    public double kFrontConveyorOuttakeDutyCycle;
    public double kBackConveyorOuttakeDutyCycle;

}
