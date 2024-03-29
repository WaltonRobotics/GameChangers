package frc.robot.robots;

/**
 * Uses the DIO ports on the rio to identify the current robot.
 */

public enum RobotIdentifier {

    COMP_DEEP_SPACE(false, true, new CompDeepSpace()),
    PRACTICE_GAME_CHANGERS(true, false, new PracticeGameChangers()),
    COMP_GAME_CHANGERS(true, true, new CompGameChangers());

    private final boolean input1;
    private final boolean input2;

    WaltRobot currentRobot;

    RobotIdentifier(boolean input1, boolean input2, WaltRobot robot) {
        this.input1 = input1;
        this.input2 = input2;
        this.currentRobot = robot;
    }

    public static RobotIdentifier findByInputs(boolean input1, boolean input2) {
        for (RobotIdentifier i : values()) {
            if (i.input1 == input1 && i.input2 == input2) {
                return i;
            }
        }

        return COMP_GAME_CHANGERS;
    }

    public WaltRobot getCurrentRobot() {
        return currentRobot;
    }

}