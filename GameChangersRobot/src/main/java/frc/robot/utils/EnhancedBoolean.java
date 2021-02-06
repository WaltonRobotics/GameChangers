package frc.robot.utils;

public class EnhancedBoolean {

    private boolean value;
    private boolean previousValue;

    public EnhancedBoolean(boolean value) {
        this.value = value;
        previousValue = value;
    }

    public EnhancedBoolean() {
        this(false);
    }

    public boolean get() {
        return value;
    }

    public void set(boolean newValue) {
        previousValue = value;
        value = newValue;
    }

    public boolean isRisingEdge() {
        return value && !previousValue;
    }

    public boolean isFallingEdge() {
        return !value && previousValue;
    }

    public boolean hasChanged() {
        return value != previousValue;
    }

    @Override
    public String toString() {
        return "EnhancedBoolean{" +
                "value=" + value +
                ", previousValue=" + previousValue +
                '}';
    }
}