package frc.robot;

public class ElevatorConstants {

    public static final double maxPos = 0; // Maximum elevator height in inches (adjust based on actual mechanism)
    public static final double countsPerInch = 0; // Encoder counts per inch (depends on your encoder)
    public static final double kD = 0;  // Derivative gain (tune this based on your control needs)
    public static final double kI = 0; // Integral gain (often kept low or zero)
    public static final double kP = 0;  // Proportional gain (tune for responsiveness)
    public static final double maxAcceleration = 0; // Maximum acceleration in inches per second squared
    public static final double maxVelocity = 0; // Maximum velocity in inches per second
    public static final int limitSwitchPort = 0; // Digital input port for the limit switch (check wiring)
    public static final int realElevatorID = 14;  // Motor controller ID (keep or change based on setup)
    public static final double downPos = 0; // Position when the elevator is fully down
    public static final double L1 = 1; // Height at L1
    public static final double L2 = 2; // Height at L2
    public static final double L3 = 3; // Height at L3
    public static final double L4 = 4; // Height at L4
    public static final double max_output = 0; // Maximum motor power (usually 1.0 for full power)
    public static final double bottomPos = 0; // Homing position (same as downPos)
    public static final double posTolerance = 0; //// Allowable error in inches. used to check if the elevator is within a small tolerance of the target height. change this
    public static final double kS = 0; // Static friction feedforward (depends on motor characteristics)
    public static final double kG = 0; // Gravity feedforward (compensates for gravity)
    public static final double kV = 0; // Velocity feedforward (depends on your system)
    public static final double minPos = 0; // Minimum setpoint (same as downPos)

}
