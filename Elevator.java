package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor;
    
    private final RelativeEncoder encoder;
    private final DigitalInput bottomLimit;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State currentState;
    private final TrapezoidProfile profile;

    private ElevatorPosition currentTarget = ElevatorPosition.DOWN;
    private boolean isHomed = false;
    private double setpoint = 0.0; 
    SparkMaxConfig resetConfig = new SparkMaxConfig();
    double currentPos;

    public enum ElevatorPosition {
        DOWN(ElevatorConstants.downPos),
        POSITION_1(ElevatorConstants.L1),
        POSITION_2(ElevatorConstants.L2),
        POSITION_3(ElevatorConstants.L3),
        POSITION_4(ElevatorConstants.L4);

        public final double positionInches;
        
        ElevatorPosition(double positionInches) {
            this.positionInches = positionInches;
        }
    }

    public Elevator() {
        elevatorMotor = new SparkMax(ElevatorConstants.realElevatorID, MotorType.kBrushless);
      

        
              encoder = elevatorMotor.getEncoder();
        bottomLimit = new DigitalInput(ElevatorConstants.limitSwitchPort);

        resetConfig.idleMode(IdleMode.kBrake);
        resetConfig.smartCurrentLimit(40);
        resetConfig.voltageCompensation(12.0);

        constraints = new TrapezoidProfile.Constraints(
            ElevatorConstants.maxVelocity,
            ElevatorConstants.maxAcceleration
        );
        
        pidController = new PIDController(
            ElevatorConstants.kP,
            ElevatorConstants.kI,
            ElevatorConstants.kD
        );
        
        pidController.setTolerance(0.5); // 0.5 inches position tolerance
        
        // Initialize states and profile
        currentState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
        
        configureMotors();
    }

    private void configureMotors() {
        // Primary motor configuration
        elevatorMotor.configure(resetConfig, ResetMode.kResetSafeParameters, null);
      
    }

    @Override
    public void periodic() {

        currentPos = encoder.getPosition() / ElevatorConstants.countsPerInch;
        
        // Calculate the next state and update current state
        currentState = profile.calculate(0.020, currentState, goalState); // 20ms control loop

        if (bottomLimit.get()) {
            handleBottomLimit();
        }

        if (getHeightInches() > ElevatorConstants.maxPos) {
            stopMotors();
        }

        // Only run control if homed
        if (isHomed) {
            double pidOutput = pidController.calculate(getHeightInches(), currentState.position);
            double ff = calculateFeedForward(currentState);
            
            double outputPower = MathUtil.clamp(
                pidOutput + ff,
                -ElevatorConstants.max_output,
                ElevatorConstants.max_output
            );
            
            elevatorMotor.set(outputPower);
        }

        // Update SmartDashboard
        updateTelemetry();
    }

    private void handleBottomLimit() {
        stopMotors();
        encoder.setPosition(ElevatorConstants.bottomPos * ElevatorConstants.countsPerInch);
        isHomed = true;
        setpoint = ElevatorConstants.bottomPos;
        currentState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        goalState = new TrapezoidProfile.State(ElevatorConstants.bottomPos, 0);
        pidController.reset();
    }

    public void stopMotors() {
        elevatorMotor.set(0);
        pidController.reset();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.posTolerance;
    }
    

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return ElevatorConstants.kS * Math.signum(state.velocity) +
               ElevatorConstants.kG +
               ElevatorConstants.kV * state.velocity;
    }

    public void setPositionInches(double inches) {
        if (!isHomed && inches > 0) {
            System.out.println("Warning: Elevator not homed! Home first before moving to positions.");
            return;
        }

        setpoint = MathUtil.clamp(
            inches,
            ElevatorConstants.minPos,
            ElevatorConstants.maxPos
        );
        
        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(setpoint, 0);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", setpoint);
        SmartDashboard.putBoolean("Elevator Homed", isHomed);
        SmartDashboard.putString("Elevator State", currentTarget.toString());
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", currentState.velocity);
    }

    public double getHeightInches() {
        return encoder.getPosition() / ElevatorConstants.countsPerInch;
    }

    public void homeElevator() {
        elevatorMotor.set(-0.1); // Slow downward movement until bottom limit is hit
        if (bottomLimit.get()) {
            handleBottomLimit();
        }
    }

    public boolean isAtPosition(ElevatorPosition position) {
        return pidController.atSetpoint() && 
               Math.abs(getHeightInches() - position.positionInches) < 0.5;
    }

    public boolean isHomed() {
        return isHomed;
    }

    public ElevatorPosition getCurrentTarget() {
        return currentTarget;
    }

    public void setManualPower(double power) {
        // Disable PID control when in manual mode
        pidController.reset();
        currentState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        
        if (!isHomed && power < 0) {
            power = 0;
        }
        
        if (getHeightInches() >= ElevatorConstants.maxPos && power > 0) {
            power = 0;
        }
        
        if (bottomLimit.get() && power < 0) {
            power = 0;
        }
        
        elevatorMotor.set(MathUtil.clamp(power, -ElevatorConstants.max_output, ElevatorConstants.max_output));
    }
}


/*
 * 
 * 
 * Put these things in your code too in robotcontainer. Dont forget pleak
 *
 *  Also change your elevatorconstants 😭
 * 
 * 
 */


//  final         CommandXboxController operatorXbox = new CommandXboxController(1);
  
//   private Elevator m_elevator = new Elevator();


// &

//  //Operator commands
// operatorXbox.leftBumper().onTrue(new InstantCommand(() -> m_elevator.setPositionInches(ElevatorConstants.L1)));
// operatorXbox.rightBumper().onTrue(new InstantCommand(() -> m_elevator.setPositionInches(ElevatorConstants.L2)));
// operatorXbox.leftTrigger().onTrue(new InstantCommand(() -> m_elevator.setPositionInches(ElevatorConstants.L3)));
// operatorXbox.rightTrigger().onTrue(new InstantCommand(() -> m_elevator.setPositionInches(ElevatorConstants.L4)));

