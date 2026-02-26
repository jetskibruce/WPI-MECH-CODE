package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    // Simulation state
    private double simRPM = 0.0;
    private double targetRPM = 0.0;

    // Shooter constants
    private static final double MAX_RPM = 5000.0;
    private static final double RESPONSE = 0.12; // how fast it spins up/down
    private final Spark shooterMotor = new Spark(4); // PWM port 4
    


    public Shooter() {
        
        Shuffleboard.getTab("Shooter")
            .addNumber("RPM", () -> simRPM);

        Shuffleboard.getTab("Shooter")
            .addNumber("Target RPM", () -> targetRPM);
    }

    /** Command the shooter to a target RPM */
    public void setRPM(double rpm) {
        targetRPM = Math.max(0, Math.min(MAX_RPM, rpm));
    }

    /** Stop the shooter */
    public void stop() {
        targetRPM = 0;
    }

    /** Current simulated RPM */
    public double getRPM() {
        return simRPM;
    }

    @Override
    public void simulationPeriodic() {
        // First‑order response model: simple, realistic, predictable
        simRPM += (targetRPM - simRPM) * RESPONSE;

        // Drive the motor visually (optional)
        shooterMotor.set(simRPM / MAX_RPM);
                
        SmartDashboard.putNumber("Shooter RPM", simRPM);
        SmartDashboard.putNumber("Shooter Target RPM", targetRPM);

    }

    
}