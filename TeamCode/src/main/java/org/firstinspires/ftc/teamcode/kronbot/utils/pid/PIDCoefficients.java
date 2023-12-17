package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

/**
 * Proportional, integral, and derivative (PID) gains used by PIDFController.
 */
public class PIDCoefficients {

    private double kP;
    private double kI;
    private double kD;

    // Constructor
    public PIDCoefficients(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    // Default constructor
    public PIDCoefficients() {
        this.kP = 0.0;
        this.kI = 0.0;
        this.kD = 0.0;
    }

    // Getters and Setters
    public double getKP() {
        return kP;
    }

    public void setKP(double kP) {
        this.kP = kP;
    }

    public double getKI() {
        return kI;
    }

    public void setKI(double kI) {
        this.kI = kI;
    }

    public double getKD() {
        return kD;
    }

    public void setKD(double kD) {
        this.kD = kD;
    }
}
