package org.firstinspires.ftc.teamcode.kronbot.utils.pid;

import static org.firstinspires.ftc.teamcode.kronbot.utils.pid.MathUtil.epsilonEquals;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;

/**
 * PID controller with various feedforward components.
 */
public class PIDFController {

    private PIDCoefficients pid;
    private double kV;
    private double kA;
    private double kStatic;
    private FeedforwardFunction kF;
    private NanoClock clock;

    private double errorSum;
    private double lastUpdateTimestamp;

    private boolean inputBounded;
    private double minInput;
    private double maxInput;

    private boolean outputBounded;
    private double minOutput;
    private double maxOutput;

    private double targetPosition;
    private double targetVelocity;
    private double targetAcceleration;
    private double lastError;

    public interface FeedforwardFunction {
        double calculate(double position, Double velocity);
    }

    public PIDFController(PIDCoefficients pid, double kV, double kA, double kStatic,
                          FeedforwardFunction kF, NanoClock clock) {
        this.pid = pid;
        this.kV = kV;
        this.kA = kA;
        this.kStatic = kStatic;
        this.kF = kF;
        this.clock = clock;

        this.errorSum = 0.0;
        this.lastUpdateTimestamp = Double.NaN;
        this.inputBounded = false;
        this.outputBounded = false;
    }

    public PIDFController(PIDCoefficients pid) {
        this(pid, 0.0, 0.0, 0.0, (position, velocity) -> 0.0, NanoClock.system());
    }

    public void setInputBounds(double min, double max) {
        if (min < max) {
            this.inputBounded = true;
            this.minInput = min;
            this.maxInput = max;
        }
    }

    public void setOutputBounds(double min, double max) {
        if (min < max) {
            this.outputBounded = true;
            this.minOutput = min;
            this.maxOutput = max;
        }
    }

    private double getPositionError(double measuredPosition) {
        double error = targetPosition - measuredPosition;
        if (inputBounded) {
            double inputRange = maxInput - minInput;
            while (abs(error) > inputRange / 2.0) {
                error -= signum(error) * inputRange;
            }
        }
        return error;
    }

    public double update(double measuredPosition, Double measuredVelocity) {
        double currentTimestamp = clock.seconds();
        double error = getPositionError(measuredPosition);

        if (Double.isNaN(lastUpdateTimestamp)) {
            lastError = error;
            lastUpdateTimestamp = currentTimestamp;
            return 0.0;
        } else {
            double dt = currentTimestamp - lastUpdateTimestamp;
            errorSum += 0.5 * (error + lastError) * dt;
            double errorDeriv = (error - lastError) / dt;

            lastError = error;
            lastUpdateTimestamp = currentTimestamp;

            double baseOutput = pid.getKP() * error + pid.getKI() * errorSum +
                pid.getKD() * (measuredVelocity != null ? targetVelocity - measuredVelocity : errorDeriv) +
                kV * targetVelocity + kA * targetAcceleration + kF.calculate(measuredPosition, measuredVelocity);
            double output = epsilonEquals(baseOutput, 0.0) ? 0.0 : baseOutput + signum(baseOutput) * kStatic;

            return outputBounded ? max(minOutput, min(output, maxOutput)) : output;
        }
    }

    public void reset() {
        errorSum = 0.0;
        lastError = 0.0;
        lastUpdateTimestamp = Double.NaN;
    }

    public void setPID(PIDCoefficients pid) {
        this.pid = pid;
    }

    public void setkV(double kV) {
        this.kV = kV;
    }

    public void setkA(double kA) {
        this.kA = kA;
    }

    public void setkStatic(double kStatic) {
        this.kStatic = kStatic;
    }

    public void setkF(FeedforwardFunction kF) {
        this.kF = kF;
    }

    public void setClock(NanoClock clock) {
        this.clock = clock;
    }

    public void setErrorSum(double errorSum) {
        this.errorSum = errorSum;
    }

    public void setLastUpdateTimestamp(double lastUpdateTimestamp) {
        this.lastUpdateTimestamp = lastUpdateTimestamp;
    }

    public void setInputBounded(boolean inputBounded) {
        this.inputBounded = inputBounded;
    }

    public void setMinInput(double minInput) {
        this.minInput = minInput;
    }

    public void setMaxInput(double maxInput) {
        this.maxInput = maxInput;
    }

    public void setOutputBounded(boolean outputBounded) {
        this.outputBounded = outputBounded;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setTargetAcceleration(double targetAcceleration) {
        this.targetAcceleration = targetAcceleration;
    }

    public void setLastError(double lastError) {
        this.lastError = lastError;
    }

    public PIDCoefficients getPID() {
        return pid;
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public double getkStatic() {
        return kStatic;
    }

    public FeedforwardFunction getkF() {
        return kF;
    }

    public NanoClock getClock() {
        return clock;
    }

    public double getErrorSum() {
        return errorSum;
    }

    public double getLastUpdateTimestamp() {
        return lastUpdateTimestamp;
    }

    public boolean isInputBounded() {
        return inputBounded;
    }

    public double getMinInput() {
        return minInput;
    }

    public double getMaxInput() {
        return maxInput;
    }

    public boolean isOutputBounded() {
        return outputBounded;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getTargetAcceleration() {
        return targetAcceleration;
    }

    public double getLastError() {
        return lastError;
    }
}
