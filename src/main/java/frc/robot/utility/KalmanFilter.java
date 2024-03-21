package frc.robot.utility;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;

public class KalmanFilter {
    private final State xState;
    private final State yState;
    private final MeasurementNoise noise;
    private final ProcessNoise processNoise;
    private double lastUpdateTime;


    public KalmanFilter(double initialPosX, double initialPosY, double initialVelX, double initialVelY, double initialAccX, double initialAccY, double positionNoiseVar, double velocityNoiseVar, double accelerationNoiseVar, double positionProcessNoise, double velocityProcessNoise, double accelerationProcessNoise) {
        this.xState = new State(initialPosX, initialVelX, initialAccX, 1, 1, 1); // Initial P values for x
        this.yState = new State(initialPosY, initialVelY, initialAccY, 1, 1, 1); // Initial P values for y
        this.noise = new MeasurementNoise(positionNoiseVar, velocityNoiseVar, accelerationNoiseVar);
        this.processNoise = new ProcessNoise(positionProcessNoise, velocityProcessNoise, accelerationProcessNoise);
        this.lastUpdateTime = System.currentTimeMillis();
    }

    private class State {
        public double position;
        public double velocity;
        public double acceleration;
        public double P_position;
        public double P_velocity;
        public double P_acceleration;

        public State(double position, double velocity, double acceleration, double Pp, double Pv, double Pa) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            this.P_position = Pp;
            this.P_velocity = Pv;
            this.P_acceleration = Pa;
        }

        public void predict(double deltaTime) {
            this.position += this.velocity * deltaTime + 0.5 * this.acceleration * deltaTime * deltaTime;
            this.velocity += this.acceleration * deltaTime;

            this.P_position += this.P_velocity + processNoise.positionProcessNoise * deltaTime;
            this.P_velocity += this.P_acceleration + processNoise.velocityProcessNoise * deltaTime;
            this.P_acceleration += processNoise.accelerationProcessNoise * deltaTime;
        }

        public void updatePosition(double sensorPosition, double R) {
            double innovation = sensorPosition - this.position;
            double K = P_position / (P_position + R);
            this.position = this.position + K * innovation;
            this.P_position = (1 - K) * P_position;

            updateMeasurementNoise(innovation, true, false, false);
        }

        public void updateVelocity(double sensorVelocity, double R) {
            double innovation = sensorVelocity - this.velocity;
            double K = P_velocity / (P_velocity + R);
            this.velocity = this.velocity + K * innovation;
            this.P_velocity = (1 - K) * P_velocity;

            updateMeasurementNoise(innovation, false, true, false);
        }

        public void updateAcceleration(double sensorAcceleration, double R) {
            double innovation = sensorAcceleration - this.acceleration;
            double K = P_acceleration / (P_acceleration + R);
            this.acceleration = this.acceleration + K * innovation;
            this.P_acceleration = (1 - K) * P_acceleration;

            updateMeasurementNoise(innovation, false, false, true);
        }

        public void reset(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;

            this.P_position = 1;
            this.P_velocity = 1;
            this.P_acceleration = 1;
        }
    }

    private static class MeasurementNoise {
        public double R_position;
        public double R_velocity;
        public double R_acceleration;

        public MeasurementNoise(double Rp, double Rv, double Ra) {
            this.R_position = Rp;
            this.R_velocity = Rv;
            this.R_acceleration = Ra;
        }
    }

    private static class ProcessNoise{
        public double positionProcessNoise;
        public double velocityProcessNoise;
        public double accelerationProcessNoise;

        public ProcessNoise(double positionProcessNoise, double velocityProcessNoise, double accelerationProcessNoise){
            this.positionProcessNoise = positionProcessNoise;
            this.velocityProcessNoise = velocityProcessNoise;
            this.accelerationProcessNoise = accelerationProcessNoise;
        }
    }

    private void updateMeasurementNoise(double innovation, boolean isPosition, boolean isVelocity, boolean isAcceleration) {
        if (isPosition) noise.R_position = adjustR(noise.R_position, innovation);

        if (isVelocity) noise.R_velocity = adjustR(noise.R_velocity, innovation);

        if (isAcceleration) noise.R_acceleration = adjustR(noise.R_acceleration, innovation);
    }

    private double adjustR(double currentR, double innovation) {
        // this part of the code like does not really work that well from initial testing, but idk
        double adjustedR = currentR + Math.abs(innovation) * 0.1; // This is a simplistic approach
        return currentR; // adjustedR
    }

    // TODO: checking values and notify driver when its crazy

    public void setMeasurementNoise(double positionNoiseVar, double velocityNoiseVar, double accelerationNoiseVar) {
        noise.R_position = positionNoiseVar;
        noise.R_velocity = velocityNoiseVar;
        noise.R_acceleration = accelerationNoiseVar;
    }

    public void setProcessNoise(double processNoisePosition, double processNoiseVelocity, double processNoiseAcceleration) {
        processNoise.positionProcessNoise = processNoisePosition;
        processNoise.velocityProcessNoise = processNoiseVelocity;
        processNoise.accelerationProcessNoise = processNoiseAcceleration;
    }

    public void reset(double posX, double posY, double velX, double velY, double accX, double accY) {
        xState.reset(posX, velX, accX);
        yState.reset(posY, velY, accY);
        lastUpdateTime = System.currentTimeMillis();
    }

    private double predictAndUpdateTime(){
        double currentTime = System.currentTimeMillis();
        double deltaTime = (currentTime - this.lastUpdateTime) / 1000;
        this.lastUpdateTime = currentTime;
        if (deltaTime <= 0) return 0;
        xState.predict(deltaTime);
        yState.predict(deltaTime);
        return deltaTime;
    }

    private void updateVelocity(double sensorVelX, double sensorVelY){
        xState.updateVelocity(sensorVelX, noise.R_velocity);
        yState.updateVelocity(sensorVelY, noise.R_velocity);
    }
    private void updateAcceleration(double sensorAccX, double sensorAccY){
        xState.updateAcceleration(sensorAccX, noise.R_acceleration);
        yState.updateAcceleration(sensorAccY, noise.R_acceleration);
    }

    public void update(double sensorPosX, double sensorPosY, double sensorVelX, double sensorVelY, double sensorAccX, double sensorAccY) {
        double deltaTime = predictAndUpdateTime();
        if (deltaTime > Constants.Odometry.maxLimeTimeout) {
            reset(sensorPosX, sensorPosY, sensorVelX, sensorVelY, sensorAccX, sensorAccY);
        }
        xState.updatePosition(sensorPosX, noise.R_position);
        yState.updatePosition(sensorPosY, noise.R_position);
        updateVelocity(sensorVelX, sensorVelY);
        updateAcceleration(sensorAccX, sensorAccY);
    }

    public void update(double sensorVelX, double sensorVelY, double sensorAccX, double sensorAccY) {
        predictAndUpdateTime();
        updateVelocity(sensorVelX, sensorVelY);
        updateAcceleration(sensorAccX, sensorAccY);
    }

    public void update(double sensorVelX, double sensorVelY) {
        predictAndUpdateTime();
        updateVelocity(sensorVelX, sensorVelY);
    }

    public void updateNewAcceleration(double sensorAccX, double sensorAccY) {
        predictAndUpdateTime();
        updateAcceleration(sensorAccX, sensorAccY);
    }

    public double getPosX() {
        return xState.position;
    }

    public double getPosY() {
        return yState.position;
    }

    public Translation2d getTranslation2d(){
        return new Translation2d(xState.position, yState.position);
    }

    public void debugDisplayValues(){
        TelemetryUpdater.setTelemetryValue("Kalman X Position", xState.position);
        TelemetryUpdater.setTelemetryValue("Kalman Y Position", yState.position);

        // TelemetryUpdater.setTelemetryValue("Kalman Measurement Noise R Position", noise.R_position);
        // TelemetryUpdater.setTelemetryValue("Kalman Measurement Noise R Velocity", noise.R_velocity);
        // TelemetryUpdater.setTelemetryValue("Kalman Measurement Noise R Acceleration", noise.R_acceleration);
    }
}