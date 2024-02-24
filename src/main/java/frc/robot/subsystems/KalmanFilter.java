package frc.robot.subsystems;

import frc.robot.Constants;

public class KalmanFilter {
    private State xState;
    private State yState;
    private MeasurementNoise noise;
    private double lastUpdateTime;

    public KalmanFilter(double initialPosX, double initialPosY, double initialVelX, double initialVelY, double initialAccX, double initialAccY, double positionNoiseVar, double velocityNoiseVar, double accelerationNoiseVar) {
        this.xState = new State(initialPosX, initialVelX, initialAccX, 1, 1, 1); // Initial P values for x
        this.yState = new State(initialPosY, initialVelY, initialAccY, 1, 1, 1); // Initial P values for y
        this.noise = new MeasurementNoise(positionNoiseVar, velocityNoiseVar, accelerationNoiseVar);
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
            // Increase prediction uncertainty
            this.P_position += P_velocity;
            this.P_velocity += P_acceleration;
            this.P_acceleration += 0.1; // Simplified process noise assumption
        }

        public void updatePosition(double sensorPosition, double R) {
            double K = P_position / (P_position + R);
            this.position = this.position + K * (sensorPosition - this.position);
            this.P_position = (1 - K) * P_position;
        }

        public void updateVelocity(double sensorVelocity, double R) {
            double K = P_velocity / (P_velocity + R);
            this.velocity = this.velocity + K * (sensorVelocity - this.velocity);
            this.P_velocity = (1 - K) * P_velocity;
        }

        public void updateAcceleration(double sensorAcceleration, double R) {
            double K = P_acceleration / (P_acceleration + R);
            this.acceleration = this.acceleration + K * (sensorAcceleration - this.acceleration);
            this.P_acceleration = (1 - K) * P_acceleration;
        }

        public void reset(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
            // Optionally reset the error covariance to initial conditions
            this.P_position = 1; // Or another suitable initial value
            this.P_velocity = 1; // Or another suitable initial value
            this.P_acceleration = 1; // Or another suitable initial value
        }
    }

    private class MeasurementNoise {
        public double R_position;
        public double R_velocity;
        public double R_acceleration;

        public MeasurementNoise(double Rp, double Rv, double Ra) {
            this.R_position = Rp;
            this.R_velocity = Rv;
            this.R_acceleration = Ra;
        }
    }

    private void reset(double posX, double posY, double velX, double velY, double accX, double accY) {
        xState.reset(posX, velX, accX);
        yState.reset(posY, velY, accY);
        lastUpdateTime = System.currentTimeMillis();
    }

    private double predictAndUpdateTime(){
        double currentTime = System.currentTimeMillis();
        double deltaTime = currentTime - this.lastUpdateTime;
        this.lastUpdateTime = currentTime;
        if (deltaTime <= 0) return 0; 
        xState.predict(deltaTime);
        yState.predict(deltaTime);
        return deltaTime;
    }

    private void updateVelocityAndAcceleration(double sensorVelX, double sensorVelY, double sensorAccX, double sensorAccY){
        xState.updateVelocity(sensorVelX, noise.R_velocity);
        yState.updateVelocity(sensorVelY, noise.R_velocity);
        xState.updateAcceleration(sensorAccX, noise.R_acceleration);
        yState.updateAcceleration(sensorAccY, noise.R_acceleration);
    }

    public void update(double currentTime, double sensorPosX, double sensorPosY, double sensorVelX, double sensorVelY, double sensorAccX, double sensorAccY) {
        double deltaTime = predictAndUpdateTime();
        if (deltaTime > Constants.Odometry.maxLimeTimeout) {
            reset(sensorPosX, sensorPosY, sensorVelX, sensorVelY, sensorAccX, sensorAccY);
        }
        xState.updatePosition(sensorPosX, noise.R_position);
        yState.updatePosition(sensorPosY, noise.R_position);
        updateVelocityAndAcceleration(sensorVelX, sensorVelY, sensorAccX, sensorAccY);
    }

    public void update(double currentTime, double sensorVelX, double sensorVelY, double sensorAccX, double sensorAccY) {
        predictAndUpdateTime();
        updateVelocityAndAcceleration(sensorVelX, sensorVelY, sensorAccX, sensorAccY);
    }
}
