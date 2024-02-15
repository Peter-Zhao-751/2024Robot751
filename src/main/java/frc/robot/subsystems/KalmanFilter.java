package frc.robot.subsystems;

public class KalmanFilter {
    public static class States {
        public double position;
        public double velocity;
        public double acceleration;
    
        public States(double p, double v, double a) { 
          this.position = p;
          this.velocity = v;
          this.acceleration = a;
        }
        public States(double p){
            this(p, 0, 0);
        }
        public double getPosition() {
          return position;
        }
        public void reset(double p){
            reset(p, false);
        }
        public void reset(double p, boolean hard){
            this.position = p;
            if (hard){
                this.velocity = 0;
                this.acceleration = 0;
            }
        }
    }

    public static double linearInterpolation(double pastDisplacement, double pastVelocity, double pastAcc, double sensorDisplacement, double bias, double deltaTime) {
        double predictedDisplacement = pastDisplacement + pastVelocity * deltaTime + 0.5 * pastAcc * deltaTime * deltaTime;
        return predictedDisplacement * (1-bias) + bias * sensorDisplacement;
    }

    public static void updateLerp(States state, double sensorPosition, double deltaTime){
        double newPosition = linearInterpolation(state.position, state.velocity, state.acceleration, sensorPosition, 0.5, deltaTime);
        double newVelocity = (newPosition - state.position) / deltaTime;
        state.acceleration = (newVelocity - state.velocity) / deltaTime;
        state.velocity = newVelocity;
        state.position = newPosition;
    }


}
