


class PIDController{

private:
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float previousError;

public:
    PIDController(float p, float i, float d): Kp(p), Ki(i), Kd(d), integral(0), previousError(0){};
    PIDController(){};
    float calculateControlSignal(float setpoint, float measuredValue ){
        float output;
        float error = setpoint-measuredValue;
        integral += error;
        float derivative = error - previousError;
        output = Kp*error + Ki*integral + Kd*derivative;
        previousError = error;
        return output;



    }





};