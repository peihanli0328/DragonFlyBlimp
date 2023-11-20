
// PID constants

// // yaw
const double kp = 1.5;
const double ki = 0.001;
const double kd = 0.01;


// const double kp = 1.5;
// const double ki = 0.001;
// const double kd = 1;

// pressure (alt)
const double kp_p = 3000.0;
const double ki_p = 0.2;
const double kd_p = 20.0;

// forward
const double kp_f = 3.8;
const double ki_f = 0.000;
const double kd_f = 0.0;

// hold
const double kp_h = 0.5;
const double ki_h = 0;
const double kd_h = 3;

double angle_difference(double angle1, double angle2) { //angle1 current , angle2 target 
    double difference = angle2 - angle1;
    if (difference > 180) {
        difference -= 360;
    } else if (difference < -180) {
        difference += 360;
    }
    return difference;
}

double computePID_yaw(double goal_yaw, double current_yaw) {
  double error_yaw = angle_difference(current_yaw, goal_yaw);

  // double error_yaw = goal_yaw - current_yaw;
  integral_yaw += error_yaw;
  double derivative_yaw = error_yaw - previous_error_yaw;
  previous_error_yaw = error_yaw;
  double output_yaw = kp * error_yaw + ki * integral_yaw + kd * derivative_yaw;
  return output_yaw;
}

double computePID_yaw_fw(double goal_yaw, double current_yaw) {
  double error_yaw = angle_difference(current_yaw, goal_yaw);
  // double error_yaw = goal_yaw - current_yaw;
  integral_yaw += error_yaw;
  double derivative_yaw = error_yaw - previous_error_yaw;
  previous_error_yaw = error_yaw;
  double output_yaw = kp_f * error_yaw + ki_f * integral_yaw + kd_f * derivative_yaw;
  return output_yaw;
}

// PID compute function
double computePID_pressure(double goal_pressure, double current_pressure) {

  double error_pressure = current_pressure - goal_pressure;
  integral_pressure += error_pressure;
  double derivative_pressure = error_pressure - previous_error_pressure;
  previous_error_pressure = error_pressure;
  double output_pressure = kp_p * error_pressure + ki_p * integral_pressure + kd_p * derivative_pressure;
  return output_pressure;
}
