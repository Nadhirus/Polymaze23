
/*********************************************************************************/
/*Robot configuration*/
/*********************************************************************************/

const int R_ENCount = 482;
const int L_ENCount = 482;

/*********************************************************************************/
/*Global variables*/
/*********************************************************************************/
float MAX_RPM = 320;
bool Direction_right, Direction_left = true;
volatile long right_wheel_pulse_count = 0, left_wheel_pulse_count = 0;

float interval = 50;
long previousMillis = 0;
long currentMillis = 0;

float rpm_right = 0, rpm_left = 0;
int pwm_right = 0, pwm_left = 0;

float ref_r = 0, ref_l = 0;

float Kpr = 1.5, Kir = 0.085, Kdr = 0.6;
float Kpl = 1.5, Kil = 0.085, Kdl = 0.6;

float error_r = 0, prev_error_r = 0, sum_error_r = 0;
float error_l = 0, prev_error_l = 0, sum_error_l = 0;

const float WHEEL_RADIUS = 0.03;    // meters
const float WHEEL_DISTANCE = 0.255; // meters

float linear_vel_cmd = 0, angular_vel_cmd = 0;
float left_wheel_vel = 0, right_wheel_vel = 0;
