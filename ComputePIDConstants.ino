#include <Arduino.h>
#include <Wire.h>

#include <stdio.h>

#define _USE_MATH_DEFINES
#include <math.h>

#include <NXTI2CDevice-master/NXTMMX.h>

#define MOTOR_R MMX_Motor_1
#define MOTOR_L MMX_Motor_2
#define MOTORS_SPEED     70
#define MAX_ADJUSTMENT    5         // max allowed step size for speed adjustment
#define MAX_SPEED       100         // max allowed speed

#ifndef LONG
#define LONG long
#endif

#ifndef FLOAT
#define FLOAT float
#endif

#ifndef UCHAR
#define UCHAR unsigned char
#endif

struct MotorsSpeed
{
    MotorsSpeed() { memset(this, 0, sizeof(MotorsSpeed)); }
    UCHAR LMotorSpeed;
    UCHAR RMotorSpeed;
};

struct MotorsTicks
{
    MotorsTicks() { memset(this, 0, sizeof(MotorsTicks)); }
    LONG LMotorTick;
    LONG RMotorTick;
};

static const double TWOPI = 2.0 * M_PI;
static const int N = 100;			// num of iterations for error computation
static const int MAX_TWIDDLE_ITER = 100;

NXTMMX mmx;

//int16_t pid_lastErr;
//int16_t pid_sumErrs;
unsigned long pid_lastCall = 0;
int pid_adjustLMotor = 0;
int pid_adjustRMotor = 0;
int pid_lastLSpeed = 0;
int pid_lastRSpeed = 0;
volatile MotorsTicks pid_prevTicks;

void mmxReset()
{
    mmx.reset();                // resets the encoder positions to zero
    delay(5);
    mmx.setSpeed(MOTOR_L, 0);
    delay(5);
    mmx.setSpeed(MOTOR_R, 0);
    delay(5);
    mmx.resetEncoder(MOTOR_L);
    delay(5);
    mmx.resetEncoder(MOTOR_R);
    delay(5);
}

// Motors seepd can be any value between 0 and 100.
void mmxSetMotorsSpeed(int LMotorSpeed, int RMotorSpeed)
{
    int newLSpeed = constrain(LMotorSpeed, -100, 100);
    int newRSpeed = constrain(RMotorSpeed, -100, 100);

    if (newLSpeed < 0)
        mmx.runUnlimited(MOTOR_L, MMX_Direction_Reverse,-newLSpeed);
    else
        mmx.runUnlimited(MOTOR_L, MMX_Direction_Forward, newLSpeed);

    if (newRSpeed < 0)
        mmx.runUnlimited(MOTOR_R, MMX_Direction_Reverse,-newRSpeed);
    else
        mmx.runUnlimited(MOTOR_R, MMX_Direction_Forward, newRSpeed);
}

void mmxStopAllMotors()
{
    mmx.stop(MMX_Motor_Both, MMX_Next_Action_BrakeHold);
}

bool mmxGetTicksSinceLastUpdate(bool isFirst, int16_t & lft, int16_t & rht, uint16_t & ms )
{
    unsigned long now = millis();
    if (isFirst && !pid_lastCall)
        pid_lastCall = now;
    unsigned long tdelta_ms = now - pid_lastCall;

    if (tdelta_ms < 200)
    {
        lft = rht = ms = 0;
        return false;
    }

    int currLTicks = mmx.getEncoderPosition(MOTOR_L);
    int currRTicks = mmx.getEncoderPosition(MOTOR_R);

    lft = currLTicks - pid_prevTicks.LMotorTick;
    rht = currRTicks - pid_prevTicks.RMotorTick;
    ms  = tdelta_ms;

    pid_prevTicks.LMotorTick = currLTicks;
    pid_prevTicks.RMotorTick = currRTicks;
    pid_lastCall = now;

    if (lft < 0 || rht < 0)
        return false;

    return true;
}

void mmxResetPIDVars()
{
    pid_lastCall = 0;
    pid_adjustLMotor = 0;
    pid_adjustRMotor = 0;
    pid_lastLSpeed = 0;
    pid_lastRSpeed = 0;
    pid_prevTicks.LMotorTick = 0;
    pid_prevTicks.RMotorTick = 0;
}

void mmxSrartNewTest()
{
    mmxReset();

    mmxResetPIDVars();

    mmx.startMotorsInSync();

    mmxSetMotorsSpeed(MOTORS_SPEED, MOTORS_SPEED);
}

double mmxComputeError(double & dt)
{
    bool isFirst = true;

    int16_t lticks;
    int16_t rticks;
    uint16_t ms;

    while(!mmxGetTicksSinceLastUpdate(isFirst, lticks, rticks, ms))
    {
        //delay(2);
        isFirst = false;
    }

    dt = ms;

    int16_t bias = 0;//(rticks*SYSTEM_BIAS)/10000L;
    double err = lticks - rticks + bias;
    return err;
}

void mmxUpdateSpeed(double steer)
{
    int16_t adjust = steer / 2.0;

    pid_adjustLMotor -= adjust;
    pid_adjustRMotor += adjust;

    constrain(pid_adjustLMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);
    constrain(pid_adjustRMotor, -MAX_ADJUSTMENT, MAX_ADJUSTMENT);

    // compute new speed
    int lspeed = MOTORS_SPEED + pid_adjustLMotor;
    int rspeed = MOTORS_SPEED + pid_adjustRMotor;

//    if (1)
//    {
//        //Serial.print(pid_time); Serial.print(", ");
//        //Serial.print(ms); Serial.print(", ");
//        //Serial.print(lticks); Serial.print(", ");
//        //Serial.print(rticks); Serial.print(", ");
//        Serial.print(diff); Serial.print(", ");
//        Serial.print(pid_sumErrs); Serial.print(", ");
//        Serial.print(adjustLMotor); Serial.print(", ");
//        Serial.print(adjustRMotor);
//        Serial.println();
//    }
//    else if (1)
//    {
//        Serial.print("DIFF = ");
//        Serial.print(diff);
//        Serial.print(" ERR = ");
//        Serial.print(pid_sumErrs);
//        Serial.print(" ADJ = (");
//        Serial.print(adjustLMotor);
//        Serial.print(", ");
//        Serial.print(adjustRMotor);
//        Serial.println(")");
//    }


    // Put a limit on the total adjustment in case PID gets out of control
    constrain(lspeed, -MAX_SPEED, MAX_SPEED);
    constrain(rspeed, -MAX_SPEED, MAX_SPEED);

    // update motors
    static const uint8_t ctrl = MMX_CONTROL_SPEED | MMX_CONTROL_GO;

    if (pid_lastLSpeed != lspeed)
    {
        mmx.setSpeedTimeAndControl(MOTOR_L, lspeed, 0, ctrl);
        pid_lastLSpeed = lspeed;
    }

    if (pid_lastRSpeed != lspeed)
    {
        mmx.setSpeedTimeAndControl(MOTOR_R, rspeed, 0, ctrl);
        pid_lastRSpeed = lspeed;
    }
}

void mmxFinishTest()
{
    mmxStopAllMotors();
}

class Random
{
public:
	Random()
	{
        // if analog input pin 0 is unconnected, random analog
        // noise will cause the call to randomSeed() to generate
        // different seed numbers each time the sketch runs.
        // randomSeed() will then shuffle the random function.
        randomSeed(analogRead(0));

		gauss_next = 0.0;
		is_gauss_next_init = false;
	}

	double random()
	{
		//double num = ((double)rand()) / ((double)RAND_MAX);
		double num = ((double)::random(RAND_MAX)) / ((double)RAND_MAX);
		return num;
	}

	double gauss(double mu, double sigma)
	{
		// When x and y are two variables from [0, 1), uniformly
		// distributed, then
		//
		//    cos(2*pi*x)*sqrt(-2*log(1-y))
		//    sin(2*pi*x)*sqrt(-2*log(1-y))
		//
		// are two *independent* variables with normal distribution
		// (mu = 0, sigma = 1).
		// (Lambert Meertens)
		// (corrected version; bug discovered by Mike Miller, fixed by LM)

		// Multithreading note: When two threads call this function
		// simultaneously, it is possible that they will receive the
		// same return value.  The window is very small though.  To
		// avoid this, you have to use a lock around all calls.  (I
		// didn't want to slow this down in the serial case by using a
		// lock here.)

		double z = gauss_next;
		bool is_z_init = is_gauss_next_init;

		gauss_next = 0;
		is_gauss_next_init = false;

		if (!is_z_init)
		{
			double x2pi = random() * TWOPI;
			double g2rad = sqrt(-2.0 * log(1.0 - random()));
			z = cos(x2pi) * g2rad;
			gauss_next = sin(x2pi) * g2rad;
			is_gauss_next_init = true;
		}

		return mu + z * sigma;
	}

private:
	double gauss_next;
	bool is_gauss_next_init;
};

//previous_error = 0
//integral = 0
//start:
//  error = setpoint - measured_value
//  integral = integral + error*dt
//  derivative = (error - previous_error)/dt
//  output = Kp*error + Ki*integral + Kd*derivative
//  previous_error = error
//  wait(dt)
//  goto start

double run(double * params)
{
    mmxSrartNewTest();

	double err = 0.0;									// the error after optimization
	double crosstrack_error = 0.0;						// current error
	double int_crosstrack_error = 0.0;					// integral
	double diff_crosstrack_error = 0.0;					// derivative
	double prev_crosstrack_error = 0.0;

    double dt;

	for (int i = 0; i < N * 2; ++i)
	{
		crosstrack_error = mmxComputeError(dt);
		int_crosstrack_error += crosstrack_error * dt;
		diff_crosstrack_error = (crosstrack_error - prev_crosstrack_error) / dt;

		double steer = params[0] * crosstrack_error + params[1] * int_crosstrack_error + params[2] * diff_crosstrack_error;

		prev_crosstrack_error = crosstrack_error;

		mmxUpdateSpeed(steer);

		if (i >= N)
			err += (crosstrack_error * crosstrack_error);
	}

	mmxFinishTest();

    delay(500);

	return err / double(N);
}

inline double sum(double * d) { return d[0] + d[1] + d[2]; }

void Twiddle(double threshold = 0.001)
{
	double p[3];
	p[0] = 0.9;
	p[1] = 1.8;
	p[2] = 0.0;

	double dp[3]; dp[0] = dp[1] = dp[2] = 1.0;

	double err, best_err = run(p);

	for (int itr = 0; itr < MAX_TWIDDLE_ITER && sum(dp) > threshold; ++itr)
	{
		for (int i = 0; i < 3; ++i)
		{
			p[i] += dp[i];
			err = run(p);
			if (err < best_err)
			{
				best_err = err;
				dp[i] *= 1.1;
			}
			else
			{
				p[i] -= 2 * dp[i];
				err = run(p);

				if (err < best_err)
				{
					best_err = err;
					dp[i] *= 1.1;
				}
				else
				{
					p[i] += dp[i];
					dp[i] *= 0.9;
				}
			}
		}

		//std::cout <<"Twiddle # " << itr << " Params => [" << p[0] << "," << p[1] << "," << p[2] << "] -> best_err " << best_err << std::endl;
        Serial.print("#");
        Serial.print(itr);
        Serial.print(" [");
        Serial.print(p[0], 3);
        Serial.print(",");
        Serial.print(p[1], 3);
        Serial.print(",");
        Serial.print(p[2], 3);
        Serial.print("] -> err ");
        Serial.println(best_err);
	}
}

void setup()
{
	Serial.begin(9600);

	mmxReset();

    Twiddle();

    Serial.println("Finished.");
}

void loop()
{
}
