#ifndef _Winnow_h
#define _Winnow_h

class winnow
{
private:
    double q; //process noise covariance
    double r; //measurement noise covariance
    double x; //value
    double p; //estimation error covariance
    double k; //kalman gain

public:
    winnow(double process_noise, double sensor_noise, double estimated_error, double intial_value)
    {

        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
        this->x = intial_value; //x will hold the iterated filtered value
    }

    double getFilteredValue(double measurement)
    {

        this->p = this->p + this->q;

        //measurement update
        this->k = this->p / (this->p + this->r);
        this->x = this->x + this->k * (measurement - this->x);
        this->p = (1 - this->k) * this->p;

        return this->x;
    }

    // ########### BATCH SET PARAMS ###########
    void setParameters(double process_noise, double sensor_noise, double estimated_error)
    {
        this->q = process_noise;
        this->r = sensor_noise;
        this->p = estimated_error;
    }

    void setParameters(double process_noise, double sensor_noise)
    {
        this->q = process_noise;
        this->r = sensor_noise;
    }

    // ########### SET INDIVIDUAL PARAMS ###########
    void setProcessNoise(double process_noise)
    {
        this->q = process_noise;
    }

    void setSensorNoise(double sensor_noise)
    {
        this->r = sensor_noise;
    }

    void setEstimatedError(double estimated_error)
    {
        this->p = estimated_error;
    }

    // ################### GET PREVIOUSLY SET PARAMS #####################
    double getProcessNoise()
    {
        return this->q;
    }

    double getSensorNoise()
    {
        return this->r;
    }

    double getEstimatedError()
    {
        return this->p;
    }
};

#endif