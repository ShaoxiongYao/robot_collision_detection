//
// Created by Joao Bimbo on 13/11/17.
//

#ifndef SOMA_CONTROLLERS_FILTERS_H
#define SOMA_CONTROLLERS_FILTERS_H

#include <vector>
#include <ctime>
#include <iostream>
#include <stdexcept>
#include <cmath>

class LP_Filter{
public:
    LP_Filter(int samples){
        t=0;
        full=false;
        n=samples;
        measurements.resize(samples);
    }
    int n;
    int t;
    bool full;
    std::vector<double> measurements;
    double filtered;
    clock_t last_sample;

    void add_measurement(double in){
        measurements.at(t)=in;
        last_sample = clock();
        t++;
        if(t==n) {
            full=true;
            t=0;
        }
    }

    double get_average(){
        double sum=0;
        int number;

        if(full) number=n;
        else number=t;

        for(int i=0;i<number;i++){
            sum+=measurements.at(i);
        }
        return sum/number;
    }

    double get_frequency(){
        clock_t now = clock();
        double elapsed_secs = double(now - last_sample) / CLOCKS_PER_SEC;
        return 1/elapsed_secs;
    }
};

/* -*- coding: utf-8 -*-
 *
 * OneEuroFilter.cc -
 *
 * Author: Nicolas Roussel (nicolas.roussel@inria.fr)
 *
 */



// -----------------------------------------------------------------
// Utilities


typedef double TimeStamp ; // in seconds

static const TimeStamp UndefinedTime = -1.0 ;

// -----------------------------------------------------------------

class LowPassFilter {

    double y, a, s ;
    bool initialized ;

    void setAlpha(double alpha) {
        if (alpha<=0.0 || alpha>1.0)
            throw std::range_error("alpha should be in (0.0., 1.0]") ;
        a = alpha ;
    }

public:

    LowPassFilter(double alpha, double initval=0.0) {
        y = s = initval ;
        setAlpha(alpha) ;
        initialized = false ;
    }

    double filter(double value) {
        double result ;
        if (initialized)
            result = a*value + (1.0-a)*s ;
        else {
            result = value ;
            initialized = true ;
        }
        y = value ;
        s = result ;
        return result ;
    }

    double filterWithAlpha(double value, double alpha) {
        setAlpha(alpha) ;
        return filter(value) ;
    }

    bool hasLastRawValue(void) {
        return initialized ;
    }

    double lastRawValue(void) {
        return y ;
    }

} ;

// -----------------------------------------------------------------

class OneEuroFilter {

    double freq ;
    double mincutoff ;
    double beta_ ;
    double dcutoff ;
    LowPassFilter *x ;
    LowPassFilter *dx ;
    TimeStamp lasttime ;

    double alpha(double cutoff) {
        double te = 1.0 / freq ;
        double tau = 1.0 / (2*M_PI*cutoff) ;
        return 1.0 / (1.0 + tau/te) ;
    }

    void setFrequency(double f) {
        if (f<=0) throw std::range_error("freq should be >0") ;
        freq = f ;
    }

    void setMinCutoff(double mc) {
        if (mc<=0) throw std::range_error("mincutoff should be >0") ;
        mincutoff = mc ;
    }

    void setBeta(double b) {
        beta_ = b ;
    }

    void setDerivateCutoff(double dc) {
        if (dc<=0) throw std::range_error("dcutoff should be >0") ;
        dcutoff = dc ;
    }

public:

    OneEuroFilter(double freq,
                  double mincutoff=1.0, double beta_=0.0, double dcutoff=1.0) {
        setFrequency(freq) ;
        setMinCutoff(mincutoff) ;
        setBeta(beta_) ;
        setDerivateCutoff(dcutoff) ;
        x = new LowPassFilter(alpha(mincutoff)) ;
        dx = new LowPassFilter(alpha(dcutoff)) ;
        lasttime = UndefinedTime ;
    }

    double filter(double value, TimeStamp timestamp=UndefinedTime) {
        // update the sampling frequency based on timestamps
        if (lasttime!=UndefinedTime && timestamp!=UndefinedTime)
            freq = 1.0 / (timestamp-lasttime) ;
        lasttime = timestamp ;
        // estimate the current variation per second
        double dvalue = x->hasLastRawValue() ? (value - x->lastRawValue())*freq : 0.0 ; // FIXME: 0.0 or value?
        double edvalue = dx->filterWithAlpha(dvalue, alpha(dcutoff)) ;
        // use it to update the cutoff frequency
        double cutoff = mincutoff + beta_*fabs(edvalue) ;
        // filter the given value
        return x->filterWithAlpha(value, alpha(cutoff)) ;
    }

    ~OneEuroFilter(void) {
        delete x ;
        delete dx ;
    }

} ;

// -----------------------------------------------------------------
/*
int
main(int argc, char **argv) {
    randSeed() ;

    double duration = 10.0 ; // seconds

    double frequency = 120 ; // Hz
    double mincutoff = 1.0 ; // FIXME
    double beta = 1.0 ;      // FIXME
    double dcutoff = 1.0 ;   // this one should be ok

    std::cout << "#SRC OneEuroFilter.cc" << std::endl
              << "#CFG {'beta': " << beta << ", 'freq': " << frequency << ", 'dcutoff': " << dcutoff << ", 'mincutoff': " << mincutoff << "}" << std::endl
              << "#LOG timestamp, signal, noisy, filtered" << std::endl ;

    OneEuroFilter f(frequency, mincutoff, beta, dcutoff) ;
    for (TimeStamp timestamp=0.0; timestamp<duration; timestamp+=1.0/frequency) {
        double signal = sin(timestamp) ;
        double noisy = signal + (unifRand()-0.5)/5.0 ;
        double filtered = f.filter(noisy, timestamp) ;
        std::cout << timestamp << ", "
                  << signal << ", "
                  << noisy << ", "
                  << filtered
                  << std::endl ;
    }

    return 0 ;
}
 */
#endif //SOMA_CONTROLLERS_FILTERS_H
