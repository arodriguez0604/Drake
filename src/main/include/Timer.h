/*
 * Timer.h
 * 
 * Helpful timer routines
 * 
 *   Created on: Jan 27, 2019
 *   Author    : dlawrence
 */

#pragma once

#include <frc/Timer.h>

/* 
 * Blocking Timer
 *
 * This time should only be used for initialization functions that need hard time delays
 * Do not use this timer to insert runtime delays
 */

class BlockingTimer
{
public:

    void wait_BLOCKING(double delay)
    {
        double timebegin = MyTimer.Get();
        MyTimer.Reset();
        MyTimer.Start();
        while( MyTimer.HasPeriodPassed(delay) == false);
        MyTimer.Stop();
    }

    frc::Timer MyTimer;
};
