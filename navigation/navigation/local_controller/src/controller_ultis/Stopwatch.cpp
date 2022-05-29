/*
 * Stopwatch.cpp
 *
 *  Created on: Mar 18, 2010
 *      Author: Sebastian Scherer <sebastian.scherer@uni-tuebingen.de>
 */

#include "controller_ultis/Stopwatch.h"
#include "controller_ultis/RamaxxException.h"

#include <cstdio>

Stopwatch::Stopwatch() { reset(); }

// Should this method do the same like resume() or should it act like reset()?
void Stopwatch::restart() {
  clock_gettime(CLOCK_REALTIME, &mStart);
  stopped = false;
}

void Stopwatch::reset() {
  clock_gettime(CLOCK_REALTIME, &mStart);
  mStop.tv_sec = 0;
  mStop.tv_nsec = 0;
  stopped = false;
}

void Stopwatch::resetStopped() {
  mStop.tv_sec = 0;
  mStop.tv_nsec = 0;
  stopped = true;
}

int Stopwatch::sElapsed() const {
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  return t.tv_sec - mStart.tv_sec;
}

int Stopwatch::msElapsed() const {
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  int msElapsed = (t.tv_sec - mStart.tv_sec) * 1000;
  msElapsed += (int)((t.tv_nsec - mStart.tv_nsec) / (long)1000000);
  return msElapsed;
}

int Stopwatch::usElapsed() const {
  timespec t;
  clock_gettime(CLOCK_REALTIME, &t);

  int seconds = t.tv_sec - mStart.tv_sec;
  int useconds = (int)((t.tv_nsec - mStart.tv_nsec) / (long)1000);
  return seconds * 1000000 + useconds;
}

double Stopwatch::sElapsedDouble() const { return msElapsed() / 1000.0; }

double Stopwatch::elapsed() const {
  int seconds = mStop.tv_sec;
  long nseconds = mStop.tv_nsec;
  if (not stopped) {
    timespec t;
    clock_gettime(CLOCK_REALTIME, &t);
    seconds += t.tv_sec - mStart.tv_sec;
    nseconds += t.tv_nsec - mStart.tv_nsec;
  }
  return seconds + nseconds / 1000000000.0;
}

std::ostream& operator<<(std::ostream& stream, const Stopwatch& watch) {
  int timeInUs = watch.usElapsed();
  int us = timeInUs % 1000;
  int ms = timeInUs / 1000 % 1000;
  int s = timeInUs / 1000000;
  if (s > 0)
    stream << s << " s, " << ms << " ms and ";
  else if (ms > 0)
    stream << ms << " ms and ";
  return stream << us << " us.";
}

void Stopwatch::stop() {
  timespec t;
  if (stopped == false) {
    clock_gettime(CLOCK_REALTIME, &t);
    stopped = true;
    mStop.tv_sec += t.tv_sec - mStart.tv_sec;
    mStop.tv_nsec += t.tv_nsec - mStart.tv_nsec;
  }
}

void Stopwatch::resume() {
  if (not stopped) throw RamaxxException("Stopwatch was not stopped.");
  clock_gettime(CLOCK_REALTIME, &mStart);
  stopped = false;
}

int Stopwatch::sElapsedStatic() { return mStop.tv_sec; }

int Stopwatch::usElapsedStatic() {
  int seconds = mStop.tv_sec;
  long useconds = mStop.tv_nsec / (long)1000;
  return seconds * 1000000 + (int)useconds;
}

long Stopwatch::nsElapsedStatic() {
  long seconds = mStop.tv_sec;
  long nseconds = mStop.tv_nsec;
  return seconds * 1000000000 + nseconds;
}
