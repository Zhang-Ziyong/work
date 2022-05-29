/*
 * RamaxxException.cpp
 *
 *  Created on: 13.09.2010
 *      Author: dube
 */

#include "controller_ultis/RamaxxException.h"

using namespace std;

RamaxxException::RamaxxException(string message) {
	mMessage = message;
}

RamaxxException::~RamaxxException() throw() {
}

const char* RamaxxException::what() const throw(){
	return mMessage.c_str();
}
