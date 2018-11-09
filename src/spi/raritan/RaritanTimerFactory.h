/**
 * @file RaritanTimerFactory.h
 */

#pragma once

#include "../ITimerFactory.h"
#include "RaritanEventLoop.h"

class RaritanTimerFactory : public ITimerFactory {
public:
	RaritanTimerFactory(RaritanEventLoop& eventLoop);
	~RaritanTimerFactory();
	virtual ITimer* create() const;
private:
	RaritanEventLoop& m_eventLoop;
};
