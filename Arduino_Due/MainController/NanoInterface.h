#pragma once
#include <Arduino.h>

void nanoInterfaceBegin();

bool nanoLeftBlocked();
bool nanoRightBlocked();
bool nanoFrontBlocked();
bool nanoRearBlocked();

void nanoHeartbeatUpdate();
bool nanoHeartbeatHealthy();