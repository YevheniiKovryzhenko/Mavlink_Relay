#pragma once

#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <cstdint>
#include "autopilot_interface.hpp"

//////////////////////////////////////////////////////////////////////////////
//                                                                          //
//                                                                          //
//                         Non-blocking CLI                                 //
//                                                                          //
//                                                                          //
//////////////////////////////////////////////////////////////////////////////
bool _kbhit();

int non_blocking_client_update(Autopilot_Interface* cli);