#pragma once

#include <iostream>
#include <cstring>
#include <limits.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>
#include <sys/mman.h>
#include <malloc.h>
#include <sched.h>
#include <chrono>
#include <memory>
#include <vector>

/****************************************************************************/
// IgH EtherCAT library header file the user-space real-time interface library.
// IgH, EtherCAT related functions and data types.
#include "ecrt.h"  

// Object dictionary paramaters PDO index and default values in here.
#include "object_dictionary.hpp"



