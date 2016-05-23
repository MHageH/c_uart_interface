#ifndef MAVLINK_CONTROL_H_
#define MAVLINK_CONTROL_H_

#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <string.h>

#include <interface.h>

int main(void);

// Scheduler
void commands(void);
void operation (float timer);

// Function helpers
void read_messages_helper(void);
void autopilot_write_helper(void);

#endif