#ifndef DRIVE_HPP
#define DRIVE_HPP
#include <vector>
#include <stdint.h>


void move_to_stage(uint8_t targetStage);

extern std::vector<double> stages; // Stage positions
extern uint8_t currentStage;      // Current stage index
[[noreturn]] void drive(void);

#endif