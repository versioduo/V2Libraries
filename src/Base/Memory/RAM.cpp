#include "RAM.h"

extern "C" char* sbrk(int incr);
uint32_t         V2Base::Memory::RAM::getFree() {
  uint8_t top;
  return &top - reinterpret_cast<uint8_t*>(sbrk(0));
}
