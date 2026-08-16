#pragma once
#include <Arduino.h>
#include <malloc.h>

extern "C" char* sbrk(int incr);
extern uint32_t  __data_start__;
extern uint32_t  __data_end__;
extern uint32_t  __bss_start__;
extern uint32_t  __bss_end__;
extern uint32_t  __HeapLimit;
extern uint32_t  __end__;
extern uint32_t  __stack;

namespace V2Base::Memory {
  namespace RAM {
    inline auto size() -> uint32_t {
      return HSRAM_SIZE;
    }

    inline auto free() -> uint32_t {
      return (uint8_t*)__get_MSP() - (uint8_t*)sbrk(0);
    }

    namespace Data {
      inline auto size() -> uint32_t {
        return uint32_t(&__bss_end__) - uint32_t(&__data_start__);
      }

      namespace Initialized {
        inline auto size() -> uint32_t {
          return uint32_t(&__data_end__) - uint32_t(&__data_start__);
        }
      }
    }

    namespace Heap {
      inline auto size() -> uint32_t {
        return (uint32_t)sbrk(0) - uint32_t(&__end__);
      }

      inline auto allocated() -> uint32_t {
        return mallinfo().uordblks;
      }
    }

    namespace Stack {
      inline auto size() -> uint32_t {
        return uint32_t(&__stack) - (uint32_t)__get_MSP();
      }
    }
  };
}
