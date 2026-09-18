#pragma once
#define ATOMIC_RESTORESTATE 0
#define ATOMIC_BLOCK(option) for (bool testAtomic=true; testAtomic; testAtomic=false)
