#include "random.h"

uint32_t mersenne_twister() {
#define N     624
#define M     397
#define HI    0x80000000
#define LO    0x7fffffff
  static const uint32_t A[2] = { 0, 0x9908b0df };
  static uint32_t y[N];
  static int index = N+1;
  static const uint32_t seed = 5489UL;
  uint32_t  e;

  if (index > N) {
    int i;
    /* Initialisiere y mit Pseudozufallszahlen */
    y[0] = seed;

    for (i=1; i<N; ++i) {
      y[i] = (1812433253UL * (y[i-1] ^ (y[i-1] >> 30)) + i);
      /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
      /* In the previous versions, MSBs of the seed affect   */
      /* only MSBs of the array mt[].                        */
      /* 2002/01/09 modified by Makoto Matsumoto             */
    }
  }

  if (index >= N) {
    int i;
    /* Berechne neuen Zustandsvektor */
    uint32_t h;

    for (i=0; i<N-M; ++i) {
      h = (y[i] & HI) | (y[i+1] & LO);
      y[i] = y[i+M] ^ (h >> 1) ^ A[h & 1];
    }
    for ( ; i<N-1; ++i) {
      h = (y[i] & HI) | (y[i+1] & LO);
      y[i] = y[i+(M-N)] ^ (h >> 1) ^ A[h & 1];
    }

    h = (y[N-1] & HI) | (y[0] & LO);
    y[N-1] = y[M-1] ^ (h >> 1) ^ A[h & 1];
    index = 0;
  }

  e = y[index++];
  /* Tempering */
  e ^= (e >> 11);
  e ^= (e <<  7) & 0x9d2c5680;
  e ^= (e << 15) & 0xefc60000;
  e ^= (e >> 18);

  return e;
#undef N
#undef M
#undef HI
#undef LO
}
