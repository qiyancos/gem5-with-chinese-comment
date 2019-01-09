#include <iostream>
#include <vector>
#include <stdlib.h>
#include <sys/time.h>

using std::vector;

typedef unsigned long long uint64;

uint64 Now();

const int trials = 5;
const int vectors = 1000;
const int vectorsize = 20000;

vector<int> *v[vectors];

int main() {
  srand(Now());

  for (int i = 0; i < vectors; ++i)
    v[i] = new vector<int>;

  uint64 timeelapsed = 0;
  uint64 timestart, timeend;
  
  for (int i = 0; i < trials; ++i) {
    // fill with dummy data
    for (int j = 0; j < vectors; ++j) {
      for (int k = 0; k < vectorsize; ++k) {
        v[j]->push_back(rand());
      }
    }

    timestart = Now();
    for (int i = 0; i < vectors; ++i) {
      // this is slow
      v[i]->clear();
    }
    timeend = Now();
  
    timeelapsed += timeend - timestart;
  }

  std::cout << "Time: " << timeelapsed << "\n";
}

uint64 Now() {
    struct timeval        tv;

    gettimeofday(&tv, NULL);
    uint64 val = tv.tv_sec;
    val = val * 1000000ull + tv.tv_usec;
    return val;
}

