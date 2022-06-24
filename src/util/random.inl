#include "util/random.h"

#include <algorithm>

namespace gopt {

template <typename T>
void RandomNumberGenerator::RandShuffle(std::vector<T>* vec) {
  std::random_shuffle((*vec).begin(), (*vec).end());
}

} // namespace gopt
