#include "util/random.h"

namespace gopt {

#ifdef GOPT_HAS_THREAD_LOCAL_KEYWORD
thread_local std::mt19937 util_generator;
#else
static std::mt19937 util_generator;
#endif  // GOPT_HAS_THREAD_LOCAL_KEYWORD

RandomNumberGenerator::RandomNumberGenerator() {
  const unsigned seed =
      std::chrono::system_clock::now().time_since_epoch().count();
  util_generator.seed(seed);
}

RandomNumberGenerator::RandomNumberGenerator(const unsigned seed) {
  util_generator.seed(seed);
}

void RandomNumberGenerator::Seed(const unsigned seed) {
  util_generator.seed(seed);
}

// Get a random double between lower and upper (inclusive).
double RandomNumberGenerator::RandDouble(const double lower,
                                         const double upper) {
  std::uniform_real_distribution<double> distribution(lower, upper);
  return distribution(util_generator);
}

float RandomNumberGenerator::RandFloat(const float lower, const float upper) {
  std::uniform_real_distribution<float> distribution(lower, upper);
  return distribution(util_generator);
}

// Get a random int between lower and upper (inclusive).
int RandomNumberGenerator::RandInt(const int lower, const int upper) {
  std::uniform_int_distribution<int> distribution(lower, upper);
  return distribution(util_generator);
}

// Gaussian Distribution with the corresponding mean and std dev.
double RandomNumberGenerator::RandGaussian(const double mean,
                                           const double std_dev) {
  std::normal_distribution<double> distribution(mean, std_dev);
  return distribution(util_generator);
}

Eigen::Vector2d RandomNumberGenerator::RandVector2d(const double min,
                                                    const double max) {
  return Eigen::Vector2d(RandDouble(min, max), RandDouble(min, max));
}

Eigen::Vector2d RandomNumberGenerator::RandVector2d() {
  return RandVector2d(-1.0, 1.0);
}

Eigen::Vector3d RandomNumberGenerator::RandVector3d(const double min,
                                                    const double max) {
  return Eigen::Vector3d(RandDouble(min, max), RandDouble(min, max),
                         RandDouble(min, max));
}

Eigen::Vector3d RandomNumberGenerator::RandVector3d() {
  return RandVector3d(-1.0, 1.0);
}

Eigen::Vector4d RandomNumberGenerator::RandVector4d(const double min,
                                                    const double max) {
  return Eigen::Vector4d(RandDouble(min, max), RandDouble(min, max),
                         RandDouble(min, max), RandDouble(min, max));
}

Eigen::Vector4d RandomNumberGenerator::RandVector4d() {
  return RandVector4d(-1.0, 1.0);
}

}  // namespace gopt
