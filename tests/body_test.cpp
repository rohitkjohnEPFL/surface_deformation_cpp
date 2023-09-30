#include <gtest/gtest.h>
#include "vectorFunctions.hpp"

TEST(VectorFunctionsTest, NormalizeTest) {
    std::array<double, 3> vec = {3.0, 1.0, 2.0};
    std::array<double, 3> normalizedVec = normalize(vec);
    // Add your test logic here
}
