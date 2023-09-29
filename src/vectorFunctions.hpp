#pragma once
#include <cmath>  // for std::sqrt
#include <array>  // for std::array

template<std::size_t N>
std::array<double, N> normalize(const std::array<double, N>& arr) {
    // Calculate the magnitude of the array
    double magnitude = 0.0;
    for (const auto& val : arr) {
        magnitude += val * val;
    }
    magnitude = std::sqrt(magnitude);

    // Handle the zero-vector case to avoid division by zero
    if (magnitude == 0.0) {
        return arr;
    }

    // Normalize the array
    std::array<double, N> normalized;
    for (std::size_t i = 0; i < N; ++i) {
        normalized[i] = arr[i] / magnitude;
    }

    return normalized;
}