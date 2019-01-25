#pragma once
#include <string>
#include <vector>

template <typename T>
void REQUIRE_VECTOR_APPROX(const std::vector<T>& a, const std::vector<T>& b,
                           T max_error = 0.000001) {
  REQUIRE(a.size() == b.size());
  const auto first_mismatch = std::mismatch(
      a.begin(), a.end(), b.begin(), [max_error](T val_a, T val_b) {
        return val_a == val_b || std::abs(val_a - val_b) < max_error;
      });
  if (first_mismatch.first != a.end()) {
    FAIL("Difference at index " +
         std::to_string(std::distance(a.begin(), first_mismatch.first)) +
         " values=(" + std::to_string(*first_mismatch.first) + "," +
         std::to_string(*first_mismatch.second) + ")");
  }
}
