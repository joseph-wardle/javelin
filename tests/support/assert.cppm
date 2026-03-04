export module javelin.tests.assert;

import std;

import javelin.core.types;

namespace javelin::tests {

export struct Failure final : std::runtime_error {
    using std::runtime_error::runtime_error;
};

export [[nodiscard]] bool nearly_equal(const f32 lhs, const f32 rhs, const f32 epsilon = 1e-4f) noexcept {
    return std::fabs(lhs - rhs) <= epsilon;
}

export void require(const bool condition, const std::string_view message) {
    if (!condition) {
        throw Failure{std::string{message}};
    }
}

export void require_near(const f32 actual, const f32 expected, const f32 epsilon, const std::string_view label) {
    if (!nearly_equal(actual, expected, epsilon)) {
        throw Failure{std::format("{} mismatch: expected {:.6f}, got {:.6f}", label, expected, actual)};
    }
}

} // namespace javelin::tests
