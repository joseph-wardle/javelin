export module javelin.tests.runner;

import std;

import javelin.tests.assert;

namespace javelin::tests {

export using TestFn = void (*)();

export struct TestCase final {
    std::string_view name{};
    TestFn fn{};
};

export [[nodiscard]] bool run_one(const TestCase &test_case) {
    try {
        test_case.fn();
        std::cout << "[PASS] " << test_case.name << '\n';
        return true;
    } catch (const Failure &failure) {
        std::cerr << "[FAIL] " << test_case.name << ": " << failure.what() << '\n';
        return false;
    } catch (const std::exception &exception) {
        std::cerr << "[FAIL] " << test_case.name << ": unexpected exception: " << exception.what() << '\n';
        return false;
    } catch (...) {
        std::cerr << "[FAIL] " << test_case.name << ": unknown exception\n";
        return false;
    }
}

export int run_cli(const std::span<const TestCase> tests, const int argc, char **argv) {
    if (argc == 2) {
        const std::string_view requested{argv[1]};
        const auto it = std::find_if(tests.begin(), tests.end(),
                                     [&](const TestCase &test_case) { return test_case.name == requested; });
        if (it == tests.end()) {
            std::cerr << "Unknown test: " << requested << '\n';
            std::cerr << "Available tests:\n";
            for (const TestCase &test_case : tests) {
                std::cerr << "  - " << test_case.name << '\n';
            }
            return 2;
        }
        return run_one(*it) ? 0 : 1;
    }

    bool all_passed = true;
    for (const TestCase &test_case : tests) {
        all_passed = run_one(test_case) && all_passed;
    }
    return all_passed ? 0 : 1;
}

} // namespace javelin::tests
