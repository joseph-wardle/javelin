export module javelin.bench.stats;

import std;

import javelin.core.types;

namespace javelin::bench {

export [[nodiscard]] f64 median(const std::span<const f64> samples) {
    if (samples.empty()) {
        return std::numeric_limits<f64>::quiet_NaN();
    }

    std::vector<f64> sorted{samples.begin(), samples.end()};
    std::sort(sorted.begin(), sorted.end());
    return sorted[sorted.size() / 2u];
}

export [[nodiscard]] f64 percentile(const std::span<const f64> samples, const f64 percentile_ratio) {
    if (samples.empty()) {
        return std::numeric_limits<f64>::quiet_NaN();
    }

    std::vector<f64> sorted{samples.begin(), samples.end()};
    std::sort(sorted.begin(), sorted.end());

    const f64 clamped = std::clamp(percentile_ratio, 0.0, 1.0);
    const usize index = static_cast<usize>(std::ceil(static_cast<f64>(sorted.size()) * clamped));
    const usize clamped_index = std::min(sorted.size() - 1u, (index == 0u) ? 0u : index - 1u);
    return sorted[clamped_index];
}

export [[nodiscard]] f64 p95(const std::span<const f64> samples) { return percentile(samples, 0.95); }

} // namespace javelin::bench
