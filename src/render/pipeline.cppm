export module javelin.render.pipeline;

import std;

import javelin.render.render_context;
import javelin.render.types;

export namespace javelin {

template <class... Passes> struct RenderPipeline final {
    std::tuple<Passes...> passes{};

    template <class Device> void init(Device &device) {
        std::apply([&](auto &...p) { (p.init(device), ...); }, passes);
    }

    template <class Device> void resize(Device &device, const Extent2D extent) {
        std::apply([&](auto &...p) { (p.resize(device, extent), ...); }, passes);
    }

    template <class Device> void shutdown(Device &device) {
        std::apply([&](auto &...p) { (p.shutdown(device), ...); }, passes);
    }

    void execute(RenderContext &ctx) {
        std::apply([&](auto &...p) { (p.execute(ctx), ...); }, passes);
    }
};

} // namespace javelin
