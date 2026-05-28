export module javelin.render.frame_exporter;

import std;

import javelin.core.types;
import javelin.render.render_system;
import javelin.render.types;

export namespace javelin {

// Captures the renderer's RGBA backbuffer to a sequence of PPM (P6) files
// suitable for stitching with ffmpeg.  Used by the offline render path; the
// interactive path never touches this.
//
// Lifecycle: init() once (creates the output directory + writes an ffmpeg
// hint file alongside the frames), then capture() per frame. The internal
// RGBA scratch buffer is sized at init() so per-frame capture is
// allocation-free.
struct FrameExporter final {
    [[nodiscard]] std::expected<void, std::string>
    init(std::filesystem::path output_dir, const Extent2D extent, const u32 fps) {
        if (!extent.is_valid()) {
            return std::unexpected(
                std::format("Frame exporter requires a valid extent (got {}x{})", extent.width, extent.height));
        }

        output_dir_ = std::move(output_dir);
        extent_ = extent;
        rgba_buffer_.assign(static_cast<usize>(extent.width) * static_cast<usize>(extent.height) *
                                static_cast<usize>(4u),
                            0u);

        if (auto r = ensure_output_dir_(); !r) {
            return r;
        }
        return write_ffmpeg_hint_(fps);
    }

    // Read back the renderer's current backbuffer and write
    // <output_dir>/frame_<frame_index:06>.ppm.
    [[nodiscard]] std::expected<void, std::string>
    capture(const RenderSystem &renderer, const u32 frame_index) {
        if (!renderer.readback_rgba8(rgba_buffer_)) {
            return std::unexpected(std::format("Frame exporter failed to read back frame {}", frame_index));
        }
        const std::filesystem::path frame_path = output_dir_ / std::format("frame_{:06}.ppm", frame_index);
        return write_ppm_frame_(frame_path);
    }

  private:
    std::filesystem::path output_dir_{};
    Extent2D extent_{};
    std::vector<u8> rgba_buffer_{};

    [[nodiscard]] std::expected<void, std::string> ensure_output_dir_() const {
        std::error_code ec{};
        std::filesystem::create_directories(output_dir_, ec);
        if (ec) {
            return std::unexpected(
                std::format("Unable to create output directory '{}': {}", output_dir_.string(), ec.message()));
        }
        return {};
    }

    [[nodiscard]] std::expected<void, std::string> write_ffmpeg_hint_(const u32 fps) const {
        const std::filesystem::path hint_path = output_dir_ / "ffmpeg_command.txt";
        std::ofstream out{hint_path, std::ios::trunc};
        if (!out.is_open()) {
            return std::unexpected(std::format("Unable to open hint file '{}'", hint_path.string()));
        }

        out << "ffmpeg -framerate " << fps << " -i frame_%06d.ppm -c:v libx264 -pix_fmt yuv420p output.mp4\n";
        if (!out) {
            return std::unexpected(std::format("I/O error while writing '{}'", hint_path.string()));
        }
        return {};
    }

    [[nodiscard]] std::expected<void, std::string> write_ppm_frame_(const std::filesystem::path &path) const {
        std::ofstream out{path, std::ios::binary | std::ios::trunc};
        if (!out.is_open()) {
            return std::unexpected(std::format("Unable to open frame output '{}'", path.string()));
        }

        out << "P6\n" << extent_.width << ' ' << extent_.height << "\n255\n";

        const usize row_rgba_size = static_cast<usize>(extent_.width) * 4u;
        std::vector<char> row_rgb(static_cast<usize>(extent_.width) * 3u);
        // PPM is top-down; OpenGL readback is bottom-up. Flip while we copy.
        for (i32 y = extent_.height - 1; y >= 0; --y) {
            const usize src_row = static_cast<usize>(y) * row_rgba_size;
            for (i32 x = 0; x < extent_.width; ++x) {
                const usize src = src_row + static_cast<usize>(x) * 4u;
                const usize dst = static_cast<usize>(x) * 3u;
                row_rgb[dst + 0u] = static_cast<char>(rgba_buffer_[src + 0u]);
                row_rgb[dst + 1u] = static_cast<char>(rgba_buffer_[src + 1u]);
                row_rgb[dst + 2u] = static_cast<char>(rgba_buffer_[src + 2u]);
            }
            out.write(row_rgb.data(), static_cast<std::streamsize>(row_rgb.size()));
            if (!out) {
                return std::unexpected(std::format("I/O error while writing '{}'", path.string()));
            }
        }
        return {};
    }
};

} // namespace javelin
