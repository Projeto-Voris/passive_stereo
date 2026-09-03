#pragma once

#include <cstdint>
#include <cstddef>
#include <string>

namespace passive_stereo {

struct alignas(16) PointXYZRGB {
    float x;
    float y;
    float z;
    uint32_t rgb;
};

struct TriangulationParams {
    int width;
    int height;
    int u0;
    int v0;
    int u1;
    int v1;
    int step;

    float fx;
    float fy;
    float cx;
    float cy;
    float baseline;
    float min_disp;
    float max_dist_sq; // <= 0.0f disables max distance filter

    // 3x3 combined rotation/affine matrix in row-major order:
    // [X', Y', Z']^T = R * [X, Y, Z]^T + T
    float R[9];
    float T[3];

    int channels;    // 1 (mono8/gray), 3 (bgr8/rgb8), 4 (bgra8/rgba8)
    bool is_rgb;     // true if channel order is RGB, false if BGR
    int img_step;    // row step in bytes for color image
    int disp_step;   // row step in bytes for disparity image

    // Confidence-based noise filter parameters
    int confidence_radius;   // Half-window size (e.g., 2 → 5×5). 0 = disabled.
    float confidence_alpha;  // Sensitivity: confidence = 1/(1 + alpha * sigma)
    float min_confidence;    // Threshold: discard points below this confidence
};

class CudaTriangulator {
public:
    CudaTriangulator();
    ~CudaTriangulator();

    bool is_available() const { return available_; }
    const std::string& get_device_name() const { return device_name_; }

    bool init();
    void cleanup();

    // Executes GPU triangulation.
    //
    // Inputs are staged into pinned buffers before H→D upload, enabling true
    // async DMA so the CPU is not stalled during the transfer.
    //
    // The point counter uses CUDA mapped memory: the GPU writes the count
    // directly into host-visible memory — no explicit D→H copy is needed.
    //
    // Output points are written into an internal pinned host buffer. The caller
    // receives a pointer to that buffer via h_points_out (no extra memcpy).
    //
    // Returns the number of valid points produced.
    size_t triangulate(
        const float*   h_disparity,
        size_t         disp_bytes,
        const uint8_t* h_image,
        size_t         img_bytes,
        const TriangulationParams& params,
        size_t         max_points,
        PointXYZRGB**  h_points_out  // receives pointer to internal pinned output buffer
    );

private:
    bool ensure_device_buffers(size_t disp_bytes, size_t img_bytes, size_t max_points);
    bool ensure_pinned_input(size_t disp_bytes, size_t img_bytes);
    bool ensure_pinned_output(size_t max_points);

    bool available_{false};
    std::string device_name_;

    // --- Device buffers (GPU VRAM) ---
    float*       d_disparity_{nullptr};
    size_t       d_disp_capacity_{0};

    uint8_t*     d_image_{nullptr};
    size_t       d_img_capacity_{0};

    PointXYZRGB* d_points_{nullptr};
    size_t       d_points_capacity_{0};

    // --- Mapped counter: GPU writes directly to host memory (zero D→H copy) ---
    // cudaHostAllocMapped gives the CPU a host pointer and lets the GPU write via
    // the device alias obtained from cudaHostGetDevicePointer.
    volatile unsigned int* h_count_mapped_{nullptr};  // CPU reads this
    unsigned int*          d_count_{nullptr};          // GPU writes this (same physical page)

    // --- Pinned staging buffers for H→D inputs (true async DMA, CPU-free) ---
    float*   h_pinned_disp_{nullptr};
    size_t   h_pinned_disp_cap_{0};

    uint8_t* h_pinned_img_{nullptr};
    size_t   h_pinned_img_cap_{0};

    // --- Pinned output buffer (fast D→H DMA; caller reads without extra memcpy) ---
    PointXYZRGB* h_pinned_points_{nullptr};
    size_t       h_pinned_points_cap_{0};  // capacity in number of points

    void* stream_ptr_{nullptr};
};

} // namespace passive_stereo

