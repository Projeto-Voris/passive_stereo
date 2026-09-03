#include "triangulation_cuda.cuh"

#include <cuda_runtime.h>
#include <iostream>
#include <algorithm>
#include <cstring>

namespace passive_stereo {

__global__ void triangulate_kernel(
    const float* __restrict__ d_disparity,
    const uint8_t* __restrict__ d_image,
    TriangulationParams params,
    PointXYZRGB* __restrict__ d_out_points,
    unsigned int* __restrict__ d_out_count,
    size_t max_points)
{
    int sample_u = blockIdx.x * blockDim.x + threadIdx.x;
    int sample_v = blockIdx.y * blockDim.y + threadIdx.y;

    int n_u = (params.u1 - params.u0 + params.step - 1) / params.step;
    int n_v = (params.v1 - params.v0 + params.step - 1) / params.step;

    bool thread_in_bounds = (sample_u < n_u && sample_v < n_v);
    bool is_valid = false;
    PointXYZRGB pt;
    pt.x = 0.0f;
    pt.y = 0.0f;
    pt.z = 0.0f;
    pt.rgb = 0;

    if (thread_in_bounds) {
        int u = params.u0 + sample_u * params.step;
        int v = params.v0 + sample_v * params.step;

        const float* disp_row = reinterpret_cast<const float*>(
            reinterpret_cast<const char*>(d_disparity) + v * params.disp_step);
        float d = disp_row[u];

        if (d > params.min_disp) {
            float Z = -params.baseline * params.fx / d;
            float X = (static_cast<float>(u) - params.cx) * Z / params.fx;
            float Y = (static_cast<float>(v) - params.cy) * Z / params.fy;

            // Combined 3D affine transform: R * [X, Y, Z]^T + T
            float x_trans = params.R[0] * X + params.R[1] * Y + params.R[2] * Z + params.T[0];
            float y_trans = params.R[3] * X + params.R[4] * Y + params.R[5] * Z + params.T[1];
            float z_trans = params.R[6] * X + params.R[7] * Y + params.R[8] * Z + params.T[2];

            bool dist_ok = true;
            if (params.max_dist_sq > 0.0f) {
                float dist_sq = x_trans * x_trans + y_trans * y_trans + z_trans * z_trans;
                if (dist_sq > params.max_dist_sq) {
                    dist_ok = false;
                }
            }

            if (dist_ok) {
                // Confidence-based noise gate
                if (params.confidence_radius > 0) {
                    float sum = 0.0f, sum_sq = 0.0f;
                    int count = 0;
                    for (int dv = -params.confidence_radius; dv <= params.confidence_radius; ++dv) {
                        int vv = v + dv;
                        if (vv < 0 || vv >= params.height) continue;
                        const float* nb_row = reinterpret_cast<const float*>(
                            reinterpret_cast<const char*>(d_disparity) + vv * params.disp_step);
                        for (int du = -params.confidence_radius; du <= params.confidence_radius; ++du) {
                            int uu = u + du;
                            if (uu < 0 || uu >= params.width) continue;
                            float dn = nb_row[uu];
                            if (dn > params.min_disp) {
                                sum += dn;
                                sum_sq += dn * dn;
                                count++;
                            }
                        }
                    }
                    if (count > 1) {
                        float mean = sum / count;
                        float variance = (sum_sq / count) - (mean * mean);
                        float sigma = sqrtf(fmaxf(variance, 0.0f));
                        float conf = 1.0f / (1.0f + params.confidence_alpha * sigma);
                        if (conf < params.min_confidence) {
                            dist_ok = false;
                        }
                    }
                }
            }

            if (dist_ok) {
                is_valid = true;
                pt.x = x_trans;
                pt.y = y_trans;
                pt.z = z_trans;

                const uint8_t* img_row = d_image + v * params.img_step;
                uint32_t rgb = 0;

                if (params.channels == 1) {
                    uint8_t gray = img_row[u];
                    rgb = (static_cast<uint32_t>(gray) << 16) |
                          (static_cast<uint32_t>(gray) << 8) |
                          static_cast<uint32_t>(gray);
                } else if (params.channels == 3) {
                    uint8_t c0 = img_row[u * 3 + 0];
                    uint8_t c1 = img_row[u * 3 + 1];
                    uint8_t c2 = img_row[u * 3 + 2];
                    if (params.is_rgb) {
                        rgb = (static_cast<uint32_t>(c0) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c2);
                    } else { // BGR
                        rgb = (static_cast<uint32_t>(c2) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c0);
                    }
                } else if (params.channels == 4) {
                    uint8_t c0 = img_row[u * 4 + 0];
                    uint8_t c1 = img_row[u * 4 + 1];
                    uint8_t c2 = img_row[u * 4 + 2];
                    if (params.is_rgb) {
                        rgb = (static_cast<uint32_t>(c0) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c2);
                    } else { // BGRA
                        rgb = (static_cast<uint32_t>(c2) << 16) |
                              (static_cast<uint32_t>(c1) << 8) |
                              static_cast<uint32_t>(c0);
                    }
                }
                pt.rgb = rgb;
            }
        }
    }

    // Warp-aggregated atomic compaction for maximum throughput
    unsigned int active_mask = __activemask();
    unsigned int valid_mask = __ballot_sync(active_mask, is_valid);

    if (is_valid) {
        int tid = threadIdx.y * blockDim.x + threadIdx.x;
        int lane = tid & 31;

        int rank = __popc(valid_mask & ((1u << lane) - 1));
        int leader_lane = __ffs(valid_mask) - 1;

        int warp_offset = 0;
        if (lane == leader_lane) {
            int warp_count = __popc(valid_mask);
            warp_offset = atomicAdd(d_out_count, static_cast<unsigned int>(warp_count));
        }
        warp_offset = __shfl_sync(valid_mask, warp_offset, leader_lane);

        int out_idx = warp_offset + rank;
        if (out_idx < static_cast<int>(max_points)) {
            d_out_points[out_idx] = pt;
        }
    }
}

CudaTriangulator::CudaTriangulator()
{
    init();
}

CudaTriangulator::~CudaTriangulator()
{
    cleanup();
}

bool CudaTriangulator::init()
{
    int device_count = 0;
    cudaError_t err = cudaGetDeviceCount(&device_count);
    if (err != cudaSuccess || device_count == 0) {
        available_ = false;
        return false;
    }

    int device_id = 0;
    cudaGetDevice(&device_id);
    cudaDeviceProp props;
    if (cudaGetDeviceProperties(&props, device_id) == cudaSuccess) {
        device_name_ = props.name;
    } else {
        device_name_ = "CUDA GPU";
    }

    cudaStream_t stream;
    err = cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking);
    if (err != cudaSuccess) {
        err = cudaStreamCreate(&stream);
    }
    if (err != cudaSuccess) {
        available_ = false;
        return false;
    }
    stream_ptr_ = stream;

    // Allocate the point counter as MAPPED memory.
    // The GPU writes via d_count_ (device alias), the CPU reads via h_count_mapped_
    // without any explicit D→H copy — the write goes directly to the host page.
    void* h_count_raw = nullptr;
    err = cudaHostAlloc(&h_count_raw, sizeof(unsigned int), cudaHostAllocMapped);
    if (err != cudaSuccess) {
        // Fallback: ordinary device allocation (loses the zero-copy benefit)
        err = cudaMalloc(&d_count_, sizeof(unsigned int));
        if (err != cudaSuccess) {
            cudaStreamDestroy(stream);
            stream_ptr_ = nullptr;
            available_ = false;
            return false;
        }
        h_count_mapped_ = nullptr;  // will use D→H copy path
    } else {
        h_count_mapped_ = reinterpret_cast<volatile unsigned int*>(h_count_raw);
        // Obtain the GPU-visible address of the same physical page
        err = cudaHostGetDevicePointer(
            reinterpret_cast<void**>(&d_count_), h_count_raw, 0);
        if (err != cudaSuccess) {
            // Fallback: free mapped alloc and use plain device memory
            cudaFreeHost(h_count_raw);
            h_count_mapped_ = nullptr;
            cudaMalloc(&d_count_, sizeof(unsigned int));
        }
    }

    available_ = true;
    return true;
}

void CudaTriangulator::cleanup()
{
    if (stream_ptr_) {
        cudaStreamSynchronize(static_cast<cudaStream_t>(stream_ptr_));
        cudaStreamDestroy(static_cast<cudaStream_t>(stream_ptr_));
        stream_ptr_ = nullptr;
    }
    if (d_disparity_) {
        cudaFree(d_disparity_);
        d_disparity_ = nullptr;
        d_disp_capacity_ = 0;
    }
    if (d_image_) {
        cudaFree(d_image_);
        d_image_ = nullptr;
        d_img_capacity_ = 0;
    }
    if (d_points_) {
        cudaFree(d_points_);
        d_points_ = nullptr;
        d_points_capacity_ = 0;
    }
    // Free mapped counter
    if (h_count_mapped_) {
        cudaFreeHost(const_cast<unsigned int*>(
            reinterpret_cast<volatile unsigned int*>(h_count_mapped_)));
        h_count_mapped_ = nullptr;
        d_count_ = nullptr;
    } else if (d_count_) {
        cudaFree(d_count_);
        d_count_ = nullptr;
    }
    // Free pinned input staging buffers
    if (h_pinned_disp_) {
        cudaFreeHost(h_pinned_disp_);
        h_pinned_disp_ = nullptr;
        h_pinned_disp_cap_ = 0;
    }
    if (h_pinned_img_) {
        cudaFreeHost(h_pinned_img_);
        h_pinned_img_ = nullptr;
        h_pinned_img_cap_ = 0;
    }
    // Free pinned output buffer
    if (h_pinned_points_) {
        cudaFreeHost(h_pinned_points_);
        h_pinned_points_ = nullptr;
        h_pinned_points_cap_ = 0;
    }
    available_ = false;
}

// Ensure GPU VRAM buffers are large enough
bool CudaTriangulator::ensure_device_buffers(size_t disp_bytes, size_t img_bytes, size_t max_points)
{
    if (!available_) return false;

    if (d_disp_capacity_ < disp_bytes) {
        if (d_disparity_) cudaFree(d_disparity_);
        size_t alloc_sz = disp_bytes + (disp_bytes / 4);
        if (cudaMalloc(&d_disparity_, alloc_sz) != cudaSuccess) {
            d_disparity_ = nullptr;
            d_disp_capacity_ = 0;
            return false;
        }
        d_disp_capacity_ = alloc_sz;
    }

    if (d_img_capacity_ < img_bytes) {
        if (d_image_) cudaFree(d_image_);
        size_t alloc_sz = img_bytes + (img_bytes / 4);
        if (cudaMalloc(&d_image_, alloc_sz) != cudaSuccess) {
            d_image_ = nullptr;
            d_img_capacity_ = 0;
            return false;
        }
        d_img_capacity_ = alloc_sz;
    }

    size_t points_bytes = max_points * sizeof(PointXYZRGB);
    if (d_points_capacity_ < points_bytes) {
        if (d_points_) cudaFree(d_points_);
        size_t alloc_sz = points_bytes + (points_bytes / 4);
        if (cudaMalloc(&d_points_, alloc_sz) != cudaSuccess) {
            d_points_ = nullptr;
            d_points_capacity_ = 0;
            return false;
        }
        d_points_capacity_ = alloc_sz;
    }

    return true;
}

// Ensure pinned (page-locked) host staging buffers for H→D input transfers
bool CudaTriangulator::ensure_pinned_input(size_t disp_bytes, size_t img_bytes)
{
    if (h_pinned_disp_cap_ < disp_bytes) {
        if (h_pinned_disp_) cudaFreeHost(h_pinned_disp_);
        size_t alloc_sz = disp_bytes + (disp_bytes / 4);
        if (cudaHostAlloc(&h_pinned_disp_, alloc_sz, cudaHostAllocDefault) != cudaSuccess) {
            h_pinned_disp_ = nullptr;
            h_pinned_disp_cap_ = 0;
            return false;
        }
        h_pinned_disp_cap_ = alloc_sz;
    }

    if (h_pinned_img_cap_ < img_bytes) {
        if (h_pinned_img_) cudaFreeHost(h_pinned_img_);
        size_t alloc_sz = img_bytes + (img_bytes / 4);
        if (cudaHostAlloc(&h_pinned_img_, alloc_sz, cudaHostAllocDefault) != cudaSuccess) {
            h_pinned_img_ = nullptr;
            h_pinned_img_cap_ = 0;
            return false;
        }
        h_pinned_img_cap_ = alloc_sz;
    }

    return true;
}

// Ensure pinned host output buffer for D→H result transfer
bool CudaTriangulator::ensure_pinned_output(size_t max_points)
{
    if (h_pinned_points_cap_ < max_points) {
        if (h_pinned_points_) cudaFreeHost(h_pinned_points_);
        size_t alloc_pts = max_points + (max_points / 4);
        if (cudaHostAlloc(&h_pinned_points_, alloc_pts * sizeof(PointXYZRGB),
                          cudaHostAllocDefault) != cudaSuccess) {
            h_pinned_points_ = nullptr;
            h_pinned_points_cap_ = 0;
            return false;
        }
        h_pinned_points_cap_ = alloc_pts;
    }
    return true;
}

size_t CudaTriangulator::triangulate(
    const float*   h_disparity,
    size_t         disp_bytes,
    const uint8_t* h_image,
    size_t         img_bytes,
    const TriangulationParams& params,
    size_t         max_points,
    PointXYZRGB**  h_points_out)
{
    if (!available_ || !h_disparity || !h_image || !h_points_out || max_points == 0) {
        if (h_points_out) *h_points_out = nullptr;
        return 0;
    }

    if (!ensure_device_buffers(disp_bytes, img_bytes, max_points) ||
        !ensure_pinned_input(disp_bytes, img_bytes) ||
        !ensure_pinned_output(max_points)) {
        if (h_points_out) *h_points_out = nullptr;
        return 0;
    }

    cudaStream_t stream = static_cast<cudaStream_t>(stream_ptr_);

    // 1. Stage inputs into pinned memory (fast CPU-side write, no page faults)
    std::memcpy(h_pinned_disp_, h_disparity, disp_bytes);
    std::memcpy(h_pinned_img_,  h_image,     img_bytes);

    // 2. Reset the mapped counter to 0 via the host pointer (no CUDA call needed)
    if (h_count_mapped_) {
        *const_cast<unsigned int*>(
            reinterpret_cast<volatile unsigned int*>(h_count_mapped_)) = 0u;
        // Fence: ensure the GPU sees the reset before the kernel runs
        __sync_synchronize();
    } else {
        cudaMemsetAsync(d_count_, 0, sizeof(unsigned int), stream);
    }

    // 3. Async H→D: pinned→device (true DMA, CPU is free immediately after call)
    cudaMemcpyAsync(d_disparity_, h_pinned_disp_, disp_bytes, cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_image_,     h_pinned_img_,  img_bytes,  cudaMemcpyHostToDevice, stream);

    // 4. Launch kernel
    int n_u = (params.u1 - params.u0 + params.step - 1) / params.step;
    int n_v = (params.v1 - params.v0 + params.step - 1) / params.step;

    dim3 block(16, 16);
    dim3 grid((n_u + block.x - 1) / block.x, (n_v + block.y - 1) / block.y);

    triangulate_kernel<<<grid, block, 0, stream>>>(
        d_disparity_,
        d_image_,
        params,
        d_points_,
        d_count_,
        max_points);

    // 5. If using mapped counter: just sync the stream; the count is already in host memory.
    //    If using plain device memory: also copy the count back.
    if (!h_count_mapped_) {
        unsigned int h_count = 0;
        cudaMemcpyAsync(&h_count, d_count_, sizeof(unsigned int), cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);

        size_t valid_points = std::min(static_cast<size_t>(h_count), max_points);
        if (valid_points > 0) {
            cudaMemcpyAsync(h_pinned_points_, d_points_,
                            valid_points * sizeof(PointXYZRGB), cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);
        }
        *h_points_out = h_pinned_points_;
        return valid_points;
    }

    // Mapped counter path: sync stream to flush kernel + point writes
    cudaStreamSynchronize(stream);

    // h_count_mapped_ now contains the final point count (written by the GPU directly)
    size_t valid_points = std::min(
        static_cast<size_t>(*h_count_mapped_), max_points);

    if (valid_points > 0) {
        // D→H: copy only the valid points into pinned output buffer
        cudaMemcpyAsync(h_pinned_points_, d_points_,
                        valid_points * sizeof(PointXYZRGB), cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
    }

    *h_points_out = h_pinned_points_;
    return valid_points;
}

} // namespace passive_stereo
