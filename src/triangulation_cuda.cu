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

    err = cudaMalloc(&d_count_, sizeof(unsigned int));
    if (err != cudaSuccess) {
        cudaStreamDestroy(stream);
        stream_ptr_ = nullptr;
        available_ = false;
        return false;
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
    if (d_count_) {
        cudaFree(d_count_);
        d_count_ = nullptr;
    }
    available_ = false;
}

bool CudaTriangulator::ensure_buffers(size_t disp_bytes, size_t img_bytes, size_t max_points)
{
    if (!available_) return false;

    if (d_disp_capacity_ < disp_bytes) {
        if (d_disparity_) cudaFree(d_disparity_);
        size_t alloc_sz = disp_bytes + (disp_bytes / 4); // +25% headroom
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

size_t CudaTriangulator::triangulate(
    const float* h_disparity,
    size_t disp_bytes,
    const uint8_t* h_image,
    size_t img_bytes,
    const TriangulationParams& params,
    PointXYZRGB* h_out_points,
    size_t max_points)
{
    if (!available_ || !h_disparity || !h_image || !h_out_points || max_points == 0) {
        return 0;
    }

    if (!ensure_buffers(disp_bytes, img_bytes, max_points)) {
        return 0;
    }

    cudaStream_t stream = static_cast<cudaStream_t>(stream_ptr_);

    // 1. Reset count to 0
    cudaMemsetAsync(d_count_, 0, sizeof(unsigned int), stream);

    // 2. Async copy inputs to device
    cudaMemcpyAsync(d_disparity_, h_disparity, disp_bytes, cudaMemcpyHostToDevice, stream);
    cudaMemcpyAsync(d_image_, h_image, img_bytes, cudaMemcpyHostToDevice, stream);

    // 3. Launch kernel
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

    // 4. Retrieve count
    unsigned int h_count = 0;
    cudaMemcpyAsync(&h_count, d_count_, sizeof(unsigned int), cudaMemcpyDeviceToHost, stream);

    // Synchronize to get the valid count
    cudaStreamSynchronize(stream);

    size_t valid_points = std::min(static_cast<size_t>(h_count), max_points);
    if (valid_points > 0) {
        // Copy only valid points back to host
        cudaMemcpyAsync(h_out_points, d_points_, valid_points * sizeof(PointXYZRGB), cudaMemcpyDeviceToHost, stream);
        cudaStreamSynchronize(stream);
    }

    return valid_points;
}

} // namespace passive_stereo
