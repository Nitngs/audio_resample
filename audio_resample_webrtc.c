// #include <iostream>
#include <stdlib.h>
#include <string.h> /* memcpy */
#include <math.h>
// #include <cmath.h>

#include "audio_resample_webrtc.h"

// webrtc resample
#define RESAMPLE_M_PI (3.14159265358979323846) /* pi */
#define FRAMES_8K (320)
#define FRAMES_16K (640)


static SincResampler g_SincResampler = {
    .kKernelSize = 32,
    .kKernelOffsetCount = 32,
    .kKernelStorageSize = 1056,
};

static PushSincResampler g_PushSincResampler = {0};


static float Convolve_C(const float *input_ptr, const float *k1, const float *k2, double kernel_interpolation_factor)
{
    float sum1 = 0;
    float sum2 = 0;

    // Generate a single output sample.  Unrolling this loop hurt performance in local testing.
    int n = g_SincResampler.kKernelSize;
    while (n--)
    {
        sum1 += *input_ptr * *k1++;
        sum2 += *input_ptr++ * *k2++;
    }

    // Linearly interpolate the two "convolutions".
    return (float)((1.0 - kernel_interpolation_factor) * sum1 + kernel_interpolation_factor * sum2);
}

static void webrtcUpdateRegions(bool second_load)
{
    g_SincResampler.r3_ = g_SincResampler.r0_ + g_SincResampler.request_frames_ - g_SincResampler.kKernelSize;
    g_SincResampler.r4_ = g_SincResampler.r0_ + g_SincResampler.request_frames_ - g_SincResampler.kKernelSize / 2;
    g_SincResampler.block_size_ = g_SincResampler.r4_ - g_SincResampler.r2_;

    // r1_ at the beginning of the buffer.
    if (g_SincResampler.r1_ != g_SincResampler.input_buffer_)
    {
        printf("webrtcUpdateRegions, r1_ is not at the beginning of the buffer: r1_=%p, input_buffer_=%p\n", g_SincResampler.r1_, g_SincResampler.input_buffer_);
        // return;
    }

    // r1_ left of r2_, r4_ left of r3_ and size correct.
    if ((g_SincResampler.r2_ - g_SincResampler.r1_) != (g_SincResampler.r4_ - g_SincResampler.r3_))
    {
        printf("webrtcUpdateRegions, (r2_ - r1_) != (r4_ - r3_).\n");
        // return;
    }

    // r2_ left of r3.
    if (g_SincResampler.r2_ >= g_SincResampler.r3_)
    {
        printf("webrtcUpdateRegions, r2_ >= r3_ .\n");
        // return;
    }

    return;
}

static void webrtcFlush(void)
{
    g_SincResampler.virtual_source_idx_ = 0;
    memset(g_SincResampler.input_buffer_, 0x0, sizeof(g_SincResampler.input_buffer_[0]) * g_SincResampler.input_buffer_size_);
    webrtcUpdateRegions(0);
    return;
}

static double webrtcSincScaleFactor(double io_ratio)
{
    // `sinc_scale_factor` is basically the normalized cutoff frequency of the low-pass filter.
    double sinc_scale_factor = io_ratio > 1.0 ? 1.0 / io_ratio : 1.0;
    sinc_scale_factor *= 0.9;
    return sinc_scale_factor;
}

static void webrtcInitializeKernel(void)
{
    // Blackman window parameters.
    static const double kAlpha = 0.16;
    const double kA0 = 0.5 * (1.0 - kAlpha);
    static const double kA1 = 0.5;
    const double kA2 = 0.5 * kAlpha;

    // Generates a set of windowed sinc() kernels.
    // We generate a range of sub-sample offsets from 0.0 to 1.0.
    const double sinc_scale_factor = webrtcSincScaleFactor(g_SincResampler.io_sample_rate_ratio_);

    int offset_idx = 0, i = 0;
    for (offset_idx = 0; offset_idx <= g_SincResampler.kKernelOffsetCount; ++offset_idx)
    {
        const float subsample_offset = (float)(offset_idx) / g_SincResampler.kKernelOffsetCount;

        for (i = 0; i < g_SincResampler.kKernelSize; ++i)
        {
            const int idx = i + offset_idx * g_SincResampler.kKernelSize;
            const float pre_sinc = (float)(RESAMPLE_M_PI * ((int)(i) - (int)(g_SincResampler.kKernelSize / 2) - subsample_offset));
            g_SincResampler.kernel_pre_sinc_storage_[idx] = pre_sinc;

            // Compute Blackman window, matching the offset of the sinc().
            const float x = (i - subsample_offset) / g_SincResampler.kKernelSize;
            const float window = (float)(kA0 - kA1 * cos(2.0 * RESAMPLE_M_PI * x) + kA2 * cos(4.0 * RESAMPLE_M_PI * x));
            g_SincResampler.kernel_window_storage_[idx] = window;

            // Compute the sinc with offset, then window the sinc() function and store at the correct offset.
            g_SincResampler.kernel_storage_[idx] =(float)(window * ((fabs(pre_sinc - 0) < 0.000001) ? sinc_scale_factor : (sin(sinc_scale_factor * pre_sinc) / pre_sinc)));
        }
    }

    return;
}

static void pushSincResampleRun(int frames, float *destination)
{
    // Ensure we are only asked for the available samples. This would fail if
    // Run() was triggered more than once per Resample() call.

    // RTC_CHECK_EQ(source_available_, frames);
    if (g_PushSincResampler.source_available_ < frames)
    {
        printf("pushSincResampleRun error, frames=%d is not equal to g_PushSincResampler.source_available_=%d\n", frames, g_PushSincResampler.source_available_);
        return;
    }

    if (g_PushSincResampler.source_ptr_)
    {
        memcpy(destination, g_PushSincResampler.source_ptr_, frames * sizeof(*destination));
    }
    else
    {
        int i = 0;
        for (i = 0; i < frames; ++i)
        {
            destination[i] = (float)(g_PushSincResampler.source_ptr_int_[i]);
        }
    }

    printf("pushSincResampleRun-1: frames=%d, g_PushSincResampler.source_available_=%d\n", frames, g_PushSincResampler.source_available_);
    g_PushSincResampler.source_available_ -= frames;
    return;
}

int audio_resample_webrtc_s16_init(int src_samplerate, int dst_samplerate)
{
    if ((src_samplerate <= 0) || (dst_samplerate <= 0))
    {
        printf("src_samplerate=%d, dst_samplerate=%d, is invalid, please check\n", src_samplerate, dst_samplerate);
        return -1;
    }

    if (src_samplerate == dst_samplerate)
    {
        printf("src_samplerate=%d, dst_samplerate=%d, same rate, no need to resample\n", src_samplerate, dst_samplerate);
        return -1;
    }

    printf("webrtc_s16_init-1: src_samplerate=%d, dst_samplerate=%d\n", src_samplerate, dst_samplerate);

    // SincResampler constructor
    g_SincResampler.io_sample_rate_ratio_ = src_samplerate * 1.0 / dst_samplerate;
    g_SincResampler.read_cb_ = pushSincResampleRun;
    g_SincResampler.request_frames_ = 0;
    g_SincResampler.input_buffer_size_ = 2048+32;
    g_SincResampler.block_size_ = 0;

    g_SincResampler.kernel_storage_ = (float *)calloc(g_SincResampler.kKernelStorageSize, sizeof(float));
    if (NULL == g_SincResampler.kernel_storage_)
    {
        printf("g_SincResampler.kernel_storage_ calloc failed.\n");
        return -1;
    }

    g_SincResampler.kernel_pre_sinc_storage_ = (float *)calloc(g_SincResampler.kKernelStorageSize, sizeof(float));
    if (NULL == g_SincResampler.kernel_pre_sinc_storage_)
    {
        printf("g_SincResampler.kernel_pre_sinc_storage_ calloc failed.\n");
        return -1;
    }

    g_SincResampler.kernel_window_storage_ = (float *)calloc(g_SincResampler.kKernelStorageSize, sizeof(float));
    if (NULL == g_SincResampler.kernel_window_storage_)
    {
        printf("g_SincResampler.kernel_window_storage_ calloc failed.\n");
        return -1;
    }

    g_SincResampler.input_buffer_ = (float *)calloc(g_SincResampler.input_buffer_size_, sizeof(float));
    if (NULL == g_SincResampler.input_buffer_)
    {
        printf("g_SincResampler.input_buffer_ calloc failed.\n");
        return -1;
    }

    g_SincResampler.convolve_proc_ = Convolve_C;

    g_SincResampler.r0_ = g_SincResampler.input_buffer_ + g_SincResampler.kKernelSize;
    g_SincResampler.r1_ = g_SincResampler.input_buffer_;
    g_SincResampler.r2_ = g_SincResampler.input_buffer_ + g_SincResampler.kKernelSize / 2;
    g_SincResampler.r3_ = NULL;
    g_SincResampler.r4_ = NULL;

    webrtcFlush();

    webrtcInitializeKernel();

    return 0;
}

static void sincResample(int frames, float *destination)
{
    int remaining_frames = frames;

    // Step (2) -- Resample!  const what we can outside of the loop for speed.  It actually has an impact on ARM performance.  See inner loop comment below.
    const double current_io_ratio = g_SincResampler.io_sample_rate_ratio_;
    const float *const kernel_ptr = g_SincResampler.kernel_storage_;

    int whileIdx = 0;
    while (remaining_frames)
    {
        whileIdx++;
        printf("sincResample-2: whileIdx=%d: block_size_=%lu, virtual_source_idx_=%f, current_io_ratio=%f\n", whileIdx, g_SincResampler.block_size_, g_SincResampler.virtual_source_idx_, current_io_ratio);

        int i = 0;
        for (i = (ceil((g_SincResampler.block_size_ - g_SincResampler.virtual_source_idx_) / current_io_ratio)); i > 0; --i)
        {
            if (g_SincResampler.virtual_source_idx_ >= g_SincResampler.block_size_)
            {
                printf("sincResample, virtual_source_idx_=%f is large than block_size_=%d.\n", g_SincResampler.virtual_source_idx_, g_SincResampler.block_size_);
                return;
            }

            // `virtual_source_idx_` lies in between two kernel offsets so figure out what they are.
            const int source_idx = (int)(g_SincResampler.virtual_source_idx_);
            const double subsample_remainder = g_SincResampler.virtual_source_idx_ - source_idx;

            const double virtual_offset_idx = subsample_remainder * g_SincResampler.kKernelOffsetCount;
            const int offset_idx = (int)(virtual_offset_idx);

            // We'll compute "convolutions" for the two kernels which straddle `virtual_source_idx_`.
            const float *const k1 = kernel_ptr + offset_idx * g_SincResampler.kKernelSize;
            const float *const k2 = k1 + g_SincResampler.kKernelSize;

            // Initialize input pointer based on quantized `virtual_source_idx_`.
            const float *const input_ptr = g_SincResampler.r1_ + source_idx;

            // Figure out how much to weight each kernel's "convolution".
            const double kernel_interpolation_factor = virtual_offset_idx - offset_idx;

            *destination++ = g_SincResampler.convolve_proc_(input_ptr, k1, k2, kernel_interpolation_factor);

            // Advance the virtual index.
            g_SincResampler.virtual_source_idx_ += current_io_ratio;

            if (!--remaining_frames)
            {
                return;
            }
        }

        // Wrap back around to the start.
        g_SincResampler.virtual_source_idx_ -= g_SincResampler.block_size_;

        // Step (3) -- Copy r3_, r4_ to r1_, r2_.
        // This wraps the last input frames back to the start of the buffer.
        memcpy(g_SincResampler.r1_, g_SincResampler.r3_, sizeof(g_SincResampler.input_buffer_[0]) * g_SincResampler.kKernelSize);

        // Step (5) -- Refresh the buffer with more input.
        g_SincResampler.read_cb_(g_SincResampler.request_frames_, g_SincResampler.r0_);

        webrtcUpdateRegions(1);
    }

    return;
}

static int pushSincResampleFloat(const float *source, int source_length, float *destination, int destination_capacity)
{
    if (g_SincResampler.input_buffer_size_ < source_length)
    {
        printf("input buffer size:%d not enough:%d", g_SincResampler.input_buffer_size_, source_length);
        return -1;
    }

    g_SincResampler.request_frames_ = source_length;

    g_PushSincResampler.source_ptr_ = source;
    g_PushSincResampler.source_available_ = source_length;

    printf("pushSincResampleFloat-1: before sincResample, source_length=%lu, destination_capacity=%lu\n", source_length, destination_capacity);
    sincResample(destination_capacity, destination);
    g_PushSincResampler.source_ptr_ = NULL;

    return g_PushSincResampler.destination_frames_;
}

static short FloatS16ToS16_single(float v)
{
    v = (v > 32767.f) ? 32767.f : v;
    v = (v < -32768.f) ? -32768.f : v;
    return (short)(v + copysign(0.5f, v));
}

static void FloatS16ToS16(const float *src, int size, short *dest)
{
    int i = 0;
    for (i = 0; i < size; ++i)
    {
        dest[i] = FloatS16ToS16_single(src[i]);
    }
    return;
}

int audio_resample_webrtc_s16_process(const short *source,
                                        int source_length,
                                        short *destination,
                                        int destination_capacity)
{
    if ((NULL == source) || (NULL == destination) || (source_length <= 0) || (destination_capacity <= 0))
    {
        printf("pps_media_audio_resample_webrtc_s16_process, invalid parametes, please check.\n");
        return -1;
    }

    if (NULL == g_PushSincResampler.float_buffer_)
    {
        g_PushSincResampler.float_buffer_ = (float *)calloc(2048, sizeof(float));
    }

    g_PushSincResampler.source_ptr_int_ = source;

    pushSincResampleFloat(NULL, source_length, g_PushSincResampler.float_buffer_, destination_capacity);
    printf("pps_media_audio_resample_webrtc_s16_process: after pushSincResampleFloat\n");
    FloatS16ToS16(g_PushSincResampler.float_buffer_, destination_capacity, destination);
    printf("pps_media_audio_resample_webrtc_s16_process: after FloatS16ToS16\n");
    g_PushSincResampler.source_ptr_int_ = NULL;

    return 0;
}

void audio_resample_webrtc_s16_deinit(void)
{
    // g_PushSincResampler
    if (NULL != g_PushSincResampler.float_buffer_)
    {
        free(g_PushSincResampler.float_buffer_);
        g_PushSincResampler.float_buffer_ = NULL;
    }

    // g_SincResampler
    if (NULL != g_SincResampler.input_buffer_)
    {
        free(g_SincResampler.input_buffer_);
        g_SincResampler.input_buffer_ = NULL;
    }

    if (NULL != g_SincResampler.kernel_window_storage_)
    {
        free(g_SincResampler.kernel_window_storage_);
        g_SincResampler.kernel_window_storage_ = NULL;
    }

    if (NULL != g_SincResampler.kernel_pre_sinc_storage_)
    {
        free(g_SincResampler.kernel_pre_sinc_storage_);
        g_SincResampler.kernel_pre_sinc_storage_ = NULL;
    }

    if (NULL != g_SincResampler.kernel_storage_)
    {
        free(g_SincResampler.kernel_storage_);
        g_SincResampler.kernel_storage_ = NULL;
    }

    return;
}
