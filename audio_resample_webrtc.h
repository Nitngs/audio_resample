#ifndef __AUDIO_RESAMPLE_WEBRTC_H__
#define __AUDIO_RESAMPLE_WEBRTC_H__

#ifdef __cplusplus
extern "C" {
#endif

// sample rate
#define SAMPLE_RATE_8K (8000)
#define SAMPLE_RATE_16K (16000)

typedef int bool;

typedef struct PushSincResampler
{
    float* float_buffer_;
    const float* source_ptr_;
    const short* source_ptr_int_;
    int destination_frames_;
    bool first_pass_;
    int source_available_;
}PushSincResampler;

typedef float (*ConvolveProc)(const float*,
                              const float*,
                              const float*,
                              double);

typedef void (*SincResamplerCallback)(int frames, float* destination);

typedef struct SincResampler
{
    const int kKernelSize;
    const int kKernelOffsetCount;
    const int kKernelStorageSize;
    double io_sample_rate_ratio_;
    double virtual_source_idx_;
    bool buffer_primed_;
    SincResamplerCallback read_cb_;
    int request_frames_;
    int block_size_;
    int input_buffer_size_;
    float* kernel_storage_;
    float* kernel_pre_sinc_storage_;
    float* kernel_window_storage_;
    float* input_buffer_;
    ConvolveProc convolve_proc_;
    float* r0_;
    float* r1_;
    float* r2_;
    float* r3_;
    float* r4_;
}SincResampler;


/**
 * @brief webrtc resampler init
 *
 * @param src_samplerate [in] source sample rate
 * @param dst_samplerate [in] dst sample rate
 * @param src_frames [in] source sample frames number
 * @return 0 if OK, others for error code.
 */
int audio_resample_webrtc_s16_init(int src_samplerate,
                                             int dst_samplerate);

/**
 * @brief webrtc resampler process
 *
 * @param source [in] source data buffer
 * @param source_length [in] sample number
 * @param destination [out] dest data buffer
 * @param destination_capacity [in] destination sample number
 * @return 0 if OK, others for error code.
 */
int audio_resample_webrtc_s16_process(const short* source,
                                                int source_length,
                                                short* destination,
                                                int destination_capacity);

/**
* @brief webrtc resampler deinit
*
* @return void.
*/
void audio_resample_webrtc_s16_deinit(void);

#ifdef __cplusplus
}
#endif
#endif /* __AUDIO_RESAMPLE_WEBRTC_H__ */
