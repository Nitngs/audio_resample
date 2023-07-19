#include <iostream>
#include <iomanip>
// #include <math.h>
#include <cmath>
using namespace std;

#include "audio_resample_webrtc.h"

static int read_len[3] = {640, 1024, 2048};

int main()
{
    FILE *infile = fopen("lpcm.pcm", "rb");
    FILE *outfile = fopen("resample.pcm", "wb");
    int len;
    int idx = 0;
    char file_buf[2048] = {0};
    int read_cnt;
    char *resample_buff = (char *)malloc(8192);
    int totle_len = 0;

    audio_resample_webrtc_s16_init(8000, 16000);
    int cyc_idx = 0;
    do
    {
        cyc_idx++;
        read_cnt = read_len[idx++%3];
        len = fread(file_buf, 1, read_cnt, infile);
        if (len <= 0)
        {
            break;
        }

        totle_len += len;
        printf("read len:%d, totle len:%d, cyc idx:%d\n", len, totle_len, cyc_idx);

        audio_resample_webrtc_s16_process((const short *)file_buf, len>>1, (short *)resample_buff, len);

        fwrite(resample_buff, len<<1, 1, outfile);
        fflush(outfile);
    } while (true);

    audio_resample_webrtc_s16_deinit();
    fclose(infile);
    fclose(outfile);

    return 0;
}
