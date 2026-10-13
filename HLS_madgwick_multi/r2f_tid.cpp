#include "r2f_tid.h"

// Ver r2f_tid.h. Escalado identico a raw2float_top (HLS_raw2float) y a r2f_mux_top.
void r2f_tid_top(hls::stream<ap_axis_rawid_t>& s_axis,
                 hls::stream<ap_axis_fid_t>& m_axis,
                 ap_uint<2> fs_g_reg) {
#pragma HLS INTERFACE axis    port=s_axis
#pragma HLS INTERFACE axis    port=m_axis
#pragma HLS INTERFACE s_axilite port=fs_g_reg bundle=ctrl offset=0x10
#pragma HLS INTERFACE s_axilite port=return    bundle=ctrl

    const float ACCMAG_SCALE = (1.0f / 16384.0f);
    float gyro_scale;
    switch (fs_g_reg.to_uint()) {
        case 0:  gyro_scale = 8.75e-3f  * 0.017453293f; break; // 245 dps
        case 1:  gyro_scale = 17.50e-3f * 0.017453293f; break; // 500 dps
        default: gyro_scale = 70.0e-3f  * 0.017453293f; break; // 2000 dps
    }

    for (int i = 0; i < 9; i++) {
#pragma HLS PIPELINE II=1
        ap_axis_rawid_t in_word = s_axis.read();
        int32_t raw = (int32_t)in_word.data;
        float scale = (i < 3) ? gyro_scale : ACCMAG_SCALE;
        ap_axis_fid_t out_word;
        out_word.data = (float)raw * scale;
        out_word.last = (i == 8) ? 1 : 0;
        out_word.keep = -1;
        out_word.id   = in_word.id;
        m_axis.write(out_word);
    }
}
