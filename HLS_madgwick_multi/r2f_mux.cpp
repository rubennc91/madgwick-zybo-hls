#include "r2f_mux.h"

// Ver r2f_mux.h. Escalado identico a raw2float_top (HLS_raw2float).
static void send_frame(hls::stream<ap_axis_raw_t>& s, ap_axis_raw_t first,
                       hls::stream<ap_axis_fid_t>& m, ap_uint<NIMU_IDW> id, float gyro_scale) {
    const float ACCMAG_SCALE = (1.0f / 16384.0f);
    ap_axis_raw_t in_word = first;
    for (int i = 0; i < 9; i++) {
#pragma HLS PIPELINE II=1
        if (i > 0) in_word = s.read();
        int32_t raw = (int32_t)in_word.data;
        float scale = (i < 3) ? gyro_scale : ACCMAG_SCALE;
        ap_axis_fid_t out_word;
        out_word.data = (float)raw * scale;
        out_word.last = (i == 8) ? 1 : 0;
        out_word.keep = -1;
        out_word.id   = id;
        m.write(out_word);
    }
}

void r2f_mux_top(hls::stream<ap_axis_raw_t>& s_axis0,
                 hls::stream<ap_axis_raw_t>& s_axis1,
                 hls::stream<ap_axis_raw_t>& s_axis2,
                 hls::stream<ap_axis_raw_t>& s_axis3,
                 hls::stream<ap_axis_fid_t>& m_axis,
                 ap_uint<2> fs_g_reg) {
#pragma HLS INTERFACE axis    port=s_axis0
#pragma HLS INTERFACE axis    port=s_axis1
#pragma HLS INTERFACE axis    port=s_axis2
#pragma HLS INTERFACE axis    port=s_axis3
#pragma HLS INTERFACE axis    port=m_axis
#pragma HLS INTERFACE s_axilite port=fs_g_reg bundle=ctrl offset=0x10
#pragma HLS INTERFACE s_axilite port=return    bundle=ctrl

    static ap_uint<2> next = 0;           // primer puerto que se mira en esta pasada

    float gyro_scale;
    switch (fs_g_reg.to_uint()) {
        case 0:  gyro_scale = 8.75e-3f  * 0.017453293f; break; // 245 dps
        case 1:  gyro_scale = 17.50e-3f * 0.017453293f; break; // 500 dps
        default: gyro_scale = 70.0e-3f  * 0.017453293f; break; // 2000 dps
    }

    ap_axis_raw_t w;
    for (int n = 0; n < 4; n++) {
        const ap_uint<2> p = next + n;    // aritmetica modulo 4 (2 bits)
        bool got = false;
        switch (p.to_uint()) {
            case 0:  got = s_axis0.read_nb(w); break;
            case 1:  got = s_axis1.read_nb(w); break;
            case 2:  got = s_axis2.read_nb(w); break;
            default: got = s_axis3.read_nb(w); break;
        }
        if (got) {
            switch (p.to_uint()) {
                case 0:  send_frame(s_axis0, w, m_axis, p, gyro_scale); break;
                case 1:  send_frame(s_axis1, w, m_axis, p, gyro_scale); break;
                case 2:  send_frame(s_axis2, w, m_axis, p, gyro_scale); break;
                default: send_frame(s_axis3, w, m_axis, p, gyro_scale); break;
            }
            next = p + 1;                 // la siguiente pasada empieza por el puerto siguiente
            break;
        }
    }
}
