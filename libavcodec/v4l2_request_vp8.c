/*
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#include "hwaccel.h"
#include "v4l2_request.h"
#include "vp8.h"
#include "vp8-ctrls.h"

typedef struct V4L2RequestControlsVP8 {
    struct v4l2_ctrl_vp8_frame_header frame_header;
} V4L2RequestControlsVP8;

static uint64_t v4l2_request_vp8_frame_ts(VP8Frame *vf)
{
    if (vf)
        return ff_v4l2_request_get_capture_timestamp(vf->tf.f);
    else
        return (uint64_t)-1;
}

static int v4l2_request_vp8_start_frame(AVCodecContext *avctx,
                                          av_unused const uint8_t *buffer,
                                          av_unused uint32_t size)
{
    const VP8Context *s = avctx->priv_data;
    V4L2RequestControlsVP8 *controls = s->framep[VP56_FRAME_CURRENT]->hwaccel_picture_private;
    struct v4l2_ctrl_vp8_frame_header *hdr = &controls->frame_header;
    struct v4l2_vp8_segment_header *segment_header = &hdr->segment_header;
    struct v4l2_vp8_loopfilter_header *lf_header = &hdr->lf_header;
    struct v4l2_vp8_entropy_header *entropy_header = &hdr->entropy_header;
    struct v4l2_vp8_quantization_header *quant_header = &hdr->quant_header;
    int i, j, k;

    hdr->key_frame = !s->keyframe;
    hdr->version = s->profile;
    hdr->width = avctx->width;
    hdr->height = avctx->height;
    hdr->horizontal_scale = 0;	
    hdr->vertical_scale = 0;
    hdr->flags = s->mbskip_enabled ? V4L2_VP8_FRAME_HDR_FLAG_MB_NO_SKIP_COEFF : 0;
    hdr->flags |= !s->invisible ? V4L2_VP8_FRAME_HDR_FLAG_SHOW_FRAME : 0;
    hdr->prob_skip_false = s->prob->mbskip;
    hdr->prob_intra = s->prob->intra;
    hdr->prob_last = s->prob->last;
    hdr->prob_gf = s->prob->golden;
    hdr->sign_bias_golden = s->sign_bias[VP56_FRAME_GOLDEN];
    hdr->sign_bias_alternate = s->sign_bias[VP56_FRAME_GOLDEN2];

    hdr->last_frame_ts = v4l2_request_vp8_frame_ts(s->framep[VP56_FRAME_PREVIOUS]);
    hdr->golden_frame_ts = v4l2_request_vp8_frame_ts(s->framep[VP56_FRAME_GOLDEN]);
    hdr->alt_frame_ts = v4l2_request_vp8_frame_ts(s->framep[VP56_FRAME_GOLDEN2]);

    // Segment header
    segment_header->segment_feature_mode = s->segmentation.absolute_vals;
    segment_header->flags = s->segmentation.enabled ? V4L2_VP8_SEGMNT_HDR_FLAG_ENABLED : 0;              
    segment_header->flags |= s->segmentation.update_map ? V4L2_VP8_SEGMNT_HDR_FLAG_UPDATE_MAP : 0;              
    segment_header->flags |= s->segmentation.update_feature_data ? V4L2_VP8_SEGMNT_HDR_FLAG_UPDATE_FEATURE_DATA : 0;

    for (i = 0; i < 3; i++)
        segment_header->segment_probs[i] = s->prob->segmentid[i];

    for (i = 0; i < 4; i++) {
        segment_header->quant_update[i] = s->segmentation.base_quant[i];
        segment_header->lf_update[i] = s->segmentation.filter_level[i];
    }

    /* Loop filter */
    lf_header->type = s->filter.simple;
    lf_header->level = s->filter.level;
    lf_header->sharpness_level = s->filter.sharpness;
    lf_header->flags = s->lf_delta.enabled ? V4L2_VP8_LF_HDR_ADJ_ENABLE : 0;
    lf_header->flags |= s->lf_delta.update ? V4L2_VP8_LF_HDR_DELTA_UPDATE : 0;
    for (i = 0; i < 4; i++) {
        lf_header->ref_frm_delta_magnitude[i] = s->lf_delta.ref[i];
        lf_header->mb_mode_delta_magnitude[i] = s->lf_delta.mode[i + MODE_I4x4];
    }

    // Probabilites
    if (s->keyframe) {
        static const uint8_t keyframe_y_mode_probs[4] = {
            145, 156, 163, 128
        };
        static const uint8_t keyframe_uv_mode_probs[3] = {
            142, 114, 183
        };
        memcpy(entropy_header->y_mode_probs,  keyframe_y_mode_probs,  4);
        memcpy(entropy_header->uv_mode_probs, keyframe_uv_mode_probs, 3);
    } else {
        for (i = 0; i < 4; i++)
            entropy_header->y_mode_probs[i] = s->prob->pred16x16[i];
        for (i = 0; i < 3; i++)
            entropy_header->uv_mode_probs[i] = s->prob->pred8x8c[i];
    }
    for (i = 0; i < 2; i++)
        for (j = 0; j < 19; j++)
            entropy_header->mv_probs[i][j] = s->prob->mvc[i][j];

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 8; j++) {
            static const int coeff_bands_inverse[8] = {
                0, 1, 2, 3, 5, 6, 4, 15
            };
            int coeff_pos = coeff_bands_inverse[j];

            for (k = 0; k < 3; k++) {
                memcpy(entropy_header->coeff_probs[i][j][k],
                       s->prob->token[i][coeff_pos][k], 11);
            }
        }
    }

    quant_header->y_ac_qi = s->quant.yac_qi;
    quant_header->y_dc_delta = s->quant.ydc_delta;
    quant_header->y2_dc_delta = s->quant.y2dc_delta;
    quant_header->y2_ac_delta = s->quant.y2ac_delta;
    quant_header->uv_dc_delta = s->quant.uvdc_delta;
    quant_header->uv_ac_delta = s->quant.uvac_delta;

    // dequant factors look like s->qmat (it's the same size),
    // but chromium is not setting it.

    // Boolean coder
    hdr->bool_dec_range = s->coder_state_at_header_end.range;
    hdr->bool_dec_value = s->coder_state_at_header_end.value;
    hdr->bool_dec_count = s->coder_state_at_header_end.bit_count;
    return 0;
}

static int v4l2_request_vp8_decode_slice(AVCodecContext *avctx, const uint8_t *buffer, uint32_t size)
{
    const VP8Context *s = avctx->priv_data;
    AVFrame *frame = s->framep[VP56_FRAME_CURRENT]->tf.f;
    V4L2RequestControlsVP8 *controls = s->framep[VP56_FRAME_CURRENT]->hwaccel_picture_private;
    struct v4l2_ctrl_vp8_frame_header *hdr = &controls->frame_header;
    unsigned int header_size = 3 + 7 * s->keyframe;
    const uint8_t *data = buffer + header_size;
    int i;

    hdr->macroblock_bit_offset = (8 * (s->coder_state_at_header_end.input - data) -
                              s->coder_state_at_header_end.bit_count - 8),
    hdr->first_part_size = s->header_partition_size;
    hdr->first_part_offset = header_size;
    hdr->num_dct_parts = s->num_coeff_partitions;
    for (i = 0; i < hdr->num_dct_parts; i++)
        hdr->dct_part_sizes[i] = s->coeff_partition_size[i];

    return ff_v4l2_request_append_output_buffer(avctx, frame, buffer, size);
}

static int v4l2_request_vp8_end_frame(AVCodecContext *avctx)
{
    const VP8Context *s = avctx->priv_data;
    AVFrame *frame = s->framep[VP56_FRAME_CURRENT]->tf.f;
    V4L2RequestControlsVP8 *controls = s->framep[VP56_FRAME_CURRENT]->hwaccel_picture_private;

    struct v4l2_ext_control control[] = {
        {
            .id = V4L2_CID_MPEG_VIDEO_VP8_FRAME_HDR,
            .ptr = &controls->frame_header,
            .size = sizeof(controls->frame_header),
        },
    };

    return ff_v4l2_request_decode_frame(avctx, frame, control, FF_ARRAY_ELEMS(control));
}

static int v4l2_request_vp8_init(AVCodecContext *avctx)
{
    return ff_v4l2_request_init(avctx, V4L2_PIX_FMT_VP8_FRAME, 1024 * 1024, NULL, 0);
}

const AVHWAccel ff_vp8_v4l2request_hwaccel = {
    .name           = "vp8_v4l2request",
    .type           = AVMEDIA_TYPE_VIDEO,
    .id             = AV_CODEC_ID_VP8,
    .pix_fmt        = AV_PIX_FMT_DRM_PRIME,
    .start_frame    = v4l2_request_vp8_start_frame,
    .decode_slice   = v4l2_request_vp8_decode_slice,
    .end_frame      = v4l2_request_vp8_end_frame,
    .frame_priv_data_size = sizeof(V4L2RequestControlsVP8),
    .init           = v4l2_request_vp8_init,
    .uninit         = ff_v4l2_request_uninit,
    .priv_data_size = sizeof(V4L2RequestContext),
    .frame_params   = ff_v4l2_request_frame_params,
    .caps_internal  = HWACCEL_CAP_ASYNC_SAFE,
};
