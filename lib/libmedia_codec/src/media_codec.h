#ifndef __RM_MEDIA_CODEC_H__
#define __RM_MEDIA_CODEC_H__


extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/mem.h>
#include <libavutil/channel_layout.h>
#include <libavutil/samplefmt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
#include <libswresample/swresample.h>
}

#include <errno.h>
#include <stdio.h>
#include <stdexcept>
#include <string>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>
#include <pybind11/pybind11.h>

#ifndef PIX_FMT_BGR24
#define PIX_FMT_BGR24 AV_PIX_FMT_BGR24
#endif

#ifndef PIX_FMT_RGB24
#define PIX_FMT_RGB24 AV_PIX_FMT_RGB24
#endif

// The truncated-input flags were removed in FFmpeg 6.0.
#if !defined(CODEC_CAP_TRUNCATED) && defined(AV_CODEC_CAP_TRUNCATED)
#define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
#endif

#if !defined(CODEC_FLAG_TRUNCATED) && defined(AV_CODEC_FLAG_TRUNCATED)
#define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
#endif

#if (LIBAVCODEC_VERSION_MAJOR <= 54)
#  define av_frame_alloc avcodec_alloc_frame
#  define av_frame_free  avcodec_free_frame
#endif

// FFmpeg 5.0 (libavcodec 59) made the decoder lookups return const pointers.
#if LIBAVCODEC_VERSION_MAJOR >= 59
using AVCodecPtr = const AVCodec *;
#else
using AVCodecPtr = AVCodec *;
#endif

// FFmpeg 6.0 (libavutil 58) replaced AVFrame/AVCodecContext channels and
// channel_layout with AVChannelLayout, and swr_alloc_set_opts() with
// swr_alloc_set_opts2(); FFmpeg 7.0 removed the old spellings outright.
#if LIBAVUTIL_VERSION_MAJOR >= 58
#define RM_HAVE_CH_LAYOUT 1
#endif

#ifdef _MSC_VER
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

using ubyte = unsigned char;
namespace py = pybind11;


class CodecException : public std::runtime_error {
public:
    CodecException(const char* s) : std::runtime_error(s) {}
};

class H264Decoder {
private:
    AVCodecContext        *context;
    AVFrame               *frame;
    AVCodecPtr             codec;
    AVCodecParserContext  *parser;
    AVPacket              *pkt;

public:
    H264Decoder() {
        // Codec registration became implicit in FFmpeg 4.0; the call was
        // removed in 5.0.
        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!codec)
            throw CodecException("H264Decoder: avcodec_find_decoder failed!");

        context = avcodec_alloc_context3(codec);
        if (!context)
            throw CodecException("H264Decoder: avcodec_alloc_context3 failed!");

#if defined(CODEC_CAP_TRUNCATED) && defined(CODEC_FLAG_TRUNCATED)
        if (codec->capabilities & CODEC_CAP_TRUNCATED) {
            context->flags |= CODEC_FLAG_TRUNCATED;
        }
#endif

        int err = avcodec_open2(context, codec, nullptr);
        if (err < 0)
            throw CodecException("H264Decoder: avcodec_open2 failed!");

        parser = av_parser_init(AV_CODEC_ID_H264);
        if (!parser)
            throw CodecException("H264Decoder: av_parser_init failed!");

        frame = av_frame_alloc();
        if (!frame)
            throw CodecException("H264Decoder: av_frame_alloc failed!");

        pkt = av_packet_alloc();
        if (!pkt)
            throw CodecException("H264Decoder: av_packet_alloc failed!");
    }

    ~H264Decoder() {
        av_parser_close(parser);
        avcodec_free_context(&context);
        av_frame_free(&frame);
        av_packet_free(&pkt);
    }

    ssize_t parse(const unsigned char* in_data, ssize_t in_size) {
        auto nread = av_parser_parse2(parser, context, &pkt->data, &pkt->size,
                                      in_data, in_size,
                                      0, 0, AV_NOPTS_VALUE);
        return nread;
    }

    bool is_frame_available() const {
        return pkt->size > 0;
    }

    const AVFrame& decode_frame() {
        // avcodec_decode_video2() was removed in FFmpeg 5.0; send/receive is
        // the equivalent and goes back to FFmpeg 3.1.
        int sent = avcodec_send_packet(context, pkt);
        if (sent < 0 && sent != AVERROR(EAGAIN))
            throw CodecException("H264Decoder: decode_frame, avcodec_send_packet failed!");

        int ret = avcodec_receive_frame(context, frame);
        if (sent == AVERROR(EAGAIN)) {
            // The decoder refused the packet until its output was drained, so
            // hand it over now that a frame has been taken out.
            avcodec_send_packet(context, pkt);
        }
        if (ret < 0)
            throw CodecException("H264Decoder: decode_frame, avcodec_receive_frame failed!");
        return *frame;
    }
};


class FormatConverter {
private:
    SwsContext *context_;
    AVFrame *output_frame_;
    AVPixelFormat output_format_;

public:
    FormatConverter(enum AVPixelFormat output_format) {
        output_format_ = output_format;
        output_frame_ = av_frame_alloc();
        if (!output_frame_)
            throw CodecException("FormatConverter: av_frame_alloc failed!");
        context_ = nullptr;
    }

    ~FormatConverter() {
        sws_freeContext(context_);
        av_frame_free(&output_frame_);
    }

    int predict_size(int w, int h) {
        // AVPicture and avpicture_fill() were removed in FFmpeg 5.0; align=1
        // reproduces the tightly packed layout avpicture_fill() gave.
        return av_image_get_buffer_size(output_format_, w, h, 1);
    }

    const AVFrame& convert(const AVFrame &frame, unsigned char* out_bgr) {
        int w = frame.width;
        int h = frame.height;
        int pix_fmt = frame.format;
        context_ = sws_getCachedContext(context_,
                                        w, h, (AVPixelFormat)pix_fmt,
                                        w, h, output_format_, SWS_BILINEAR,
                                        nullptr, nullptr, nullptr);
        if (!context_)
            throw CodecException("FormatConverter: convert, sws_getCachedContext failed!");

        av_image_fill_arrays(output_frame_->data, output_frame_->linesize, out_bgr,
                            output_format_, w, h, 1);

        sws_scale(context_, frame.data, frame.linesize, 0, h,
                  output_frame_->data, output_frame_->linesize);
        output_frame_->width = w;
        output_frame_->height = h;
        return *output_frame_;
    }
};

void disable_logging() {
    av_log_set_level(AV_LOG_QUIET);
}

std::pair<int, int> width_height(const AVFrame& f) {
    return std::make_pair(f.width, f.height);
}

int row_size(const AVFrame& f) {
    return f.linesize[0];
}


class PyH264Decoder {
public:
    std::unique_ptr<H264Decoder> decoder;
    std::unique_ptr<FormatConverter> converter;

    py::tuple decode_frame_impl(const ubyte *data_in, ssize_t len, ssize_t &num_consumed, bool &is_frame_available) {
        py::gil_scoped_release decode_release;
        num_consumed = decoder->parse((ubyte*)data_in, len);

        if (is_frame_available = decoder->is_frame_available()) {
            const auto &frame = decoder->decode_frame();
            int w, h; std::tie(w,h) = width_height(frame);
            Py_ssize_t out_size = converter->predict_size(w,h);

            py::gil_scoped_acquire decode_acquire;
            py::object py_out_str = py::reinterpret_steal<py::object>(PYBIND11_BYTES_FROM_STRING_AND_SIZE(NULL, out_size));
            char* out_buffer = PYBIND11_BYTES_AS_STRING(py_out_str.ptr());

            py::gil_scoped_release convert_release;
            const auto &out_frame = converter->convert(frame, (ubyte*)out_buffer);

            py::gil_scoped_acquire convert_acquire;
            return py::make_tuple(py_out_str, w, h, row_size(out_frame));
        }
        else {
            py::gil_scoped_acquire decode_acquire;
            return py::make_tuple(py::none(), 0, 0, 0);
        }
    }

public:
    PyH264Decoder(std::string output_format, bool verbose) {
        decoder = std::unique_ptr<H264Decoder>(new H264Decoder());
        if (output_format == "RGB") {
            converter = std::unique_ptr<FormatConverter>(new FormatConverter(PIX_FMT_RGB24));
        }
        else if (output_format == "BGR") {
            converter = std::unique_ptr<FormatConverter>(new FormatConverter(PIX_FMT_BGR24));
        }
        else {
            converter = std::unique_ptr<FormatConverter>(new FormatConverter(PIX_FMT_BGR24));
        }
        if (verbose) {
            disable_logging();
        }
    }

    ~PyH264Decoder() = default;

    py::list decode(const py::bytes &input) {
        ssize_t len = PYBIND11_BYTES_SIZE(input.ptr());
        const ubyte* data_in = (const ubyte*)(PYBIND11_BYTES_AS_STRING(input.ptr()));

        py::list out;

        try {
            while (len > 0) {
                ssize_t num_consumed = 0;
                bool is_frame_available = false;

                try {
                    auto frame = decode_frame_impl(data_in, len, num_consumed, is_frame_available);
                    if (is_frame_available)
                        out.append(frame);
                }
                catch (const CodecException &e) {
                    if (num_consumed <= 0) throw e;
                }
                len -= num_consumed;
                data_in += num_consumed;
            }
        }
        catch (const CodecException &e) {}
        return out;
    }
};


// The vendored opus-share package ships no MSVC import library (and no DLL), so
// the Opus stream is decoded through libavcodec instead. Both the native "opus"
// decoder and the "libopus" wrapper are handled; the output stays raw
// little-endian interleaved s16, as the libopus version produced.
class PyOpusDecoder {
public:
    PyOpusDecoder(int frame_size, int sample_rate, int channels):
            FRAME_SIZE(frame_size), SAMPLE_RATE(sample_rate), CHANNELS(channels) {
        AVCodecPtr codec = avcodec_find_decoder_by_name("libopus");
        if (!codec) {
            codec = avcodec_find_decoder(AV_CODEC_ID_OPUS);
        }
        if (!codec)
            throw CodecException("PyOpusDecoder: libavcodec has no Opus decoder!");

        context_ = avcodec_alloc_context3(codec);
        if (!context_)
            throw CodecException("PyOpusDecoder: avcodec_alloc_context3 failed!");

        context_->sample_rate = SAMPLE_RATE;
#ifdef RM_HAVE_CH_LAYOUT
        av_channel_layout_uninit(&context_->ch_layout);
        av_channel_layout_default(&context_->ch_layout, CHANNELS);
#else
        context_->channels = CHANNELS;
        context_->channel_layout = channel_layout(CHANNELS);
#endif
        context_->request_sample_fmt = AV_SAMPLE_FMT_S16;
        set_opus_head_extradata();

        if (avcodec_open2(context_, codec, nullptr) < 0)
            throw CodecException("PyOpusDecoder: avcodec_open2 failed!");

        frame_ = av_frame_alloc();
        if (!frame_)
            throw CodecException("PyOpusDecoder: av_frame_alloc failed!");

        pkt_ = av_packet_alloc();
        if (!pkt_)
            throw CodecException("PyOpusDecoder: av_packet_alloc failed!");
    }

    ~PyOpusDecoder() {
        if (swr_)
            swr_free(&swr_);
        if (pkt_)
            av_packet_free(&pkt_);
        if (frame_)
            av_frame_free(&frame_);
        if (context_)
            avcodec_free_context(&context_);
    }

    py::bytes decode(const py::bytes &input) {
        char *data_in = nullptr;
        Py_ssize_t len = 0;
        if (PyBytes_AsStringAndSize(input.ptr(), &data_in, &len) != 0)
            throw py::error_already_set();

        std::string out;
        {
            py::gil_scoped_release decoder_release;
            out = decode_packet(reinterpret_cast<const unsigned char *>(data_in),
                                static_cast<int>(len));
        }
        return py::bytes(out);
    }

private:
#ifndef RM_HAVE_CH_LAYOUT
    static int64_t channel_layout(int channels) {
        int64_t layout = av_get_default_channel_layout(channels);
        return layout ? layout : ((channels == 1) ? AV_CH_LAYOUT_MONO : AV_CH_LAYOUT_STEREO);
    }
#endif

    // libavcodec has no equivalent of opus_decoder_create(rate, channels), it
    // takes the stream layout from an OpusHead header, so synthesize one.
    void set_opus_head_extradata() {
        const int head_size = 19;
        unsigned char *extradata = static_cast<unsigned char *>(
                av_mallocz(head_size + AV_INPUT_BUFFER_PADDING_SIZE));
        if (!extradata)
            throw CodecException("PyOpusDecoder: extradata allocation failed!");

        memcpy(extradata, "OpusHead", 8);
        extradata[8] = 1;                                            // version
        extradata[9] = static_cast<unsigned char>(CHANNELS);         // channel count
        extradata[10] = 0;                                           // pre-skip, LE
        extradata[11] = 0;
        extradata[12] = static_cast<unsigned char>(SAMPLE_RATE & 0xFF);         // input rate, LE
        extradata[13] = static_cast<unsigned char>((SAMPLE_RATE >> 8) & 0xFF);
        extradata[14] = static_cast<unsigned char>((SAMPLE_RATE >> 16) & 0xFF);
        extradata[15] = static_cast<unsigned char>((SAMPLE_RATE >> 24) & 0xFF);
        extradata[16] = 0;                                           // output gain, LE
        extradata[17] = 0;
        extradata[18] = 0;                                           // channel mapping family

        context_->extradata = extradata;
        context_->extradata_size = head_size;
    }

    std::string decode_packet(const unsigned char *data_in, int len) {
        std::string out;
        if (len <= 0)
            return out;

        // libavcodec readers may over-read a packet, so keep the padding.
        packet_buffer_.assign(data_in, data_in + len);
        packet_buffer_.resize(len + AV_INPUT_BUFFER_PADDING_SIZE, 0);

        av_packet_unref(pkt_);
        pkt_->data = packet_buffer_.data();
        pkt_->size = len;
        int sent = avcodec_send_packet(context_, pkt_);
        pkt_->data = nullptr;
        pkt_->size = 0;
        if (sent < 0)
            return out;

        while (avcodec_receive_frame(context_, frame_) >= 0) {
            append_s16(out, *frame_);
            av_frame_unref(frame_);
        }
        return out;
    }

    void append_s16(std::string &out, const AVFrame &frame) {
        if (frame.nb_samples <= 0)
            return;

#ifdef RM_HAVE_CH_LAYOUT
        int in_channels = frame.ch_layout.nb_channels > 0 ? frame.ch_layout.nb_channels : CHANNELS;
#else
        int in_channels = frame.channels > 0 ? frame.channels : CHANNELS;
#endif
        int in_rate = frame.sample_rate > 0 ? frame.sample_rate : SAMPLE_RATE;
        if (static_cast<AVSampleFormat>(frame.format) == AV_SAMPLE_FMT_S16
                && in_channels == CHANNELS && in_rate == SAMPLE_RATE) {
            out.append(reinterpret_cast<const char *>(frame.data[0]),
                       static_cast<size_t>(frame.nb_samples) * CHANNELS * sizeof(int16_t));
            return;
        }

        // The native decoder emits planar float, so convert to interleaved s16.
        if (!swr_) {
#ifdef RM_HAVE_CH_LAYOUT
            // av_channel_layout_copy() uninits its destination first, so these
            // must be zeroed rather than left as stack garbage.
            AVChannelLayout out_layout = {};
            AVChannelLayout in_layout = {};
            av_channel_layout_default(&out_layout, CHANNELS);
            if (frame.ch_layout.nb_channels > 0) {
                av_channel_layout_copy(&in_layout, &frame.ch_layout);
            } else {
                av_channel_layout_default(&in_layout, in_channels);
            }
            int ret = swr_alloc_set_opts2(&swr_,
                                          &out_layout, AV_SAMPLE_FMT_S16, SAMPLE_RATE,
                                          &in_layout, static_cast<AVSampleFormat>(frame.format),
                                          in_rate, 0, nullptr);
            av_channel_layout_uninit(&in_layout);
            av_channel_layout_uninit(&out_layout);
            if (ret < 0 || !swr_ || swr_init(swr_) < 0) {
                if (swr_)
                    swr_free(&swr_);
                throw CodecException("PyOpusDecoder: swr_alloc_set_opts2 failed!");
            }
#else
            int64_t in_layout = frame.channel_layout ? static_cast<int64_t>(frame.channel_layout)
                                                     : channel_layout(in_channels);
            swr_ = swr_alloc_set_opts(nullptr,
                                      channel_layout(CHANNELS), AV_SAMPLE_FMT_S16, SAMPLE_RATE,
                                      in_layout, static_cast<AVSampleFormat>(frame.format), in_rate,
                                      0, nullptr);
            if (!swr_ || swr_init(swr_) < 0) {
                if (swr_)
                    swr_free(&swr_);
                throw CodecException("PyOpusDecoder: swr_init failed!");
            }
#endif
        }

        int out_samples = swr_get_out_samples(swr_, frame.nb_samples);
        if (out_samples <= 0)
            return;
        std::vector<unsigned char> buffer(
                static_cast<size_t>(out_samples) * CHANNELS * sizeof(int16_t));
        unsigned char *out_planes[1] = {buffer.data()};
        int converted = swr_convert(swr_, out_planes, out_samples,
                                    const_cast<const unsigned char **>(frame.data),
                                    frame.nb_samples);
        if (converted > 0)
            out.append(reinterpret_cast<const char *>(buffer.data()),
                       static_cast<size_t>(converted) * CHANNELS * sizeof(int16_t));
    }

    int FRAME_SIZE = 960;
    int SAMPLE_RATE = 48000;
    int CHANNELS = 1;
    AVCodecContext *context_ = nullptr;
    AVFrame *frame_ = nullptr;
    AVPacket *pkt_ = nullptr;
    SwrContext *swr_ = nullptr;
    std::vector<unsigned char> packet_buffer_;
};

#endif