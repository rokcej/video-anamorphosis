// Thanks to https://github.com/bartjoyce/video-app

#pragma once

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
#include <stdint.h>
}

struct VideoReaderState {
	// Public things for other parts of the program to read from
	int width, height;
	AVRational time_base;

	// Private internal state
	AVFormatContext* av_format_ctx = NULL;
	AVCodecContext* av_codec_ctx = NULL;
	int video_stream_index;
	AVFrame* av_frame = NULL;
	AVPacket* av_packet = NULL;
	SwsContext* sws_scaler_ctx = NULL;
};

VideoReaderState videoState;
int videoWidth, videoHeight;
uint8_t* videoData = nullptr;

bool initVideo(VideoReaderState* state, const char* filename) {
	// Unpack members of state
	auto& width = state->width;
	auto& height = state->height;
	auto& time_base = state->time_base;
	auto& av_format_ctx = state->av_format_ctx;
	auto& av_codec_ctx = state->av_codec_ctx;
	auto& video_stream_index = state->video_stream_index;
	auto& av_frame = state->av_frame;
	auto& av_packet = state->av_packet;

	// Open the file using libavformat
	av_format_ctx = avformat_alloc_context();
	if (!av_format_ctx) {
		printf("Couldn't create AVFormatContext\n");
		return false;
	}

	if (avformat_open_input(&av_format_ctx, filename, NULL, NULL) != 0) {
		printf("Couldn't open video file\n");
		return false;
	}

	// Find the first valid video stream inside the file
	video_stream_index = -1;
	AVCodecParameters* av_codec_params = nullptr;
	AVCodec* av_codec = nullptr;
	for (unsigned int i = 0; i < av_format_ctx->nb_streams; ++i) {
		av_codec_params = av_format_ctx->streams[i]->codecpar;
		av_codec = avcodec_find_decoder(av_codec_params->codec_id);
		if (!av_codec) {
			continue;
		}
		if (av_codec_params->codec_type == AVMEDIA_TYPE_VIDEO) {
			video_stream_index = i;
			width = av_codec_params->width;
			height = av_codec_params->height;
			time_base = av_format_ctx->streams[i]->time_base;
			break;
		}
	}
	if (video_stream_index == -1) {
		printf("Couldn't find valid video stream inside file\n");
		return false;
	}

	// Set up a codec context for the decoder
	av_codec_ctx = avcodec_alloc_context3(av_codec);
	if (!av_codec_ctx) {
		printf("Couldn't create AVCodecContext\n");
		return false;
	}
	if (avcodec_parameters_to_context(av_codec_ctx, av_codec_params) < 0) {
		printf("Couldn't initialize AVCodecContext\n");
		return false;
	}
	if (avcodec_open2(av_codec_ctx, av_codec, NULL) < 0) {
		printf("Couldn't open codec\n");
		return false;
	}

	av_frame = av_frame_alloc();
	if (!av_frame) {
		printf("Couldn't allocate AVFrame\n");
		return false;
	}
	av_packet = av_packet_alloc();
	if (!av_packet) {
		printf("Couldn't allocate AVPacket\n");
		return false;
	}

	videoHeight = state->height;
	videoWidth = state->width;
	videoData = new uint8_t[videoHeight * videoWidth * 4];

	return true;
}

bool getVideoFrame(VideoReaderState* state, uint8_t* frame_buffer, int64_t* pts) {
	// Unpack members of state
	auto& width = state->width;
	auto& height = state->height;
	auto& av_format_ctx = state->av_format_ctx;
	auto& av_codec_ctx = state->av_codec_ctx;
	auto& video_stream_index = state->video_stream_index;
	auto& av_frame = state->av_frame;
	auto& av_packet = state->av_packet;
	auto& sws_scaler_ctx = state->sws_scaler_ctx;

	// Decode one frame
	int response;
	int ret;
	while ((ret = av_read_frame(av_format_ctx, av_packet)) >= 0) {
		if (av_packet->stream_index != video_stream_index) {
			av_packet_unref(av_packet);
			continue;
		}

		response = avcodec_send_packet(av_codec_ctx, av_packet);
		char buf[256];
		if (response < 0) {
			av_strerror(response, buf, sizeof(buf));
			printf("Failed to decode packet: %s\n", buf);
			return false;
		}

		response = avcodec_receive_frame(av_codec_ctx, av_frame);
		if (response == AVERROR(EAGAIN) || response == AVERROR_EOF) {
			av_packet_unref(av_packet);
			continue;
		}
		else if (response < 0) {
			av_strerror(response, buf, sizeof(buf));
			printf("Failed to decode packet: %s\n", buf);
			return false;
		}

		av_packet_unref(av_packet);
		break;
	}
	if (ret < 0) {
		av_seek_frame(av_format_ctx, video_stream_index, 0, 0);
	}

	*pts = av_frame->pts;

	// Set up sws scaler
	if (!sws_scaler_ctx) {
		sws_scaler_ctx = sws_getContext(width, height, av_codec_ctx->pix_fmt,
			width, height, AV_PIX_FMT_RGB0,
			SWS_BILINEAR, NULL, NULL, NULL);
	}
	if (!sws_scaler_ctx) {
		printf("Couldn't initialize sw scaler\n");
		return false;
	}

	uint8_t* dest[4] = { frame_buffer, NULL, NULL, NULL };
	int dest_linesize[4] = { width * 4, 0, 0, 0 };
	sws_scale(sws_scaler_ctx, av_frame->data, av_frame->linesize, 0, av_frame->height, dest, dest_linesize);

	return true;
}

void cleanupVideo(VideoReaderState* state) {
	sws_freeContext(state->sws_scaler_ctx);
	avformat_close_input(&state->av_format_ctx);
	avformat_free_context(state->av_format_ctx);
	av_frame_free(&state->av_frame);
	av_packet_free(&state->av_packet);
	avcodec_free_context(&state->av_codec_ctx);
	delete[] videoData;
}
