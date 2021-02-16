#include "Record.h"
#include <afxdlgs.h>

#ifndef LOG_H_
#define LOG_H_ (1)

#ifdef NODEBUG
#define LOG(fmt, ...) do {} while (0)
#else
#define LOG(fmt, ...) fprintf(stdout, "[DEBUG] %s:%s:%d: " fmt "\n", __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__)
#endif

#endif

#define USE_AACBSF (0)
#define USE_H264BSF (0)

//@fungofljm Recording �߰� 20201019
const char* test_infile;
FILE* test;

const char* in_filename;
const char* codec_name;
char out_filename[200];
const AVCodec* codec;
AVCodecContext* AVC_context = NULL;
FILE* file;
float in_frame = .0;
int ret;

AVFrame* frame;
AVPacket* pkt;
AVPacket mpkt;
uint8_t endcode[] = { 0, 0, 1, 0xb7 };

AVOutputFormat* ofmt = NULL;
AVFormatContext* ifmt_ctx_v = NULL;
AVFormatContext* ofmt_ctx_v = NULL;

uint8_t* buffer;
uint8_t* YUV_Y;
uint8_t* YUV_U;
uint8_t* YUV_V;
int size;

int yIndex = 0;
int uIndex = 0;
int vIndex = 0;
int rgbIndex = 0;
int maxrate = 10000000;

void Encode(AVCodecContext* AVCodecContext, AVFrame* frame, AVPacket* pkt, FILE* outfile)
{
	int ret;

	/* send the frame to the encoder */
	if (frame)
		printf("Send frame %3lld\n", frame->pts);

	ret = avcodec_send_frame(AVCodecContext, frame);
	while (ret >= 0) {
		ret = avcodec_receive_packet(AVCodecContext, pkt);
		if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			return;
		else if (ret < 0) {
			fprintf(stderr, "Error during encoding\n");
			//exit(1);
		}

		printf("Write packet %3lld (size=%5d)\n", pkt->pts, pkt->size);
		fwrite(pkt->data, 1, pkt->size, outfile);
		av_packet_unref(pkt);
	}
}

bool Record_info(int Width, int Height, int Fps) {
	CFileDialog dlg(FALSE, _T("mp4"), _T("*"), OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT, _T("mp4 FILE (*.mp4)|*.mp4|ALL FILE (*.*)|*.*|"));

	if (dlg.DoModal() == IDOK)
	{
		CString file_path = dlg.GetPathName();

		in_filename = "\savefile.h264";
		sprintf(out_filename, "%s", file_path);

		codec_name = "libx264";

		avcodec_register_all();

		/* find the mpeg1video encoder */
		codec = avcodec_find_encoder_by_name(codec_name);
		if (!codec) {
			fprintf(stderr, "Codec '%s' not found\n", codec_name);
			//exit(1);
		}

		AVC_context = avcodec_alloc_context3(codec);
		if (!AVC_context) {
			fprintf(stderr, "Could not allocate video codec context\n");
			free(AVC_context);
			//exit(1);
		}

		pkt = av_packet_alloc();
		if (!pkt) {
			fprintf(stderr, "Could not allocate video AVPacket\n");
			free(pkt);
			//exit(1);
		}

		/* put sample parameters */
		AVC_context->bit_rate = maxrate;
		/* resolution must be a multiple of two */
		AVC_context->width = Width;
		AVC_context->height = Height;
		/* frames per second */
		AVC_context->time_base = AVRational{ 1, Fps };
		AVC_context->framerate = AVRational{ Fps, 1 };

		/* emit one intra frame every ten frames
		 * check frame pict_type before passing frame
		 * to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
		 * then gop_size is ignored and the output of encoder
		 * will always be I frame irrespective to gop_size
		 */
		AVC_context->gop_size = 1;
		//AVC_context->max_b_frames = 1;
		AVC_context->pix_fmt = AV_PIX_FMT_YUV420P;

		if (codec->id == AV_CODEC_ID_H264)
			av_opt_set(AVC_context->priv_data, "preset", "ultrafast", 0);

		/* open it */
		ret = avcodec_open2(AVC_context, codec, NULL);
		if (ret < 0) {
			fprintf(stderr, "Could not open codec\n");
			//exit(1);
		}

		fopen_s(&file, in_filename, "wb");
		if (!file) {
			fprintf(stderr, "Could not open %s\n", in_filename);
			//exit(1);
		}

		frame = av_frame_alloc();
		if (!frame) {
			fprintf(stderr, "Could not allocate video frame\n");
			free(frame);
			//exit(1);
		}

		frame->format = AVC_context->pix_fmt;
		frame->width = AVC_context->width;
		frame->height = AVC_context->height;

		/* make sure the frame data is writable */
		ret = av_frame_make_writable(frame);
		if (ret < 0) {
			fprintf(stderr, "Could not write to the frame\n");
			//exit(1);
		}

		size = av_image_get_buffer_size((AVPixelFormat)frame->format, frame->width, frame->height, 2);
		if (size < 0) {
			fprintf(stderr, "Could not allocte the image buffer\n");
			//exit(1);
		}

		/* Buffer allocation */
		buffer = (uint8_t*)av_malloc(size);

		ret = av_image_fill_arrays(frame->data, frame->linesize, buffer, (AVPixelFormat)frame->format, frame->width, frame->height, 2);
		if (ret < 0) {
			fprintf(stderr, "Could not fill array in image buffer\n");
			free(buffer);
			//exit(1);
		}

		return true;
	}
	else {
		return false;
	}
}

void Gray2YUV420p_Converter(const unsigned char* Src, int Width, int Height, int Y0, int U, int Y1, int V)
{
	fflush(stdout);

	yIndex = 0;
	uIndex = 0;
	vIndex = 0;

	//Gray -> YUV420p Converter
	for (int x = 0; x < Width * Height; x++) {
		frame->data[0][yIndex++] = 0.895 * Src[x] + 16;
		YUV_U[uIndex++] = 128;
		YUV_V[vIndex++] = 128;
	}

	memcpy(frame->data[1], YUV_U, frame->linesize[1] * frame->height / 2);
	memcpy(frame->data[2], YUV_V, frame->linesize[2] * frame->height / 2);

	frame->pts = in_frame;
	in_frame++;
	/* encode the image */
	Encode(AVC_context, frame, pkt, file);
}

void RGB2YUV420P_Converter(const unsigned char* Src, int Width, int Height)
{
	fflush(stdout);

	rgbIndex = 0;
	//RGB -> YUV
	for (int x = 0; x < Width * Height * 3; x += 3) {
		frame->data[0][rgbIndex] = (0.257 * Src[x + 2]) + (0.504 * Src[x + 1]) + (0.098 * Src[x]) + 16;
		YUV_U[rgbIndex] = -(0.148 * Src[x + 2]) - (0.291 * Src[x + 1]) + (0.439 * Src[x]) + 128;
		YUV_V[rgbIndex] = (0.439 * Src[x + 2]) - (0.368 * Src[x + 1]) - (0.071 * Src[x]) + 128;
		rgbIndex++;
	}

	rgbIndex = 0;
	//YUV -> YUV420P Converter
	for (int y = 0; y < Height; y += 2) {
		for (int x = 0; x < Width; x += 2) {
			frame->data[1][rgbIndex] = (YUV_U[y * Width + x] + YUV_U[(y + 1) * Width + x] + YUV_U[y * Width + (x + 1)] + YUV_U[(y + 1) * Width + (x + 1)]) / 4;
			frame->data[2][rgbIndex] = (YUV_V[y * Width + x] + YUV_V[(y + 1) * Width + x] + YUV_V[y * Width + (x + 1)] + YUV_V[(y + 1) * Width + (x + 1)]) / 4;
			rgbIndex++;
		}
	}

	frame->pts = in_frame;
	in_frame++;
	/* encode the image */
	Encode(AVC_context, frame, pkt, file);
}

//void YUYV2YUV420P_Converter(const unsigned char* Src, int Width, int Height, int Y0=0, int U=1, int Y1=2, int V=3)
void YUYV2YUV420P_Converter(const unsigned char* Src, int Width, int Height, int Y0, int U, int Y1, int V)
{
	fflush(stdout);

	yIndex = 0;
	uIndex = 0;
	vIndex = 0;

	//YUV422 -> YUV
	for (int x = 0; x < Width * Height * 2; x += 4)
	{
		frame->data[0][yIndex++] = Src[x + Y0];
		YUV_U[uIndex++] = Src[x + U];
		frame->data[0][yIndex++] = Src[x + Y1];
		YUV_V[vIndex++] = Src[x + V];
	}

	uIndex = 0;
	vIndex = 0;

	//YUV -> YUV420P Converter
	for (int x = 0; x < Height; x += 2) {
		for (int y = 0; y < Width / 2; y++) {
			frame->data[1][uIndex++] = (YUV_U[x * Width / 2 + y] + YUV_U[(x + 1) * Width / 2 + y]) / 2;
			frame->data[2][vIndex++] = (YUV_V[x * Width / 2 + y] + YUV_V[(x + 1) * Width / 2 + y]) / 2;
		}
	}

	frame->pts = in_frame;
	in_frame++;
	// encode the image
	Encode(AVC_context, frame, pkt, file);
}

int Record_End() {
	
	/* flush the encoder */
	Encode(AVC_context, NULL, pkt, file);
	if (codec->id == AV_CODEC_ID_MPEG1VIDEO || codec->id == AV_CODEC_ID_MPEG2VIDEO)
		fwrite(endcode, 1, sizeof(endcode), file);
	fclose(file);

	fopen_s(&file, in_filename, "rb");
	if (!file) {
		fprintf(stderr, "Could not open %s\n", in_filename);
		//exit(1);
	}

	if ((ret = avformat_open_input(&ifmt_ctx_v, in_filename, 0, 0)) < 0) {
		LOG("Could not open input file.");
		goto end;
	}
	if ((ret = avformat_find_stream_info(ifmt_ctx_v, 0)) < 0) {
		LOG("Failed to retrieve input stream information");
		goto end;
	}

	LOG("Input Information=====================");
	av_dump_format(ifmt_ctx_v, 0, in_filename, 0);
	LOG("======================================");
	// Output
	avformat_alloc_output_context2(&ofmt_ctx_v, NULL, NULL, out_filename);
	if (!ofmt_ctx_v) {
		LOG("Could not create output context");
		ret = AVERROR_UNKNOWN;
		goto end;
	}
	ofmt = ofmt_ctx_v->oformat;
	int videoindex_v = -1, videoindex_out = -1;
	for (int k = 0; k < ifmt_ctx_v->nb_streams; k++) {
		// Create output AVStream according to input AVStream
		if (ifmt_ctx_v->streams[k]->codec->codec_type == AVMEDIA_TYPE_VIDEO) {
			videoindex_v = k;
			AVStream* in_stream = ifmt_ctx_v->streams[k];
			AVStream* out_stream = avformat_new_stream(ofmt_ctx_v, in_stream->codec->codec);
			if (!out_stream) {
				LOG("Failed allocating output stream\n");
				ret = AVERROR_UNKNOWN;
				goto end;
			}
			videoindex_out = out_stream->index;
			// Copy the settings of AVCodecContext
			if (avcodec_copy_context(out_stream->codec, in_stream->codec) < 0) {
				LOG("Failed to copy context from input to output stream codec context\n");
				goto end;
			}
			out_stream->codec->codec_tag = 0;
			if (ofmt_ctx_v->oformat->flags & AVFMT_GLOBALHEADER)
				out_stream->codec->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
			break;
		}
	}

	LOG("Output Information====================");
	av_dump_format(ofmt_ctx_v, 0, out_filename, 1);
	LOG("======================================");

	// Open output file
	if (!(ofmt->flags & AVFMT_NOFILE)) {
		if (avio_open(&ofmt_ctx_v->pb, out_filename, AVIO_FLAG_WRITE) < 0) {
			LOG("Could not open output file '%s'", out_filename);
			goto end;
		}
	}
	// Write file header
	if (avformat_write_header(ofmt_ctx_v, NULL) < 0) {
		LOG("Error occurred when opening output file");
		goto end;
	}
	int frame_index = 0;
	int64_t cur_pts_v = 0, cur_pts_a = 0;

	while (1) {
		AVFormatContext* ifmt_ctx;
		int stream_index = 0;
		AVStream* in_stream, * out_stream;

		ifmt_ctx = ifmt_ctx_v;
		stream_index = videoindex_out;

		if (av_read_frame(ifmt_ctx, &mpkt) >= 0) {
			do {
				if (mpkt.stream_index == videoindex_v) {
					cur_pts_v = mpkt.pts;
					break;
				}
			} while (av_read_frame(ifmt_ctx, &mpkt) >= 0);
		}
		else {
			break;
		}

		in_stream = ifmt_ctx->streams[mpkt.stream_index];
		out_stream = ofmt_ctx_v->streams[stream_index];

		//FIX��No PTS (Example: Raw H.264)
		//Simple Write PTS
		if (mpkt.pts == AV_NOPTS_VALUE) {
			//Write PTS
			AVRational time_base1 = in_stream->time_base;
			//Duration between 2 frames (us)
			int64_t calc_duration = (double)AV_TIME_BASE / av_q2d(in_stream->r_frame_rate);
			//Parameters
			mpkt.pts = (double)(frame_index * calc_duration) / (double)(av_q2d(time_base1) * AV_TIME_BASE);
			mpkt.dts = mpkt.pts;
			mpkt.duration = (double)calc_duration / (double)(av_q2d(time_base1) * AV_TIME_BASE);
			frame_index++;
		}
		/* copy packet */
		// Convert PTS/DTS
		mpkt.pts = av_rescale_q_rnd(mpkt.pts, in_stream->time_base, out_stream->time_base, (enum AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
		mpkt.dts = av_rescale_q_rnd(mpkt.dts, in_stream->time_base, out_stream->time_base, (enum AVRounding)(AV_ROUND_NEAR_INF | AV_ROUND_PASS_MINMAX));
		mpkt.duration = av_rescale_q(mpkt.duration, in_stream->time_base, out_stream->time_base);
		mpkt.pos = -1;
		mpkt.stream_index = stream_index;

		LOG("Write 1 Packet. size:%5d\tpts:%lld", mpkt.size, mpkt.pts);
		// Write
		if (av_interleaved_write_frame(ofmt_ctx_v, &mpkt) < 0) {
			LOG("Error muxing packet");
			break;
		}
		av_free_packet(&mpkt);

	}
	// Write file trailer
	av_write_trailer(ofmt_ctx_v);

end:
	avformat_close_input(&ifmt_ctx_v);
	FlushFileBuffers(out_filename);

	/* close output */
	if (ofmt_ctx_v && !(ofmt->flags & AVFMT_NOFILE))
		avio_close(ofmt_ctx_v->pb);
	avformat_free_context(ofmt_ctx_v);
	if (ret < 0 && ret != AVERROR_EOF) {
		LOG("Error occurred.");
		return -1;
	}

	fclose(file);

	in_frame = 0;

	av_frame_free(&frame);
	av_packet_free(&pkt);
	av_free(buffer);
	avcodec_free_context(&AVC_context);

	remove(in_filename);
	printf("Recording End\n");
	return 0;
}