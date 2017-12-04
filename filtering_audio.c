/*
 * Copyright (c) 2010 Nicolas George
 * Copyright (c) 2011 Stefano Sabatini
 * Copyright (c) 2012 Clément Bœsch
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/**
 * @file
 * API example for audio decoding and filtering
 * @example filtering_audio.c
 */
extern "C" {
#define __STDC_FORMAT_MACROS
#include <unistd.h>
#include <libavutil/channel_layout.h>
#include "libavutil/md5.h"
#include "libavutil/mem.h"
#include "libavutil/samplefmt.h"
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavfilter/avfiltergraph.h>
#include <libavfilter/buffersink.h>
#include <libavfilter/buffersrc.h>
#include <libavutil/opt.h>
#include <inttypes.h> 
}

#include <string>
#include <iostream>
#include "Nls.h"
#include "openssl/crypto.h"
#include "openssl/err.h"
#include "openssl/ssl.h"
#include <vector>
#include <fstream>
#include <sstream>

#include "jason.h"
#include <stdio.h>

using namespace std;
static const char *status_code = "\"status_code\":0";
static const char *filter_descr = "aresample=8000";
static const char *player       = "ffplay -f s16le -ar 8000 -ac 1 -";

static AVFormatContext *fmt_ctx;
static AVCodecContext *dec_ctx;
AVFilterContext *buffersink_ctx;
AVFilterContext *buffersrc_ctx;
AVFilterGraph *filter_graph;
static int audio_stream_index = -1;

static int open_input_file(const char *filename)
{
    int ret;
    AVCodec *dec;

    if ((ret = avformat_open_input(&fmt_ctx, filename, NULL, NULL)) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open input file\n");
        return ret;
    }

    if ((ret = avformat_find_stream_info(fmt_ctx, NULL)) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot find stream information\n");
        return ret;
    }

    /* select the audio stream */
    ret = av_find_best_stream(fmt_ctx, AVMEDIA_TYPE_AUDIO, -1, -1, &dec, 0);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot find an audio stream in the input file\n");
        return ret;
    }
    audio_stream_index = ret;

    /* create decoding context */
    dec_ctx = avcodec_alloc_context3(dec);
    if (!dec_ctx)
        return AVERROR(ENOMEM);
    avcodec_parameters_to_context(dec_ctx, fmt_ctx->streams[audio_stream_index]->codecpar);
    av_opt_set_int(dec_ctx, "refcounted_frames", 1, 0);

    /* init the audio decoder */
    if ((ret = avcodec_open2(dec_ctx, dec, NULL)) < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot open audio decoder\n");
        return ret;
    }

    return 0;
}

static int init_filters(const char *filters_descr)
{
    char args[512];
    int ret = 0;
    AVFilter *abuffersrc  = avfilter_get_by_name("abuffer");
    AVFilter *abuffersink = avfilter_get_by_name("abuffersink");
    AVFilterInOut *outputs = avfilter_inout_alloc();
    AVFilterInOut *inputs  = avfilter_inout_alloc();
    static const enum AVSampleFormat out_sample_fmts[] = { AV_SAMPLE_FMT_S16, AV_SAMPLE_FMT_NONE };
    static const int64_t out_channel_layouts[] = { AV_CH_LAYOUT_MONO, -1 };
    static const int out_sample_rates[] = { 16000, -1 };
    const AVFilterLink *outlink;
    AVRational time_base = fmt_ctx->streams[audio_stream_index]->time_base;

    filter_graph = avfilter_graph_alloc();
    if (!outputs || !inputs || !filter_graph) {
        ret = AVERROR(ENOMEM);
        goto end;
    }

    /* buffer audio source: the decoded frames from the decoder will be inserted here. */
    if (!dec_ctx->channel_layout)
        dec_ctx->channel_layout = av_get_default_channel_layout(dec_ctx->channels);
    snprintf(args, sizeof(args),
            "time_base=%d/%d:sample_rate=%d:sample_fmt=%s:channel_layout=0x%" PRIx64,
             time_base.num, time_base.den, dec_ctx->sample_rate,
             av_get_sample_fmt_name(dec_ctx->sample_fmt),
	     //(uint64_t)AV_CH_LAYOUT_STEREO);
	     dec_ctx->channel_layout);
    ret = avfilter_graph_create_filter(&buffersrc_ctx, abuffersrc, "in",
                                       args, NULL, filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create audio buffer source  %s \n", args);
        goto end;
    }

    /* buffer audio sink: to terminate the filter chain. */
    ret = avfilter_graph_create_filter(&buffersink_ctx, abuffersink, "out",
                                       NULL, NULL, filter_graph);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot create audio buffer sink\n");
        goto end;
    }

    ret = av_opt_set_int_list(buffersink_ctx, "sample_fmts", out_sample_fmts, -1,
                              AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot set output sample format\n");
        goto end;
    }

    ret = av_opt_set_int_list(buffersink_ctx, "channel_layouts", out_channel_layouts, -1,
                              AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot set output channel layout\n");
        goto end;
    }

    ret = av_opt_set_int_list(buffersink_ctx, "sample_rates", out_sample_rates, -1,
                              AV_OPT_SEARCH_CHILDREN);
    if (ret < 0) {
        av_log(NULL, AV_LOG_ERROR, "Cannot set output sample rate\n");
        goto end;
    }

    /*
     * Set the endpoints for the filter graph. The filter_graph will
     * be linked to the graph described by filters_descr.
     */

    /*
     * The buffer source output must be connected to the input pad of
     * the first filter described by filters_descr; since the first
     * filter input label is not specified, it is set to "in" by
     * default.
     */
    outputs->name       = av_strdup("in");
    outputs->filter_ctx = buffersrc_ctx;
    outputs->pad_idx    = 0;
    outputs->next       = NULL;

    /*
     * The buffer sink input must be connected to the output pad of
     * the last filter described by filters_descr; since the last
     * filter output label is not specified, it is set to "out" by
     * default.
     */
    inputs->name       = av_strdup("out");
    inputs->filter_ctx = buffersink_ctx;
    inputs->pad_idx    = 0;
    inputs->next       = NULL;

    if ((ret = avfilter_graph_parse_ptr(filter_graph, filters_descr,
                                        &inputs, &outputs, NULL)) < 0)
        goto end;

    if ((ret = avfilter_graph_config(filter_graph, NULL)) < 0)
        goto end;

    /* Print summary of the sink buffer
     * Note: args buffer is reused to store channel layout string */
    outlink = buffersink_ctx->inputs[0];
    av_get_channel_layout_string(args, sizeof(args), -1, outlink->channel_layout);
    av_log(NULL, AV_LOG_INFO, "Output: srate:%dHz fmt:%s chlayout:%s\n",
           (int)outlink->sample_rate,
           "",
           //(char *)av_x_if_null(av_get_sample_fmt_name(outlink->format), "?"),
           args);

end:
    avfilter_inout_free(&inputs);
    avfilter_inout_free(&outputs);

    return ret;
}
static void recognoize(AVCodecContext * dec_ctx,const AVFrame * frame, Nls * nls){//, FILE*outfile){
    int i, ch, data_size;
    char buf[1024];
    const int n = frame->nb_samples * av_get_channel_layout_nb_channels(frame->channel_layout);
    //fprintf(stdout, "n = %d, samples = %d, channel = %d\n", n, frame->nb_samples, av_get_channel_layout_nb_channels(frame->channel_layout));
    const char *p     = (char*)frame->data[0];
    nls->sendAsr(p, n*2);
    i = 0;
    while(i<n*2){
        buf[i] = p[i] & 0xff;
        i++;
    }
    //fwrite(buf, 1, n*2, outfile);
    nanosleep((const struct timespec[]){{0,  20000000L}}, NULL);
    
    data_size = av_get_bytes_per_sample(dec_ctx->sample_fmt);
    if (data_size<0){
        fprintf(stderr, "Error decoding");
        exit(1);
    }
    //av_log(NULL, AV_LOG_INFO, "samples: %d  channel: %d data:%d, linesize:%d layout:%d\n", frame->nb_samples,dec_ctx->channels ,n,frame->linesize, frame->channel_layout);
 
    return ;
}

static void print_frame(const AVFrame *frame, FILE*outfile)
{
    int i=0;
    const int n = frame->nb_samples * av_get_channel_layout_nb_channels(frame->channel_layout);
    const uint16_t *p     = (uint16_t*)frame->data[0];
    const uint16_t *p_end = p + n;
    
    //fprintf(stdout, "%lld %lld % lld\n",frame->pts, frame->pkt_pos,frame->pkt_dts);

    while (p < p_end) {
        //fprintf(stdout, "%x%x ", *p & 0xff, *p>>8 & 0xff);
        fputc(*p    & 0xff, outfile);
        fputc(*p>>8 & 0xff, outfile);
        p++;
        i++;
    }
    av_log(NULL, AV_LOG_INFO, "data: %d \n", i);
    
    //fflush(stdout);
}



int test_jason()
{
    const char *json = "{\"Users\":[{\"Name\":\"John\",\"Age\":25,\"Salary\":20000},{\"Name\":\"Mary\",\"Age\":42,\"Salary\":45000}]}";
    
    jason jason;
    memset(&jason, 0, sizeof(jason));
    
    jasonStatus status = jason_Deserialize(&jason, json, (int32_t)strlen(json));
    
    if(status != jasonStatus_Finished)
    {
        printf("%s: \"%.50s...\"\n", jasonStatus_Describe(status), jason.ParsePosition);
    }
    else
    {
        jasonValue *usersList = jason_HashLookup(&jason, jason.RootValue, "Users", strlen("Users"));
        if(usersList != NULL)
        {
            for(jasonValue *user = jasonValue_GetFirstChild(usersList); user != NULL; user = jasonValue_GetNextSibling(user))
            {
                jasonValue *name = jason_HashLookup(&jason, user, "Name", strlen("Name"));
                jasonValue *age = jason_HashLookup(&jason, user, "Age", strlen("Age"));
                jasonValue *salary = jason_HashLookup(&jason, user, "Salary", strlen("Salary"));
                
                if(name != NULL)
                {
                    printf("Name: %.*s\n", jasonValue_GetValueLen(name), jasonValue_GetValue(name));
                }
                
                if(age != NULL)
                {
                    printf("Age: %i\n", atoi(jasonValue_GetValue(age)));
                }
                
                if(salary != NULL)
                {
                    printf("Salary: %f\n", atof(jasonValue_GetValue(salary)));
                }
            }
        }
    }
    
    jason_Cleanup(&jason);
    return 0;
}



char* prase_jason(char* json)
{
   	char begintime[16], endtime[16], respond[2048];
   	char *pstr;
   	//jasonValue *begin_time, end_time,sentence_id, status_code, text;
    jason jason;
    memset(&jason, 0, sizeof(jason));
    
    jasonStatus status = jason_Deserialize(&jason, json, (int32_t)strlen(json));
    
    if(status != jasonStatus_Finished)
    {
        printf("%s: \"%.50s...\"\n", jasonStatus_Describe(status), jason.ParsePosition);
    }else{
		    jasonValue *begin_time = jason_HashLookup(&jason, jason.RootValue, "begin_time", strlen("begin_time"));
		    jasonValue *end_time = jason_HashLookup(&jason, jason.RootValue, "end_time", strlen("end_time"));
		    jasonValue *sentence_id = jason_HashLookup(&jason, jason.RootValue, "sentence_id", strlen("sentence_id"));
		    jasonValue *status_code = jason_HashLookup(&jason, jason.RootValue, "status_code", strlen("status_code"));
		    jasonValue *text = jason_HashLookup(&jason, jason.RootValue, "text", strlen("text"));
		    
		    if(begin_time != NULL)
		    {
		    		int btime = atoi(jasonValue_GetValue(begin_time))/2000;
						sprintf(begintime, "%02d:%02d:%02d", btime/3600, (btime%3600)/60, btime%60);
		        printf("begintime: %s\n", begintime);
		    }
		    
		    if(end_time != NULL)
		    {
		    		int etime = atoi(jasonValue_GetValue(end_time))/2000;
						sprintf(endtime, "%02d:%02d:%02d", etime/3600, (etime%3600)/60, etime%60);
		        printf("endtime: %s\n", endtime);
		    }
		    
		    if(sentence_id != NULL)
		    {
		        printf("sentence_id: %d\n", atoi(jasonValue_GetValue(sentence_id)));
		    }
		    if(status_code != NULL)
		    {
		        printf("status_code: %d\n", atoi(jasonValue_GetValue(status_code)));
		    }
		    if(text != NULL)
		    {
		        //printf("text: %s\n", jasonValue_GetValue(text));
		        pstr = strtok(jasonValue_GetValue(text), "\"");
		    }
		    sprintf(respond, "%d\n%s,0 --> %s,0\n%s\n\n", atoi(jasonValue_GetValue(sentence_id)), begintime, endtime, pstr);
  	}
  	
    jason_Cleanup(&jason);
    return respond;
}


static char* parse_result(char* str)
{
	int i;
	char *delim = "{";
	char *pstr = NULL;
	char result[2048] ;

	pstr = strtok(str, delim);
	if ((pstr = strtok(NULL, delim)) != NULL)
	{
	}
	sprintf(result, "{%s", pstr);
	return result;
}

int main(int argc, char *argv[])
{
		static FILE*fp=NULL;
		char result[2048];
		char file_line[2048];
    int port = 443;
    string ip = "nls-trans.dataapi.aliyun.com";
    string id = "fQ0sCAzPbJJZgYMo";                  //  id
    string scret = "41tzjO1Ce24qZPHxQMUNY6tUSx3hv6"; // scret
    string url = "wss://" + ip + ":" + to_string(port) + "/realtime";
    Nls nls(url, 10);
    nls.setSampleRate(8000);
    nls.setResponseMode(0);
    nls.setApp_key("nls-service-multi-domain"); //key
    nls.authorize(id, scret);
    
    nls._onResultReceivedEvent = ([&](string str) {
    		const char *p = str.c_str();
        if(strstr(p, status_code) != NULL){
        	cout << ">>>>>" << str << endl;
        	strcpy(result,parse_result(p));
        	strcpy(file_line,prase_jason(result)); 
        	
          if(NULL==fp)
              fp=fopen("1.srt","wb");
          if(fp)
          {
              fwrite(file_line,1,strlen(file_line),fp);
              fflush(fp);
          }
        }
    });
    if (nls.startNls() == false)
    {
        return 0;
    }
    
    int ret;
    AVPacket packet;
    AVFrame *frame = av_frame_alloc();
    AVFrame *filt_frame = av_frame_alloc();

    if (!frame || !filt_frame) {
        perror("Could not allocate frame");
        exit(1);
    }
 
    av_register_all();
    avformat_network_init();
    avfilter_register_all();
		
    char file_url[256];
    if(argc<2){
    		strcpy(file_url, "http://v.videoincloud.com/gd/20170425/NeVMXK/NeVMXK.m3u8");
    }else{
    		strcpy(file_url, argv[1]);
  	}
    if ((ret = open_input_file(file_url)) < 0)
        goto end;
    if ((ret = init_filters(filter_descr)) < 0)
        goto end;

    /* read all packets */
    while (1) {
        if ((ret = av_read_frame(fmt_ctx, &packet)) < 0)
            break;
        if (packet.stream_index == audio_stream_index) {
            ret = avcodec_send_packet(dec_ctx, &packet);
            if (ret < 0) {
                av_log(NULL, AV_LOG_ERROR, "Error while sending a packet to the decoder\n");
                break;
            }
            while (ret >= 0) {
                ret = avcodec_receive_frame(dec_ctx, frame);
                if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                    break;
                } else if (ret < 0) {
                    av_log(NULL, AV_LOG_ERROR, "Error while receiving a frame from the decoder\n");
                    goto end;
                }
                if (ret >= 0) {
                    /* push the audio data from decoded frame into the filtergraph */
                    if (av_buffersrc_add_frame_flags(buffersrc_ctx, frame, AV_BUFFERSRC_FLAG_KEEP_REF) < 0) {
                        av_log(NULL, AV_LOG_ERROR, "Error while feeding the audio filtergraph\n");
                        break;
                    }
                    /* pull filtered audio from the filtergraph */
                    while (1) {
                        ret = av_buffersink_get_frame(buffersink_ctx, filt_frame);
                        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                            break;
                        if (ret < 0)
                            goto end;
												recognoize(dec_ctx, filt_frame, &nls);
                        //recognoize(dec_ctx, filt_frame, &nls, outfile);
                        av_frame_unref(filt_frame);
                        //goto end;
                    }
                    av_frame_unref(frame);
                }
            }
        }
        av_packet_unref(&packet);
    }
end:
		fclose(fp);
    avfilter_graph_free(&filter_graph);
    avcodec_free_context(&dec_ctx);
    avformat_close_input(&fmt_ctx);
    av_frame_free(&frame);
    av_frame_free(&filt_frame);
	
    if (ret < 0 && ret != AVERROR_EOF) {
        fprintf(stderr, "Error occurred: %s\n", (ret));
        exit(1);
    }
		
    exit(0);
}
