//
// Copyright (c) 2021, University of Edinburgh, Vladimir Ivan
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <exception>
#include <vector>
#include <chrono>
#include <math.h>

#include <libuvc/libuvc.h> // BSD
#include "thetauvc.h" // BSD

extern "C"
{
// LGPL 2.1
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

typedef std::chrono::time_point<std::chrono::system_clock> Time;

inline void TIC(Time& start)
{
    start = std::chrono::system_clock::now();
};

inline double TOC(Time start)
{
    return static_cast<std::chrono::duration<double>>(std::chrono::system_clock::now() - start).count();
};

inline double round(double val, int dec)
{
    double mul = pow(10.0, static_cast<double>(dec));
    return round(val * mul) / mul;
}

class Theta
{
    uvc_context_t *ctx = nullptr;
	uvc_device_t *dev = nullptr;
	uvc_device_handle_t *devh = nullptr;
	uvc_stream_ctrl_t ctrl;
    uvc_error_t res;
    std::shared_ptr<image_transport::ImageTransport> it;
    sensor_msgs::Image img;

    AVFrame* picture = nullptr;
    AVFrame* picture_rgb = nullptr;
    AVCodecContext* codec_context = nullptr;
    AVCodec* codec = nullptr;
    SwsContext * sctx = nullptr;

    ros::NodeHandle nh;
    image_transport::Publisher pub;

    bool has_picture = false;
    int ignore_errors = 1;
    double decode_latency = 0.0;

public:
    Theta() : nh()
    {

        av_log_set_level(AV_LOG_QUIET);

        AVCodec *p;
        p = nullptr;
        while (p)
        {
            if (av_codec_is_decoder(p))
                ROS_INFO_STREAM(p->name);
            p = p->next;
        }

        codec = avcodec_find_decoder(AV_CODEC_ID_H264);
        if(!codec)
        {
            throw std::runtime_error("Cannot find the h264 codec");
        }

        codec_context = avcodec_alloc_context3(codec);

        if(codec->capabilities & AV_CODEC_CAP_TRUNCATED)
        {
            codec_context->flags |= AV_CODEC_FLAG_TRUNCATED;
        }

        if(avcodec_open2(codec_context, codec, NULL) < 0)
        {
            throw std::runtime_error("Could not open codec.");
        }

        picture = av_frame_alloc();
        picture_rgb = av_frame_alloc();

        res = uvc_init(&ctx, NULL);
        if (res != UVC_SUCCESS)
        {
            std::string msg = std::string(uvc_strerror(res));
            throw std::runtime_error(msg);
        }

        int device_id;
        ros::param::param<int>("~device", device_id, 0);
        res = thetauvc_find_device(ctx, &dev, static_cast<unsigned int>(device_id));
        if (res != UVC_SUCCESS)
        {
            throw std::runtime_error("THETA not found");
        }

        res = uvc_open(dev, &devh);
        if (res != UVC_SUCCESS)
        {
            throw std::runtime_error("Can't open THETA");
        }

        bool uhd;
        ros::param::param<bool>("~uhd", uhd, true);
        ROS_INFO_STREAM("Running in " << (uhd ? "4K" : "FullHD") << " mode.");
        res = thetauvc_get_stream_ctrl_format_size(devh, uhd ? THETAUVC_MODE_UHD_2997 : THETAUVC_MODE_FHD_2997, &ctrl);

        it.reset(new image_transport::ImageTransport(nh));
        pub = it->advertise("image_raw", 1);
    }

    ~Theta()
    {
        if (devh != nullptr) uvc_close(devh);
        if (ctx != nullptr) uvc_exit(ctx);
        if(picture != nullptr)
        {
            av_free(picture);
            picture = nullptr;
        }
        if(picture_rgb != nullptr)
        {
            av_free(picture_rgb);
            picture_rgb = nullptr;
        }
        if(codec_context != nullptr)
        {
            avcodec_close(codec_context);
            av_free(codec_context);
            codec_context = nullptr;
        }
    }

    void run()
    {
        res = uvc_start_streaming(devh, &ctrl, Theta::cb_, reinterpret_cast<void*>(this), 0);
        ros::Rate rate(100.0);
        Time t0;
        while(ros::ok())
        {
            if(has_picture)
            {

                if (sctx == nullptr)
                {
                    sctx = sws_getContext(picture->width, picture->height,
                            static_cast<AVPixelFormat>(picture->format), picture->width, picture->height,
                            AV_PIX_FMT_RGB24, 0, 0, 0, 0);
                    img.width = picture->width;
                    img.height = picture->height;
                    img.data.resize(picture->height * picture->width * 3);
                    img.step = picture->width * 3;
                    img.encoding = "rgb8";
                }
                TIC(t0);
                img.header.stamp = ros::Time::now();
                img.header.seq++;
                av_image_fill_arrays (picture_rgb->data, picture_rgb->linesize, img.data.data(), AV_PIX_FMT_RGB24, picture->width, picture->height, 1);
                sws_scale(sctx, picture->data, picture->linesize, 0, picture->height, picture_rgb->data, picture_rgb->linesize);
                has_picture = false;
                pub.publish(img);
                double image_pub_latency = TOC(t0);
                ROS_WARN_STREAM_THROTTLE(1.0, "Latency: " << round((decode_latency + image_pub_latency)*1e3, 2) << "ms (decode: " << round((decode_latency)*1e3, 2) << "ms, ros: " << round((image_pub_latency)*1e3, 2) << "ms)");
            }
            rate.sleep();
        }
    }

    void cb(struct uvc_frame *frame)
    {
        bool got_picture = false;
        int ret = 0;

        AVPacket pkt;
        av_init_packet(&pkt);
        pkt.data = reinterpret_cast<uint8_t*>(frame->data);
        pkt.size = frame->data_bytes;

        Time t0;
        TIC(t0);

        if (pkt.size <= 0) return;

        ret = avcodec_send_packet(codec_context, &pkt);
        if (ret < 0)
        {
            if (ignore_errors > 0)
            {
                ignore_errors--;
                return;
            }
            ROS_ERROR_STREAM_THROTTLE(1.0, "Error sending a packet for decoding " << ret);
            return;
        }

        while (ret >= 0)
        {
            ret = avcodec_receive_frame(codec_context, picture);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            {
                return;
            }
            else if (ret < 0)
            {
                ROS_ERROR_STREAM_THROTTLE(1.0, "Error while decoding a frame.");
                return;
            }
            got_picture = true;
            break;
        }

        decode_latency = TOC(t0);

        if (got_picture)
        {
            has_picture = true;
        }
    }

    static void cb_(struct uvc_frame *frame, void *user_ptr)
    {
        Theta* instance = reinterpret_cast<Theta*>(user_ptr);
        instance->cb(frame);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ricoh_theta_camera");
    avcodec_register_all();
    Theta theta = Theta();
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO_STREAM("Started Theta streaming...");
    theta.run();
    ROS_INFO_STREAM("Stopped theta");
}