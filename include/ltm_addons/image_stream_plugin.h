#ifndef LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
#define LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/stream_base.h>
#include <ltm/plugin/stream_default.h>
#include <ltm_addons/ImageStream.h>
#include <ltm_addons/ImageStreamSrv.h>
#include <sensor_msgs/Image.h>

namespace ltm_addons
{
    class ImageStreamPlugin :
            public ltm::plugin::StreamBase,
            public ltm::plugin::StreamDefault<ltm_addons::ImageStream, ltm_addons::ImageStreamSrv>
    {
    private:
        typedef ltm_addons::ImageStream StreamType;

        // plugin
        std::vector<sensor_msgs::ImageConstPtr> _buffer;
        size_t _last_idx;
        size_t _buffer_size;

        // timing
        int _buffer_max_size;
        ros::Duration _buffer_period;
        ros::Time _last_callback;

        // ROS API
        std::string _log_prefix;
        std::string _image_topic;
        ros::Subscriber _image_sub;

    public:
        ImageStreamPlugin(){}
        ~ImageStreamPlugin();

    private:
        // ROS topic callback
        void subscribe();
        void unsubscribe();
        void image_callback(const sensor_msgs::ImageConstPtr& msg);

    public:
        void initialize(const std::string &param_ns, DBConnectionPtr ptr, std::string db_name);
        void collect(uint32_t uid, ltm::What &msg, ros::Time _start, ros::Time _end);
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        void degrade(uint32_t uid);
        void drop_db();
        void reset(const std::string &db_name);
        void append_status(std::stringstream &status);
        MetadataPtr make_metadata(const StreamType &stream);
    };

};
#endif // LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
