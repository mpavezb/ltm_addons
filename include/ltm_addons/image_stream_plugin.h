#ifndef LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
#define LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/stream_base.h>
#include <ltm_addons/ImageStream.h>
#include <ltm_addons/ImageStreamSrv.h>
#include <sensor_msgs/Image.h>

namespace ltm_addons
{
    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    private:
        typedef ltm::db::StreamCollectionManager<ltm_addons::ImageStream, ltm_addons::ImageStreamSrv> Manager;
        typedef boost::shared_ptr<Manager> ManagerPtr;
        ManagerPtr manager;

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

        void image_callback(const sensor_msgs::ImageConstPtr& msg);

        // DB API
        MetadataPtr make_metadata(const ImageStream &stream);
        bool insert(const ImageStream &stream);
        bool get(uint32_t uid, Manager::StreamWithMetadataPtr &stream_ptr);
        bool update(uint32_t uid, const ImageStream &stream);

    protected:
        void subscribe();
        void unsubscribe();

    public:

        ImageStreamPlugin(){}
        ~ImageStreamPlugin();

        void initialize(const std::string& param_ns, DBConnectionPtr ptr, std::string db_name);

        void collect(uint32_t uid, ltm::What& msg, ros::Time start, ros::Time end);

        void degrade(uint32_t uid);

        void setup_db();
        std::string get_type();
        std::string get_collection_name();
        void register_episode(uint32_t uid);
        void unregister_episode(uint32_t uid);
        bool is_reserved(int uid);

        // DB API
        bool remove(uint32_t uid);
        int count();
        bool has(int uid);
        bool drop_db();

    };

};
#endif // LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
