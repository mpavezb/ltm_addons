#ifndef LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
#define LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_

#include <ros/ros.h>
#include <ltm/plugin/stream_base.h>
#include <ltm_addons/ImageStream.h>
#include <ltm_addons/ImageStreamSrv.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

#include <warehouse_ros/message_with_metadata.h>
#include <warehouse_ros_mongo/database_connection.h>

typedef warehouse_ros::MessageCollection<ltm_addons::ImageStream> ImageStreamCollection;
typedef boost::shared_ptr<ImageStreamCollection> ImageStreamCollectionPtr;

typedef warehouse_ros::MessageWithMetadata<ltm_addons::ImageStream> ImageStreamWithMetadata;
typedef boost::shared_ptr<const ImageStreamWithMetadata> ImageStreamWithMetadataPtr;

namespace ltm_addons
{
    class ImageStreamPlugin : public ltm::plugin::StreamBase
    {
    private:
        // plugin
        std::string _log_prefix;
        std::string _type;
        std::string _collection_name;
        std::vector<sensor_msgs::ImageConstPtr> _buffer;
        std::vector<uint32_t> registry;
        size_t _last_idx;
        size_t _buffer_size;

        // timing
        int _buffer_max_size;
        ros::Duration _buffer_period;
        ros::Time _last_callback;

        // ROS API
        std::string _image_topic;
        ros::Subscriber _image_sub;
        ros::ServiceServer _status_service;
        ros::ServiceServer _drop_db_service;
        ros::ServiceServer _add_stream_service;
        ros::ServiceServer _get_stream_service;
        ros::ServiceServer _delete_stream_service;

        // database
        DBConnectionPtr _conn;
        ImageStreamCollectionPtr _coll;
        std::string _db_name;


        void image_callback(const sensor_msgs::ImageConstPtr& msg);
        void subscribe();
        void unsubscribe();

        // DB API
        MetadataPtr make_metadata(const ImageStream &stream);
        bool insert(const ImageStream &stream);
        bool get(uint32_t uid, ImageStreamWithMetadataPtr &stream_ptr);
        bool update(uint32_t uid, const ImageStream &stream);


    public:

        ImageStreamPlugin(){}
        ~ImageStreamPlugin();

        void initialize(const std::string& param_ns, DBConnectionPtr ptr, std::string db_name);

        void register_episode(uint32_t uid);

        void unregister_episode(uint32_t uid);

        void collect(uint32_t uid, ltm::What& msg, ros::Time start, ros::Time end);

        void degrade(uint32_t uid);

        std::string get_type();

        std::string get_collection_name();

        // DB API
        bool remove(uint32_t uid);
        int count();
        bool has(int uid);
        bool is_reserved(int uid);
        bool drop_db();
        void setup_db();

        // ROS API
        bool status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
        bool drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
        bool add_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res);
        bool get_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res);
        bool delete_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res);
    };

};
#endif // LTM_ADDONS_IMAGE_STREAM_PLUGIN_H_
