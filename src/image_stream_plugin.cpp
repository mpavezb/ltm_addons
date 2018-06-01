#include <ltm_addons/image_stream_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_addons
{
    void ImageStreamPlugin::initialize(const std::string& param_ns, DBConnectionPtr ptr, std::string db_name) {
        _log_prefix = "[LTM][Image Stream]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        double buffer_frequency;
        std::string type;
        std::string collection_name;
        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", type, "images");
        psw.getParameter(param_ns + "collection", collection_name, "image_streams");
        psw.getParameter(param_ns + "topic", _image_topic, "/robot/fake/sensors/camera/image_raw");
        psw.getParameter(param_ns + "buffer_frequency", buffer_frequency, 3.0);
        psw.getParameter(param_ns + "buffer_size", _buffer_max_size, 100);
        if (_buffer_max_size < 10) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_size' param is too low. Setting to minimum: 10");
            _buffer_max_size = 10;
        }
        if (_buffer_max_size > 1000) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_size' param is too high. Setting to maximum: 1000");
            _buffer_max_size = 1000;
        }
        if (buffer_frequency > 10.0) {
            ROS_WARN_STREAM(_log_prefix << "'buffer_frequency' param is too high. The server might slow down because of this.");
        }

        // timing
        _buffer_period = ros::Duration((1.0 / buffer_frequency));
        _last_callback = ros::Time(0.0);

        // setup (init buffer with zero timed images)
        _last_idx = (size_t)(_buffer_max_size - 1);
        _buffer.reserve((size_t)_buffer_max_size);
        _buffer.resize((size_t)_buffer_max_size);
        _buffer_size = 0;

        // DB connection
        manager.reset(new Manager(ptr, db_name, collection_name, type, _log_prefix));
        manager->setup();
    }

    ImageStreamPlugin::~ImageStreamPlugin() {
        std::vector<sensor_msgs::ImageConstPtr>::iterator buffer_it;
        for (buffer_it = _buffer.begin(); buffer_it != _buffer.end(); ++buffer_it) {
            buffer_it->reset();
        }
    }


    // =================================================================================================================
    // Private API
    // =================================================================================================================

    void ImageStreamPlugin::image_callback(const sensor_msgs::ImageConstPtr& msg) {
        // keep buffer period
        ros::Time now = ros::Time::now();
        if (now - _last_callback < _buffer_period) return;
        _last_callback = now;

        // fill buffer
        _buffer_size = std::min((size_t)_buffer_max_size, _buffer_size + 1);
        _last_idx = (_last_idx + 1) % _buffer_max_size;
        _buffer[_last_idx] = msg;
        ROS_DEBUG_STREAM_THROTTLE(5.0, _log_prefix << "Received image (" << (_last_idx + 1) << "/" << _buffer_max_size << ").");
    }


    // =================================================================================================================
    // Public API
    // =================================================================================================================

    void ImageStreamPlugin::collect(uint32_t uid, ltm::What& msg, ros::Time start, ros::Time end) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: collecting episode " << uid << ".");

        // save into Image Stream collection
        ImageStream stream;
        stream.uid = uid;
        stream.start = start;
        stream.end = end;

        // TODO: mutex (it is not required, because we are using a single-threaded Spinner for callbacks)
        size_t oldest_msg = (_last_idx + 1) % _buffer_max_size;
        size_t curr_msg = oldest_msg;
        int cnt = 0;
        for (int i = 0; i < _buffer_max_size; ++i) {
            sensor_msgs::ImageConstPtr ptr = _buffer[curr_msg];
            curr_msg = (curr_msg + 1) % _buffer_max_size;

            // invalid ptr
            if (!ptr) continue;

            // only msgs in the required interval
            if (ptr->header.stamp < start) continue;
            if (end < ptr->header.stamp) break;

            stream.images.push_back(*ptr);
            cnt++;

            // circular loop
            curr_msg = (curr_msg + 1) % _buffer_max_size;
        }
        ROS_DEBUG_STREAM(_log_prefix << "Collected (" << cnt << ") images.");
        insert(stream);

        // append to msg
        msg.streams.push_back(get_type());

        // unregister
        // TODO: redundant calls to (un)register methods?
        unregister_episode(uid);
    }

    void ImageStreamPlugin::degrade(uint32_t uid) {
        // TODO
    }

    void ImageStreamPlugin::subscribe() {
        ros::NodeHandle priv("~");
        _image_sub = priv.subscribe(_image_topic, 2, &ImageStreamPlugin::image_callback, this,
                                    ros::TransportHints().unreliable().reliable());
        ROS_WARN_STREAM(_log_prefix << "Subscribing to topic: " << _image_sub.getTopic());
    }

    void ImageStreamPlugin::unsubscribe() {
        ROS_WARN_STREAM(_log_prefix << "Unsubscribing from topic: " << _image_sub.getTopic());
        _image_sub.shutdown();
    }

    void ImageStreamPlugin::setup_db() {

    }

    std::string ImageStreamPlugin::get_type() {
        return manager->get_type();
    }

    std::string ImageStreamPlugin::get_collection_name() {
        return manager->get_collection_name();
    }

    void ImageStreamPlugin::register_episode(uint32_t uid) {
        manager->register_episode(uid);
    }

    void ImageStreamPlugin::unregister_episode(uint32_t uid) {
        manager->unregister_episode(uid);
    }

    bool ImageStreamPlugin::is_reserved(int uid) {
        return manager->is_reserved(uid);
    }


    // -----------------------------------------------------------------------------------------------------------------
    // CRUD methods
    // -----------------------------------------------------------------------------------------------------------------

    MetadataPtr ImageStreamPlugin::make_metadata(const ltm_addons::ImageStream &stream) {
        MetadataPtr meta = manager->create_metadata();
        meta->append("uid", (int) stream.uid);

        double start = stream.start.sec + stream.start.nsec * pow10(-9);
        double end = stream.end.sec + stream.end.nsec * pow10(-9);
        meta->append("start", start);
        meta->append("end", end);
        return meta;
    }

    bool ImageStreamPlugin::insert(const ImageStream &stream) {
        return manager->insert(stream, make_metadata(stream));
    }

    bool ImageStreamPlugin::get(uint32_t uid, Manager::StreamWithMetadataPtr &stream_ptr) {
        return manager->get(uid, stream_ptr);
    }

    bool ImageStreamPlugin::update(uint32_t uid, const ltm_addons::ImageStream &stream) {
        return manager->update(uid, stream);
    }

    bool ImageStreamPlugin::remove(uint32_t uid) {
        return manager->remove(uid);
    }

    int ImageStreamPlugin::count() {
        return manager->count();
    }

    bool ImageStreamPlugin::has(int uid) {
        return manager->has(uid);
    }

    bool ImageStreamPlugin::drop_db() {
        return manager->drop_db();
    }

};
