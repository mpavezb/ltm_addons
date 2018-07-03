#include <ltm_addons/image_stream_plugin.h>
#include <ltm/util/parameter_server_wrapper.h>

namespace ltm_addons
{
    // =================================================================================================================
    // Public API
    // =================================================================================================================

    void ImageStreamPlugin::initialize(const std::string &param_ns, DBConnectionPtr db_ptr, std::string db_name) {
        _log_prefix = "[LTM][Image Stream]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        double buffer_frequency;
        ltm::util::ParameterServerWrapper psw("~");
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
        this->ltm_setup(param_ns, db_ptr, db_name);

        // init ROS interface
        this->ltm_init();
    }

    ImageStreamPlugin::~ImageStreamPlugin() {
        std::vector<sensor_msgs::ImageConstPtr>::iterator buffer_it;
        for (buffer_it = _buffer.begin(); buffer_it != _buffer.end(); ++buffer_it) {
            buffer_it->reset();
        }
    }

    void ImageStreamPlugin::collect(uint32_t uid, ltm::What &msg, ros::Time start, ros::Time end) {
        ROS_DEBUG_STREAM(_log_prefix << "Collecting episode " << uid << ".");

        // save into Image Stream collection
        StreamType stream;
        stream.meta.uid = uid;
        stream.meta.episode = uid;
        stream.meta.start = start;
        stream.meta.end = end;

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

        // append to msg
        ltm::StreamRegister reg;
        reg.uid = uid;
        reg.type = ltm_get_type();
        msg.streams.push_back(reg);

        // unregister
        unregister_episode(uid);

        ltm_insert(stream, make_metadata(stream));
    }

    void ImageStreamPlugin::degrade(uint32_t uid) {
        // TODO
    }

    void ImageStreamPlugin::register_episode(uint32_t uid) {
        bool subscribe = this->ltm_register_episode(uid);
        if (subscribe) this->subscribe();
    }

    void ImageStreamPlugin::unregister_episode(uint32_t uid) {
        bool unsubscribe = this->ltm_unregister_episode(uid);
        if (unsubscribe) this->unsubscribe();
    }

    void ImageStreamPlugin::drop_db() {
        this->reset(this->ltm_get_db_name());
        this->ltm_drop_db();
    }

    void ImageStreamPlugin::reset(const std::string &db_name) {
        _last_callback = ros::Time(0.0);

        // setup (init buffer with zero timed images)
        _last_idx = (size_t)(_buffer_max_size - 1);
        _buffer.reserve((size_t)_buffer_max_size);
        _buffer.resize((size_t)_buffer_max_size);
        _buffer_size = 0;

        this->ltm_resetup_db(db_name);
    }

    void ImageStreamPlugin::append_status(std::stringstream &status) {
        status << this->ltm_get_status();
    }

    MetadataPtr ImageStreamPlugin::make_metadata(const StreamType &stream) {
        MetadataPtr meta = ltm_create_metadata();
        meta->append("uid", (int) stream.meta.uid);
        meta->append("uid", (int) stream.meta.episode);

        double start = stream.meta.start.sec + stream.meta.start.nsec * pow10(-9);
        double end = stream.meta.end.sec + stream.meta.end.nsec * pow10(-9);
        meta->append("start", start);
        meta->append("end", end);

        meta->append("images", (int) stream.images.size());
        return meta;
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

    void ImageStreamPlugin::subscribe() {
        ros::NodeHandle priv("~");
        _image_sub = priv.subscribe(_image_topic, 2, &ImageStreamPlugin::image_callback, this,
                                    ros::TransportHints().unreliable().reliable());
        ROS_DEBUG_STREAM(_log_prefix << "Subscribing to topic: " << _image_sub.getTopic());
    }

    void ImageStreamPlugin::unsubscribe() {
        ROS_DEBUG_STREAM(_log_prefix << "Unsubscribing from topic: " << _image_sub.getTopic());
        _image_sub.shutdown();
    }

};
