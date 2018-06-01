#include <ltm_addons/image_stream_plugin.h>
#include <ltm/parameter_server_wrapper.h>

namespace ltm_addons
{
    void ImageStreamPlugin::initialize(const std::string& param_ns, DBConnectionPtr ptr, std::string db_name) {
        _log_prefix = "[LTM][Image Stream]: ";
        ROS_DEBUG_STREAM(_log_prefix << "plugin initialized with ns: " << param_ns);

        // ROS Parameter Server
        double buffer_frequency;
        ltm::ParameterServerWrapper psw("~");
        psw.getParameter(param_ns + "type", _type, "images");
        psw.getParameter(param_ns + "collection", _collection_name, "image_streams");
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
        _conn = ptr;
        _db_name = db_name;
        setup_db();

        // ROS API
        ros::NodeHandle priv("~");
        _status_service = priv.advertiseService("stream/" + _type + "/status", &ImageStreamPlugin::status_service, this);
        _drop_db_service = priv.advertiseService("stream/" + _type + "/drop_db", &ImageStreamPlugin::drop_db_service, this);
        _add_stream_service = priv.advertiseService("stream/" + _type + "/add", &ImageStreamPlugin::add_service, this);
        _get_stream_service = priv.advertiseService("stream/" + _type + "/get", &ImageStreamPlugin::get_service, this);
        _delete_stream_service = priv.advertiseService("stream/" + _type + "/delete", &ImageStreamPlugin::delete_service, this);
    }

    ImageStreamPlugin::~ImageStreamPlugin() {
        std::vector<sensor_msgs::ImageConstPtr>::iterator buffer_it;
        for (buffer_it = _buffer.begin(); buffer_it != _buffer.end(); ++buffer_it) {
            buffer_it->reset();
        }
    }

    void ImageStreamPlugin::setup_db() {
        try {
            // host, port, timeout
            _coll = _conn->openCollectionPtr<ImageStream>(_db_name, _collection_name);
        }
        catch (const warehouse_ros::DbConnectException& exception) {
            // Connection timeout
            ROS_ERROR_STREAM("Connection timeout to DB '" << _db_name << "' while trying to open collection " << _collection_name);
        }
        // Check for empty database
        if (!_conn->isConnected() || !_coll) {
            ROS_ERROR_STREAM("Connection to DB failed for collection '" << _collection_name << "'.");
        }
    }


    // =================================================================================================================
    // Private API
    // =================================================================================================================

    void ImageStreamPlugin::subscribe() {
        ros::NodeHandle priv("~");
        _image_sub = priv.subscribe(_image_topic, 2, &ImageStreamPlugin::image_callback, this,
                                    ros::TransportHints().unreliable().reliable());
        ROS_INFO_STREAM(_log_prefix << "Subscribing to topic: " << _image_sub.getTopic());
    }

    void ImageStreamPlugin::unsubscribe() {
        ROS_INFO_STREAM(_log_prefix << "Unsubscribing from topic: " << _image_sub.getTopic());
        _image_sub.shutdown();
    }

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

    std::string ImageStreamPlugin::get_type() {
        return _type;
    }

    std::string ImageStreamPlugin::get_collection_name() {
        return _collection_name;
    }

    void ImageStreamPlugin::register_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: registering episode " << uid);

        // subscribe on demand
        if (registry.empty()) subscribe();

        // register in cache
        std::vector<uint32_t>::const_iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it == registry.end()) registry.push_back(uid);
    }

    void ImageStreamPlugin::unregister_episode(uint32_t uid) {
        ROS_DEBUG_STREAM(_log_prefix << "LTM Image Stream plugin: unregistering episode " << uid);
        // unregister from cache
        std::vector<uint32_t>::iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it != registry.end()) registry.erase(it);

        // unsubscribe on demand
        if (registry.empty()) unsubscribe();
    }

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


    // -----------------------------------------------------------------------------------------------------------------
    // CRUD methods
    // -----------------------------------------------------------------------------------------------------------------

    MetadataPtr ImageStreamPlugin::make_metadata(const ltm_addons::ImageStream &stream) {
        MetadataPtr meta = _coll->createMetadata();
        meta->append("uid", (int) stream.uid);

        double start = stream.start.sec + stream.start.nsec * pow10(-9);
        double end = stream.end.sec + stream.end.nsec * pow10(-9);
        meta->append("start", start);
        meta->append("end", end);
        return meta;
    }

    bool ImageStreamPlugin::insert(const ImageStream &stream) {
        _coll->insert(stream, make_metadata(stream));
        // todo: insert into cache
        ROS_INFO_STREAM(_log_prefix << "Inserting stream (" << stream.uid << ") into collection " << "'" << _collection_name << "'. (" << count() << ") entries.");
        return true;
    }

    bool ImageStreamPlugin::get(uint32_t uid, ImageStreamWithMetadataPtr &stream_ptr) {
        QueryPtr query = _coll->createQuery();
        query->append("uid", (int)uid);
        try {
            stream_ptr = _coll->findOne(query, false);
        }
        catch (const warehouse_ros::NoMatchingMessageException& exception) {
            stream_ptr.reset();
            return false;
        }
        return true;
    }

    bool ImageStreamPlugin::update(uint32_t uid, const ltm_addons::ImageStream &stream) {
        ROS_WARN("UPDATE: Method not implemented");
        return false;
    }

    bool ImageStreamPlugin::remove(uint32_t uid) {

        // value is already reserved
        std::vector<uint32_t>::iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it != registry.end()) registry.erase(it);

        // value is in db cache
        // remove from cache

        // remove from DB
        QueryPtr query = _coll->createQuery();
        query->append("uid", (int)uid);
        _coll->removeMessages(query);
        return true;
    }

    int ImageStreamPlugin::count() {
        return _coll->count();
    }

    bool ImageStreamPlugin::has(int uid) {
        // TODO: doc, no revisa por uids ya registradas
        QueryPtr query = _coll->createQuery();
        query->append("uid", uid);
        try {
            _coll->findOne(query, true);
        }
        catch (const warehouse_ros::NoMatchingMessageException& exception) {
            return false;
        }
        return true;
    }

    bool ImageStreamPlugin::is_reserved(int uid) {

        // value is in registry
        std::vector<uint32_t>::iterator it = std::find(registry.begin(), registry.end(), uid);
        if (it != registry.end()) return true;

        // TODO: value is in db cache

        // value is already in DB
        return has(uid);
    }

    bool ImageStreamPlugin::drop_db() {
        _conn->dropDatabase(_db_name);
        registry.clear();
        // TODO: clear cache
        setup_db();
        return true;
    }


    // -----------------------------------------------------------------------------------------------------------------
    // ROS API
    // -----------------------------------------------------------------------------------------------------------------

    bool ImageStreamPlugin::status_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
        ROS_INFO_STREAM(_log_prefix << "Collection '" << _collection_name << "' has " << count() << " entries.");
        return true;
    }

    bool ImageStreamPlugin::drop_db_service(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
        ROS_WARN_STREAM(_log_prefix << "Deleting all entries from collection '" << _collection_name << "'.");
        std_srvs::Empty srv;
        drop_db();
        status_service(srv.request, srv.response);
        return true;
    }

    bool ImageStreamPlugin::add_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res) {
        ROS_WARN_STREAM(_log_prefix << "ADD service is not implemented yet.");
        return true;
    }

    bool ImageStreamPlugin::get_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res) {
        ROS_INFO_STREAM(_log_prefix << "Retrieving stream (" << req.uid << ") from collection '" << _collection_name << "'");
        ImageStreamWithMetadataPtr stream_ptr;
        if (!get(req.uid, stream_ptr)) {
            ROS_ERROR_STREAM(_log_prefix << "Stream with uid '" << req.uid << "' not found.");
            res.succeeded = (uint8_t) false;
            return true;
        }
        res.msg = *stream_ptr;
        res.succeeded = (uint8_t) true;
        return true;
    }

    bool ImageStreamPlugin::delete_service(ltm_addons::ImageStreamSrv::Request  &req, ltm_addons::ImageStreamSrv::Response &res) {
        ROS_INFO_STREAM(_log_prefix << "Removing (" << req.uid << ") from collection '" << _collection_name << "'");
        return remove(req.uid);
    }

};
