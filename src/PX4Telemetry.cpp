#include "PX4Telemetry.hpp"

#define MIN_VOLTAGE 19.2
#define MAX_VOLTAGE 25.2

using std::placeholders::_1;
using namespace std::chrono_literals;

int trail_id_ = 0;

PX4Telemetry::PX4Telemetry() : Node("px4_telemetry_node"), landing_requested_(false), alt_init_(false), lpos_init_(false), gpos_init_(false) {
    
    //Get namespace (remove the slash with substr)
    px4_id_ = std::string(this->get_namespace()).substr(1);

    init_parameters();

    init_publishers();

    init_subscribers();

    init_service_clients();

    send_connection_request();


	trail_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.01), std::bind(&PX4Telemetry::publish_trail, this));  
    drone_state_timer_ = this->create_wall_timer(std::chrono::duration<double>(0.02), std::bind(&PX4Telemetry::publish_drone_state, this));

    if (sim_mode_)
        RCLCPP_INFO(this->get_logger(), "PX4 Telemetry Initialized In Sim Mode.");
    else 
        RCLCPP_INFO(this->get_logger(), "PX4 Telemetry Initialized In APark Mode.");
}

void PX4Telemetry::publish_trail() {
	pose_trail_.header.frame_id = "autonomy_park";
	pose_trail_.ns = "trail";
	pose_trail_.id = trail_id_;
	pose_trail_.header.stamp = this->get_clock()->now();
	pose_trail_.action = visualization_msgs::msg::Marker::ADD;
	pose_trail_.type = visualization_msgs::msg::Marker::CUBE;
	pose_trail_.scale.x = 0.05;
	pose_trail_.scale.y = 0.05;
	pose_trail_.scale.z = 0.05;
	std_msgs::msg::ColorRGBA color;
	color.r = 1.0;
	color.g = 0.0;
	color.b = 0.0;
	color.a = 1.0;
	builtin_interfaces::msg::Duration t;
	t.sec = 2;
	pose_trail_.lifetime = t;
	pose_trail_.color = color;
	pose_trail_.pose.position.x	= apark_pose_.pose.position.x;
	pose_trail_.pose.position.y	= apark_pose_.pose.position.y;
	pose_trail_.pose.position.z	= apark_pose_.pose.position.z;
	pose_trail_publisher_->publish(pose_trail_);
	trail_id_++;
}

void PX4Telemetry::init_parameters() {
    //Temporary string storage for UTM band
    std::string utm_band_str;

    //Convert park transform to quaternion
    q_utm_to_apark_.setRPY(0, 0, origin_r_);
    q_apark_to_utm_.setRPY(0, 0, -origin_r_);

    //Initialize egm96 (WGS-84) ellipsoid 
    egm96_5_ = std::make_shared<GeographicLib::Geoid>("egm96-5", "", true, true);
    
    // geodesy parameters + sim mode
    this->declare_parameter("origin_x", 0.0);
    this->declare_parameter("origin_y", 0.0);
    this->declare_parameter("origin_r", 0.0);
    this->declare_parameter("utm_zone", 0);
    this->declare_parameter("utm_band", "R");
    this->declare_parameter("sim_mode", false);
    if (
        this->get_parameter("origin_x", origin_x_) && 
        this->get_parameter("origin_y", origin_y_) && 
        this->get_parameter("origin_r", origin_r_) && 
        this->get_parameter("utm_zone", utm_zone_) &&
        this->get_parameter("utm_band", utm_band_str) &&
        this->get_parameter("sim_mode", sim_mode_)
    ) {
        utm_band_ = utm_band_str[0];
        RCLCPP_DEBUG(this->get_logger(), "Park origin set to (%.4f, %.4f), %.4f rad, Zone %d, Band %c", origin_x_, origin_y_, origin_r_, utm_zone_, utm_band_);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Park geodesy parameters not provided.");
        rclcpp::shutdown();
    }
}

void PX4Telemetry::init_publishers() {
    apark_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("autonomy_park/pose", 1);
    gp_origin_publisher_ = this->create_publisher<geographic_msgs::msg::GeoPointStamped>("global_position/set_gp_origin", 1);
    heartbeat_publisher_ = this->create_publisher<swarm_interfaces::msg::Heartbeat>("heartbeat", 1);
	pose_trail_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("pose_trail", 1);
	drone_state_publisher_ = this->create_publisher<swarm_interfaces::msg::DroneState>("drone_state", 1);

    //Autonomy park tf broadcaster
    apark_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    apark_tf_.header.frame_id = "autonomy_park";
    apark_tf_.child_frame_id = px4_id_;
}

void PX4Telemetry::init_subscribers() {
    //Set mavros QOS to keep last
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
    sub_qos.best_effort();
    sub_qos.durability_volatile();

    //Mavros subscribers
    battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>("battery", sub_qos, std::bind(&PX4Telemetry::battery_callback, this, _1));
    altitude_sub_ = this->create_subscription<mavros_msgs::msg::Altitude>("altitude", sub_qos, std::bind(&PX4Telemetry::altitude_callback, this, _1));
    global_lpos_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("global_position/local", sub_qos, std::bind(&PX4Telemetry::global_lpos_callback, this, _1));
    global_gpos_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>("global_position/global", sub_qos, std::bind(&PX4Telemetry::global_gpos_callback, this, _1));
    fleet_manager_heartbeat_sub_ = this->create_subscription<swarm_interfaces::msg::Heartbeat>("/fleet_manager/heartbeat", sub_qos, std::bind(&PX4Telemetry::fleet_manager_heartbeat_callback_, this, _1));
}

void PX4Telemetry::init_service_clients() {
    connect_agent_client_ = this->create_client<swarm_interfaces::srv::ConnectAgent>("/fleet_manager/connect_agent");

    // Wait for connect agent service
    while(!connect_agent_client_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for connect agent service. Exiting.");
            rclcpp::shutdown();
            return;
        }
        RCLCPP_DEBUG(this->get_logger(), "Connect agent service not available, waiting again...");
    }

}

void PX4Telemetry::battery_callback(const sensor_msgs::msg::BatteryState::SharedPtr msg) {
    //Todo: Fix this so it matches readout on Astro (scale via usable battery life)
    battery_voltage_ = (msg->voltage-MIN_VOLTAGE)/(MAX_VOLTAGE-MIN_VOLTAGE)*100.0;
}

//Get local altitude from altitude topic
void PX4Telemetry::altitude_callback(const mavros_msgs::msg::Altitude::SharedPtr msg) {
    //Gazebo sim uses monotonic altitude, physical drone uses local tied to bottom_clearance via lidar
    if (sim_mode_) {
        apark_pose_.pose.position.z = msg->local;
    } else {
        apark_pose_.pose.position.z = msg->local;
    }
    
    altitude_amsl_ = msg->amsl;

    //Set initialization flag
    if (!alt_init_) alt_init_ = true;
}

//Get orientation from UTM pose
void PX4Telemetry::global_lpos_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    //Get UTM orientation
    tf2::Quaternion q_utm, q_apark;
    tf2::fromMsg(msg->pose.pose.orientation, q_utm);
    q_apark = q_utm_to_apark_*q_utm;
    apark_pose_.pose.orientation = tf2::toMsg(q_apark);

    // add twist 

    //Set initialization flag
    if (!lpos_init_) lpos_init_ = true;
}

//Get LL from GPS pos
void PX4Telemetry::global_gpos_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    //Record the time the message came in
    rclcpp::Time now = this->get_clock()->now();

    //Convert GPS coords to DP frame and publish seperately
    auto geo_msg = geographic_msgs::msg::GeoPoint();
    geo_msg.latitude = msg->latitude;
    geo_msg.longitude = msg->longitude;
    geo_msg.altitude = msg->altitude; //Ellipsoidal altitude

    //Convert LLA to UTM
    geodesy::UTMPoint utm_pos;
    geodesy::fromMsg(geo_msg, utm_pos);
    
    //Convert ellipsoidal height to AMSL
    // double geoid_height = GeographicLib::Geoid::GEOIDTOELLIPSOID * (*egm96_5_)(msg->latitude, msg->longitude);
    // double altitude_amsl = msg->altitude - geoid_height;
    // RCLCPP_WARN(this->get_logger(), "AMSL = %.4f meters", altitude_amsl_);

    double dx = utm_pos.easting - origin_x_;
    double dy = utm_pos.northing - origin_y_;
    apark_pose_.pose.position.x = cos(origin_r_)*dx - sin(origin_r_)*dy;
    apark_pose_.pose.position.y = sin(origin_r_)*dx + cos(origin_r_)*dy;

    // apark_pose_.pose.position.z = altitude_amsl - origin_z_;

    apark_pose_.header.stamp = now;

    // RCLCPP_WARN(this->get_logger(), "Apark Z = %.4f meters", apark_pose_.pose.position.z);

    //Publish pose
    apark_pose_.header.frame_id = "autonomy_park";
    this->apark_pose_publisher_->publish(apark_pose_);
	


    //Broadcast TF
    apark_tf_.header.stamp = now;
    apark_tf_.transform.translation.x = apark_pose_.pose.position.x;
    apark_tf_.transform.translation.y = apark_pose_.pose.position.y;
    apark_tf_.transform.translation.z = apark_pose_.pose.position.z;
    apark_tf_.transform.rotation = apark_pose_.pose.orientation;

    apark_tf_broadcaster_->sendTransform(apark_tf_);

    drone_state_.local_pose = apark_pose_;

    //Set initialization flag
    if (!gpos_init_) gpos_init_ = true;
}

void PX4Telemetry::connect_agent_response_callback(rclcpp::Client<swarm_interfaces::srv::ConnectAgent>::SharedFuture future) {
    auto response = future.get();

    if (response->success) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to fleet manager.");
        
        // start sending heartbeats
        heartbeat_timer_ = this->create_wall_timer(0.1s, std::bind(&PX4Telemetry::send_heartbeat, this));
        
        // initialize first heartbeat from fleet manager
        last_heartbeat_time_ = this->get_clock()->now();
        
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to fleet manager.");
    }
}

void PX4Telemetry::fleet_manager_heartbeat_callback_(const swarm_interfaces::msg::Heartbeat::SharedPtr msg) {
    last_heartbeat_time_ = this->get_clock()->now();
}

// service call to connect with fleet manager
void PX4Telemetry::send_connection_request() {
    auto request = std::make_shared<swarm_interfaces::srv::ConnectAgent::Request>();
    request->agent_name = px4_id_;
    request->battery_level = battery_voltage_;

    auto connect_result = connect_agent_client_->async_send_request(request, std::bind(&PX4Telemetry::connect_agent_response_callback, this, _1));
}

void PX4Telemetry::send_heartbeat() {
    auto msg = swarm_interfaces::msg::Heartbeat();
    msg.agent_name = px4_id_;
    msg.battery_level = battery_voltage_;
    msg.timestamp = this->get_clock()->now();
    heartbeat_publisher_->publish(msg);

    // check for fleet manager timeout 
    rclcpp::Time now = this->get_clock()->now();
    if ((now - last_heartbeat_time_).seconds() > heartbeat_timeout_.seconds()) {
        RCLCPP_ERROR(this->get_logger(), "No heartbeat response from fleet manager, SHUTTING DOWN, ADD PROTOCOL HERE");
        rclcpp::shutdown();
    }
}

void PX4Telemetry::publish_drone_state() {
    drone_state_publisher_->publish(drone_state_);
}
