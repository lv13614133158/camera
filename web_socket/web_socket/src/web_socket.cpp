#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <websocket_msgs/msg/websocket.hpp>
#include <boost/beast.hpp>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <proj.h>

using namespace std;
namespace beast = boost::beast;   // from <boost/beast.hpp>
namespace http = beast::http;     // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;       // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;  // from <boost/asio/ip/tcp.hpp>

class WebSocketNode : public rclcpp::Node {
public:
    WebSocketNode()
        : Node("web_socket_node"),
          io_context_(),
          ws_(io_context_),
          work_guard_(net::make_work_guard(io_context_)),
          shutdown_requested_(false)
    {


        connect_to_server("39.106.60.229", "8090");

        read_thread_ = std::thread([this]() { read_loop(); });



  

        // Use a separate thread to run the io_context
        io_context_thread_ = std::thread([this]() { io_context_.run(); });

        publisher_ = this->create_publisher<std_msgs::msg::String>("websocket_out", 10);
        ws_publisher_ = this->create_publisher<std_msgs::msg::String>("ws_out", 10);
        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "ws_out", 10, std::bind(&WebSocketNode::send_data, this, std::placeholders::_1));
        ws_subscription_ = this->create_subscription<websocket_msgs::msg::Websocket>(
        "websocket_in", 10, std::bind(&WebSocketNode::ws_topic_data, this, std::placeholders::_1));

        std::cout<<"WebSocketNode ok"<<std::endl;

    }

    ~WebSocketNode() {
        shutdown_requested_ = true;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }

        // Gracefully close the WebSocket connection
        std::lock_guard<std::mutex> lock(ws_mutex_);
        beast::error_code ec;
        ws_.close(websocket::close_code::normal, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Error closing WebSocket: %s", ec.message().c_str());
        }

        // Stop io_context and join its thread
        work_guard_.reset();
        io_context_.stop();
        if (io_context_thread_.joinable()) {
            io_context_thread_.join();
        }
    }

private:

  
    void ws_topic_data(const websocket_msgs::msg::Websocket::SharedPtr msg)
    {
        char msg_data[1024];
        string type=msg->type;
        string vin=msg->vin;
        int taskld=msg->data.taskld;
        int battery = msg->data.battery;
        string carNo = msg->data.car_no;
        int cupulaWorkStatus = msg->data.cupula_work_status;
        int drivingMode = msg->data.driving_mode;
        int expectedRemainingDrivingTime =msg->data.expected_remaining_driving_time;
        int garbageBinOverflowStatus = msg->data.garbage_bin_overflow_status;
        double latitude_j02 = msg->data.latitude;
        string location = msg->data.location;
        double longitude_j02 = msg->data.longitude;
        string params = msg->data.params;
        int remainingWaterPercentage = msg->data.remaining_water_percentage;
        string remark = msg->data.remark;
        string reportingTime=msg->data.reporting_time;
        string speed = msg->data.speed;
        int sweepWorkStatus =msg->data.sweep_work_status;
        int vehicleStatus = msg->data.vehicle_status;
        string data_vin = msg->data.vin;
        int wateringWorkStatus = msg->data.watering_work_status;


        snprintf(msg_data, sizeof(msg_data),
                 "{ \"type\": \"%s\",\"vin\" : \"%s\", \"data\" :{ \"taskld\":%d,\"battery\": %d, \"carNo\": \"%s\", \"cupulaWorkStatus\": %d, \"drivingMode\": %d, "
                 "\"expectedRemainingDrivingTime\": %d, \"garbageBinOverflowStatus\": %d, "
                 "\"latitude\": %lf, \"location\": \"%s\", \"longitude\": %lf, \"params\": \"%s\", "
                 "\"remainingWaterPercentage\": %d, \"remark\": \"%s\", \"reportingTime\": \"%s\", "
                 "\"speed\": \"%s\", \"sweepWorkStatus\": %d, \"vehicleStatus\": %d, \"vin\": \"%s\", "
                 "\"wateringWorkStatus\": %d } }",
                 type.c_str(),vin.c_str(),taskld,battery, carNo.c_str(), cupulaWorkStatus, drivingMode,
                 expectedRemainingDrivingTime, garbageBinOverflowStatus,
                 latitude_j02, location.c_str(), longitude_j02, params.c_str(), remainingWaterPercentage,
                 remark.c_str(), reportingTime.c_str(), speed.c_str(), sweepWorkStatus, vehicleStatus, data_vin.c_str(),
                 wateringWorkStatus);

        printf("%s\n", msg_data);
        auto message = std_msgs::msg::String();
        message.data = msg_data;  // 将C字符串赋值给消息的data成员
        ws_publisher_->publish(message);
    }

    void connect_to_server(const std::string& host, const std::string& port) {
        tcp::resolver resolver(io_context_);
        auto const results = resolver.resolve(host, port);
        net::connect(ws_.next_layer(), results.begin(), results.end());
        ws_.handshake(host, "/websocket/123");
    }

    void read_loop() {
        rclcpp::WallRate loop_rate(5);
        while (rclcpp::ok()) {
            beast::flat_buffer buffer;
            try {
                {  
                    ws_.read(buffer);
                }
                std_msgs::msg::String msg;
                msg.data = beast::buffers_to_string(buffer.data());
                publisher_->publish(msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Read error: %s", e.what());
            }
           loop_rate.sleep();

        }
    }

    void send_data(const std_msgs::msg::String::SharedPtr msg) {
        ws_.async_write(net::buffer(msg->data),
            [this](beast::error_code ec, std::size_t /*bytes_transferred*/) {
                if (ec) {
                    RCLCPP_ERROR(this->get_logger(), "Send error: %s", ec.message().c_str());
                }
            });
    }
   

private:

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ws_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<websocket_msgs::msg::Websocket>::SharedPtr ws_subscription_;

    net::io_context io_context_;
    websocket::stream<tcp::socket> ws_;
    std::atomic<bool> shutdown_requested_;
    std::mutex ws_mutex_;
    std::thread read_thread_;
    std::thread io_context_thread_;
    net::executor_work_guard<net::io_context::executor_type> work_guard_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebSocketNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
