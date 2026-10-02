// Real custom ROSIDL transport through the retained schema-driven subprocess.
// This is middleware/codec admission, not a mechanical trajectory test.
#include "chrono_ros/ChROSBridge.h"
#include "chrono_ros/ChROSManager.h"
#include <chrono_ros_interfaces/msg/body.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>

#include <chrono>
#include <thread>
#include <string>

namespace {
using namespace chrono::ros;
using Body = chrono_ros_interfaces::msg::Body;

void EnsureMiddleware() {
    if (!rclcpp::ok()) rclcpp::init(0, nullptr);
}

template <class Done>
bool Pump(ChROSManager& manager, const rclcpp::Node::SharedPtr& peer, Done done) {
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(peer);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    double time = 0;
    while (!done()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        EXPECT_TRUE(manager.Update(time, .01));
        time += .01;
        executor.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

Body Sample() {
    Body value;
    value.header.stamp.sec = 1;
    value.header.stamp.nanosec = 250000000;
    value.header.frame_id = "robodyna/custom/body";
    value.pose.position.x = 1.25; value.pose.position.y = -2.5; value.pose.position.z = .125;
    value.pose.orientation.x = .5; value.pose.orientation.y = -.5;
    value.pose.orientation.z = .5; value.pose.orientation.w = -.5;
    value.twist.linear.x = 2; value.twist.linear.y = -4; value.twist.linear.z = 8;
    value.twist.angular.x = .25; value.twist.angular.y = .5; value.twist.angular.z = -.75;
    value.accel.linear.x = -.125; value.accel.linear.y = .25; value.accel.linear.z = 16;
    value.accel.angular.x = -1; value.accel.angular.y = 2; value.accel.angular.z = -3;
    return value;
}

template <class Vector>
void SetVector(ChROSMessage& message, const std::string& prefix, const Vector& value) {
    message.SetDouble(prefix + ".x", value.x);
    message.SetDouble(prefix + ".y", value.y);
    message.SetDouble(prefix + ".z", value.z);
}

template <class Vector>
void CheckVector(const ChROSMessageView& message, const std::string& prefix, const Vector& expected) {
    EXPECT_DOUBLE_EQ(message.GetDouble(prefix + ".x"), expected.x);
    EXPECT_DOUBLE_EQ(message.GetDouble(prefix + ".y"), expected.y);
    EXPECT_DOUBLE_EQ(message.GetDouble(prefix + ".z"), expected.z);
}

TEST(CustomRosMessages, EveryOriginalSchemaLoadsAndPassesNativeCdrAdmission) {
    EnsureMiddleware();
    ChROSManager manager("custom_schema_admission");
    manager.Initialize();
    const char* names[] = {"Body", "DriverInputs", "Rassor", "RassorDriver", "RassorDrumID",
        "RassorSpeedDriverArm", "RassorSpeedDriverDrum", "RassorSpeedDriverWheel", "RassorWheelID",
        "Viper", "ViperDCMotorControl", "ViperDriver", "ViperMotorNoLoadSpeed", "ViperMotorStallTorque",
        "ViperSteeringCommand", "ViperWheelID"};
    for (const char* name : names) {
        SCOPED_TRACE(name);
        const auto type = std::string("chrono_ros_interfaces/msg/") + name;
        // CreatePublisher performs real introspection, CDR validation against
        // generated ROSIDL support, and middleware advertisement in the node.
        auto publisher = manager.GetBridge()->CreatePublisher(std::string("/robodyna/admit/") + name, type);
        ASSERT_TRUE(publisher);
        EXPECT_FALSE(publisher->NewMessage().DescribeType().empty());
    }
}

TEST(CustomRosMessages, FieldBuiltBodyArrivesAsAnActualTypedRosMessage) {
    EnsureMiddleware();
    ChROSManager manager("custom_body_outgoing");
    manager.Initialize();
    auto publisher = manager.GetBridge()->CreatePublisher("/robodyna/custom/body/out", "chrono_ros_interfaces/msg/Body");
    auto peer = std::make_shared<rclcpp::Node>("custom_body_outgoing_peer");
    Body received;
    int count = 0;
    auto subscription = peer->create_subscription<Body>("/robodyna/custom/body/out", 10,
        [&](Body::SharedPtr value) { received = *value; ++count; });
    const auto expected = Sample();
    ASSERT_TRUE(Pump(manager, peer, [&] {
        auto message = publisher->NewMessage();
        message.SetTime("header.stamp", 1.25);
        message.SetString("header.frame_id", expected.header.frame_id);
        SetVector(message, "pose.position", expected.pose.position);
        SetVector(message, "pose.orientation", expected.pose.orientation);
        message.SetDouble("pose.orientation.w", expected.pose.orientation.w);
        SetVector(message, "twist.linear", expected.twist.linear);
        SetVector(message, "twist.angular", expected.twist.angular);
        SetVector(message, "accel.linear", expected.accel.linear);
        SetVector(message, "accel.angular", expected.accel.angular);
        publisher->Publish(message);
        return count > 0;
    }));
    EXPECT_EQ(received, expected);
}

TEST(CustomRosMessages, TypedBodyArrivesThroughTheOriginalFieldAddressedCallback) {
    EnsureMiddleware();
    ChROSManager manager("custom_body_incoming");
    manager.Initialize();
    const auto expected = Sample();
    int count = 0;
    auto subscription = manager.GetBridge()->CreateSubscription("/robodyna/custom/body/in", "chrono_ros_interfaces/msg/Body",
        [&](const ChROSMessageView& message) {
            EXPECT_DOUBLE_EQ(message.GetTimeSec("header.stamp"), 1.25);
            EXPECT_EQ(message.GetString("header.frame_id"), expected.header.frame_id);
            CheckVector(message, "pose.position", expected.pose.position);
            CheckVector(message, "pose.orientation", expected.pose.orientation);
            EXPECT_DOUBLE_EQ(message.GetDouble("pose.orientation.w"), expected.pose.orientation.w);
            CheckVector(message, "twist.linear", expected.twist.linear);
            CheckVector(message, "twist.angular", expected.twist.angular);
            CheckVector(message, "accel.linear", expected.accel.linear);
            CheckVector(message, "accel.angular", expected.accel.angular);
            ++count;
        });
    auto peer = std::make_shared<rclcpp::Node>("custom_body_incoming_peer");
    auto publisher = peer->create_publisher<Body>("/robodyna/custom/body/in", 10);
    ASSERT_TRUE(Pump(manager, peer, [&] { publisher->publish(expected); return count > 0; }));
    EXPECT_GT(subscription->GetReceivedCount(), 0U);
}
}  // namespace
