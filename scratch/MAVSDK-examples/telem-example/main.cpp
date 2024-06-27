#include <mavsdk/mavsdk.h>
#include <mavsdk/system.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <thread>
#include <chrono>

using namespace mavsdk;

void print_position(const Telemetry::Position& position) {
    std::cout << "Latitude: " << position.latitude_deg << " deg\n"
              << "Longitude: " << position.longitude_deg << " deg\n"
              << "Absolute altitude: " << position.absolute_altitude_m << " m\n"
              << "Relative altitude: " << position.relative_altitude_m << " m\n";
}

void print_velocity(const Telemetry::VelocityNed& velocity) {
    std::cout << "Velocity North: " << velocity.north_m_s << " m/s\n"
              << "Velocity East: " << velocity.east_m_s << " m/s\n"
              << "Velocity Down: " << velocity.down_m_s << " m/s\n";
}

void print_imu(const Telemetry::Imu& imu) {
    std::cout << "Acceleration: ["
              << imu.acceleration_frd.forward_m_s2 << ", "
              << imu.acceleration_frd.right_m_s2 << ", "
              << imu.acceleration_frd.down_m_s2 << "] m/s²\n"
              << "Angular velocity: ["
              << imu.angular_velocity_frd.forward_rad_s << ", "
              << imu.angular_velocity_frd.right_rad_s << ", "
              << imu.angular_velocity_frd.down_rad_s << "] rad/s\n"
              << "Magnetic field: ["
              << imu.magnetic_field_frd.forward_gauss << ", "
              << imu.magnetic_field_frd.right_gauss << ", "
              << imu.magnetic_field_frd.down_gauss << "] gauss\n";
}

int main(int argc, char** argv) {
    // Create a configuration object
    Mavsdk::Configuration config(Mavsdk::ComponentType::CompanionComputer);

    // Instantiate Mavsdk with the configuration
    Mavsdk mavsdk(config);

    std::string connection_url = "serial:///dev/ttyTHS1:115200";  // Serial port and baud rate

    std::cout << "Connecting to " << connection_url << std::endl;

    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << "\n";
        return 1;
    }

    bool system_discovered = false;
    mavsdk.subscribe_on_new_system([&]() {
        auto system = mavsdk.systems().back();
        if (system->is_connected()) {
            std::cout << "Discovered system\n";
            system_discovered = true;
        }
    });

    std::cout << "Waiting to discover system...\n";
    while (!system_discovered) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    auto system = mavsdk.systems().back();
    if (!system->is_connected()) {
        std::cerr << "No connected system found\n";
        return 1;
    }

    auto telemetry = std::make_shared<Telemetry>(system);

    telemetry->subscribe_position(print_position);
    telemetry->subscribe_velocity_ned(print_velocity);
    telemetry->subscribe_imu(print_imu);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
