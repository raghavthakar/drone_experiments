#include <future>
#include <iostream>
#include <fstream>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#define CRUISE_ALT -2.5 //cruising altitude

// Handles Action's result
// it is inline as it is so small and commonly called.
inline void action_error_exit(Action::Result result, const std::string& message)
{
    if (result != Action::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

// Wait until the drone has been discovered
void wait_until_discover(Mavsdk& mavsdk)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    mavsdk.subscribe_on_new_system([&mavsdk, &discover_promise]() {
        const auto system = mavsdk.systems().at(0);

        if (system->is_connected()) {
            std::cout << "Discovered system" << std::endl;
            discover_promise.set_value();
        }
    });

    discover_future.wait();
}

Telemetry::LandedStateCallback
landed_state_callback(Telemetry& telemetry, std::promise<void>& landed_promise)
{
    return [&landed_promise, &telemetry](Telemetry::LandedState landed) {
        switch (landed) {
            case Telemetry::LandedState::OnGround:
                std::cout << "On ground" << std::endl;
                break;
            case Telemetry::LandedState::TakingOff:
                std::cout << "Taking off..." << std::endl;
                break;
            case Telemetry::LandedState::Landing:
                std::cout << "Landing..." << std::endl;
                break;
            case Telemetry::LandedState::InAir:
                std::cout << "Taking off has finished." << std::endl;
                telemetry.subscribe_landed_state(nullptr);
                landed_promise.set_value();
                break;
            case Telemetry::LandedState::Unknown:
                std::cout << "Unknown landed state." << std::endl;
                break;
        }
    };
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string& message)
{
    if (result != Offboard::Result::Success) {
        std::cerr << ERROR_CONSOLE_TEXT << message << result << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string& offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using NED co-ordinates.
 *
 * returns true if everything went well in Offboard control, exits with a log
 * otherwise.
 */
bool offboard_control_ned(mavsdk::Offboard& offboard, std::vector<float> coords)
{
    //CONVERT O-1000 COORDINATES TO -11-+9 (TO ACCOUNT FOR DRONE STARTING AT (1, 1))
    // COLUMN NUMBER INCREASES LEFT TO RIGHT, SAME AS EARLIER, BUT ROW NUMBER MUST
    //MAP TO NORTH DISTANCE, WHICH INCREASES TOP TO BOTTOM.
    //east/column number
    coords[0]/=50;
    coords[0]-=11;

    //north/ row number
    coords[1]=1000-coords[1];
    coords[1]/=50;
    coords[1]-=11;
    std::cout <<coords[1]<< ", "<<coords[0]<<'\n';
    const std::string offb_mode = "NED";
    // Send it once before starting offboard, otherwise it will be rejected.
    const Offboard::VelocityNedYaw stay{};
    offboard.set_velocity_ned(stay);

    //Attempt to start offboard mode, handle errors.
    Offboard::Result offboard_result = offboard.start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    // Co-ordinates are relative to start frame
    offboard_log(offb_mode, "Going to next waypoint");
    Offboard::PositionNedYaw next_waypoint{};
    next_waypoint.north_m=coords[1];
    next_waypoint.east_m=coords[0];
    next_waypoint.down_m=CRUISE_ALT;
    offboard.set_position_ned(next_waypoint);
    sleep_for(seconds(2));

    // Now, stop offboard mode.
    offboard_result = offboard.stop();
    offboard_error_exit(offboard_result, "Offboard stop failed: ");
    offboard_log(offb_mode, "Offboard stopped");

    return true;
}

int main(int argc, char** argv)
{
    Mavsdk mavsdk;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = mavsdk.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    wait_until_discover(mavsdk);

    // System got discovered.
    auto system = mavsdk.systems().at(0);

    // ----------------ABOVE THIS MUST BE IN EVERY APPLICATION------------------

    //Create instances of Action, Telemetry and Offboard
    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};

    // Wait till the telemetry health is not ok
    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "System is ready" << std::endl;

    // IDK
    std::promise<void> in_air_promise;
    auto in_air_future = in_air_promise.get_future();

    // Attempt to arm the drone. Handle the action error.
    Action::Result arm_result = action.arm();
    action_error_exit(arm_result, "Arming failed");
    std::cout << "Armed" << std::endl;

    // Take off. Handle the action error if any.
    Action::Result takeoff_result = action.takeoff();
    action_error_exit(takeoff_result, "Takeoff failed");

    telemetry.subscribe_landed_state(landed_state_callback(telemetry, in_air_promise));
    in_air_future.wait();

    // //store the next waypoint coordinates {colnum, rownum}
    std::vector<float> coords;
    //Read the waypoints from csv file
    std::ifstream path_file;
    path_file.open("path.csv");
    std::string line, word;

    //read the cooridnates from the csv file
    while(!path_file.eof())
    {
        coords.clear();
        path_file>>line;
        std::stringstream s(line);//break the csv line into words
        while(getline(s, word, ','))
        {
            //push the coordinate into the coordinates vector
            coords.push_back(std::stoi(word));
        }
        //  using local NED co-ordinates for offboard control
        bool ret = offboard_control_ned(offboard, coords);
        if (ret == false) {
            return EXIT_FAILURE;
        }
    }

    const Action::Result land_result = action.land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for
    // a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
