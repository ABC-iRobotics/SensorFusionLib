/*! \mainpage Structure of the library
 *
 * The package provides 
 * - Kalman filter-based centralized filtering methods and an interface to build complex sensor systems based on their known dynamics and output characteristics
 * - an interface to use ZMQ and flatbuffers-based communication between the sensors and the filter and filter and controller/plotter applications.
 *
 * The source contains CMake building system, tested on windows (Win10 x64), and linux (ubuntu x64).
 *
 * \section intro_sec Installation
 *
 * - Clone the repository
 *
 * - Run CMake
 *    - enable test, set policies for the third parties (easiest way: policy for third parties - download all), disable plot, set install folder
 *    - configure & generate
 *
 * - Build the makefile/solution (cmake --build . from the build folder)
 *
 * - Run ctest (ctest .)
 *
 * - Install package: (cmake --install .)
 *
 * \section usage Usage
 *
 * \subsection peripheries Peripheries
 *
 * The applications which process signals of sensors, must initialize an SF::Periphery class, defining an address where it is available as
 * - "tcp://*:1234" where 1234 is the port
 * - "ipc://" - cannot used on Windows
 * - "inproc://" - cannot used on Windows
 * Then the measured values can be published via SF::Periphery::SendValue(), SF::Periphery::SendVariance(), SF::Periphery::SendValueAndVariance()
 * 
 * (The class SF::Periphery is NOT thread safe! The same thread must initialize the class and call its functions.)
 *
 * If there are Peripheries runned by an other unit, clocks can be synchronized by initializing a clock synchronizer instance via
 *		IClockSynchronizerServer::IClockSynchronizerServerPtr InitClockSynchronizerServer(const std::string& address)
 *
 * \subsection centre Central unit (Data collecting / Filtering / Emulating situations)
 *
 * - Connecting to peripheries for real time filtering / data collecting
 *
 * \subsection  step1 Define a BaseSystem:
 * See KUKAyouBot or Truck classes for 2D localisation examples
 *
 * \subsection step2 Define one or more Sensor-s
 * See INSSensor, IMUSensor, AbsoluthePoseSensor classes for examples
 *
 * \subsection step25 Initialize the setting ID-s as unique ID for the followings
 *	See youBotINSGPS::youBotINSGPS() for an example
 *
 *
 * \subsection step3 Initialize a filter (KalmanFilter and WAUKF is implemented) with the system instances
 *  See youBotINSGPS::youBotINSGPS() for an example
 *
 * \subsection step4 Use the Step(Ts) according to the sensor inputs or use FilteringManager class to do it automatically
 *  See youBotINSGPS::youBotINSGPS() for an example
 *
 * \section apps The implemented applications
 *
 * \subsection app1 TesterApp
 *
 * - Simulation of KUKAyouBot with INSSensor and AbsoluthePoseSensor (GPS)
 * - Filtering via KalmanFilter or WAUKF
 * - Plotting
 *
 * \subsection app2 ZMQPubApp and ZMQSubApp
 * - To show and test ZMQ based communication
 *
 * \subsection app3 youBotINSGPSEmulatorApp and FilterApp (and PlotterApp)
 * - The emulator app realtime simulates the sensor and the platform and plublishes the result via zmq
 * - The FilterApp recieves the zmq msg-s and performs filtering, optionally publishes the results
 * - (The PlotterApp visualizes the results of filtering realtime)
 *
 * \section todo TODO:
 * - to omit mesg.-s based on timestamp?
 * - to apply saturation on state values / covariances?
 * - to extend the methods for time-delayed sensor signals?
 * - to perform more tests on the WAUKF methods?
 * - to implement other adaptive methods?
 * - error handling/communication?
 */
 
 
 
 
 
 
 