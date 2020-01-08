/*! \mainpage Structure of the library
 *
 * The package provides 
 * - Kalman filter-based centralized filtering methods and an interface to build complex sensor systems based on their known dynamics and output characteristics
 * - an interface to use ZMQ and flatbuffers-based communication between the sensors and the filter and filter and controller/plotter applications.
 *
 * The source contains CMake building system (tested on windows).
 *
 * \section intro_sec Structure of the package
 *
 *	\image html structure.png
 *  \image latex structure.eps
 *
 *
 * \section install_sec Installation
 *
 * Third Parties:
 * - Download Eigen and install it via CMake (http://eigen.tuxfamily.org/index.php?title=Main_Page)
 * - Download and build ZeroMQ (http://zeromq.org/)
 * - Download and build CPPZMQ (https://github.com/zeromq/cppzmq)
 * - (Download and build OpenCV)
 *
 * Generate Visual Studio Solution:
 * - run CMake, set the pathes manually if it cannot find the installed libraries, as
 *	\image html CMakeOptions.png
 *  \image latex CMakeOptions.png
 * - generate, build and use it...
 *
 *
 *
 * \section usage Usage
 *
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
 * - to use poll?
 * - to omit mesg.-s based on timestamp?
 * - to apply saturation on state values / covariances?
 * - to extend the methods for time-delayed sensor signals?
 * - to perform more tests on the WAUKF methods?
 * - to implement other adaptive methods?
 * - error handling/communication?
 */
 
 
 
 
 
 
 