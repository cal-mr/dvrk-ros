/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */
/*

  Modified From Original File By Adnan Munawar (WPI) and Adithya Murali (UC Berkeley)
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-07

  (C) Copyright 2013-2014 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
#include <map>

// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsQtApplication.h>
#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/mtsRobotIO1394QtWidgetFactory.h>
#include <sawControllers/mtsPIDQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsole.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitConsoleQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArmQtWidget.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitUDPStreamer.h>
#include <sawControllers/mtsTeleOperation.h>
#include <sawControllers/mtsTeleOperationQtWidget.h>

#include <cisstMultiTask/mtsCollectorFactory.h>
#include <cisstMultiTask/mtsCollectorQtFactory.h>
#include <cisstMultiTask/mtsCollectorQtWidget.h>

#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QTabWidget>

#include <json/json.h>
#include <signal.h>

void mySigintHandler(int sig)
{
  QCoreApplication::exit(0);
}

void fileExists(const std::string & description, const std::string & filename)
{
    if (!cmnPath::Exists(filename)) {
        std::cerr << "File not found for " << description
                  << ": " << filename << std::endl;
        exit(-1);
    } else {
        std::cout << "File found for " << description
                  << ": " << filename << std::endl;
    }
}


int main(int argc, char ** argv)
{
    // configuration
    const double periodIO = 0.5 * cmn_ms;
    const double periodKinematics = 2.0 * cmn_ms;
    const double periodTeleop = 2.0 * cmn_ms;
    const double periodUDP = 20.0 * cmn_ms;

    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClass("mtsIntuitiveResearchKit", CMN_LOG_ALLOW_VERBOSE);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // -------------- Some ROS Hack ------------
    signal(SIGINT, mySigintHandler);
    // ------------------------------------------

    // parse options
    cmnCommandLineOptions options;
    int firewirePort = 0;
    std::string gcmip = "-1";
    std::string jsonMainConfigFile;
    std::string jsonCollectionConfigFile;
    typedef std::map<std::string, std::string> ConfigFilesType;
    ConfigFilesType configFiles;
    std::string masterName, slaveName;

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonMainConfigFile);

    options.AddOptionOneValue("f", "firewire",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &firewirePort);
    options.AddOptionOneValue("g", "gcmip",
                              "global component manager IP address",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &gcmip);

    options.AddOptionOneValue("c", "collection-config",
                              "json configuration file for data collection",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &jsonCollectionConfigFile);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // make sure the json config file exists and can be parsed
    fileExists("JSON configuration", jsonMainConfigFile);
    std::ifstream jsonStream;
    jsonStream.open(jsonMainConfigFile.c_str());

    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        std::cerr << "Error: failed to parse configuration\n"
                  << jsonReader.getFormattedErrorMessages();
        return -1;
    }

    // start initializing the cisstMultiTask manager
    std::cout << "FirewirePort: " << firewirePort << std::endl;

    std::string processname = "dvTeleop";
    mtsManagerLocal * componentManager = 0;
    if (gcmip != "-1") {
        try {
            componentManager = mtsManagerLocal::GetInstance(gcmip, processname);
        } catch(...) {
            std::cerr << "Failed to get GCM instance." << std::endl;
            return -1;
        }
    } else {
        componentManager = mtsManagerLocal::GetInstance();
    }

    // create a Qt application and tab to hold all widgets
    mtsQtApplication * qtAppTask = new mtsQtApplication("QtApplication", argc, argv);
    qtAppTask->Configure();
    componentManager->AddComponent(qtAppTask);

    // console
    mtsIntuitiveResearchKitConsole * console = new mtsIntuitiveResearchKitConsole("console");
    componentManager->AddComponent(console);
    mtsIntuitiveResearchKitConsoleQtWidget * consoleGUI = new mtsIntuitiveResearchKitConsoleQtWidget("consoleGUI");
    componentManager->AddComponent(consoleGUI);
    // connect consoleGUI to console
    componentManager->Connect("console", "Main", "consoleGUI", "Main");

    // organize all widgets in a tab widget
    QTabWidget * tabWidget = new QTabWidget;
    tabWidget->addTab(consoleGUI, "Main");

    // IO is shared accross components
    mtsRobotIO1394 * io = new mtsRobotIO1394("io", periodIO, firewirePort);

    // find name of button event used to detect if operator is present
    std::string operatorPresentComponent = jsonConfig["operator-present"]["component"].asString();
    std::string operatorPresentInterface = jsonConfig["operator-present"]["interface"].asString();
    //set defaults
    if (operatorPresentComponent == "") {
        operatorPresentComponent = "io";
    }
    if (operatorPresentInterface == "") {
        operatorPresentInterface = "COAG";
    }
    std::cout << "Using \"" << operatorPresentComponent << "::" << operatorPresentInterface
              << "\" to detect if operator is present" << std::endl;

    // setup io defined in the json configuration file
    const Json::Value pairs = jsonConfig["pairs"];
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        // master
        Json::Value jsonMaster = pairs[index]["master"];
        std::string armName = jsonMaster["name"].asString();
        std::string ioFile = jsonMaster["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
        // slave
        Json::Value jsonSlave = pairs[index]["slave"];
        armName = jsonSlave["name"].asString();
        ioFile = jsonSlave["io"].asString();
        fileExists(armName + " IO", ioFile);
        io->Configure(ioFile);
    }

    componentManager->AddComponent(io);

    // connect ioGUIMaster to io
    mtsRobotIO1394QtWidgetFactory * robotWidgetFactory = new mtsRobotIO1394QtWidgetFactory("robotWidgetFactory");
    componentManager->AddComponent(robotWidgetFactory);
    componentManager->Connect("robotWidgetFactory", "RobotConfiguration", "io", "Configuration");
    robotWidgetFactory->Configure();

    // add all IO GUI to tab
    mtsRobotIO1394QtWidgetFactory::WidgetListType::const_iterator iterator;
    for (iterator = robotWidgetFactory->Widgets().begin();
         iterator != robotWidgetFactory->Widgets().end();
         ++iterator) {
        tabWidget->addTab(*iterator, (*iterator)->GetName().c_str());
    }
    tabWidget->addTab(robotWidgetFactory->ButtonsWidget(), "Buttons");

    // setup arms defined in the json configuration file
    for (unsigned int index = 0; index < pairs.size(); ++index) {
        Json::Value jsonMaster = pairs[index]["master"];
        std::string masterName =  jsonMaster["name"].asString();
        std::string masterPIDFile = jsonMaster["pid"].asString();
        std::string masterKinematicFile = jsonMaster["kinematic"].asString();
        std::string masterUDPIP = jsonMaster["UDP-IP"].asString();
        short masterUDPPort = jsonMaster["UDP-port"].asInt();

        fileExists(masterName + " PID", masterPIDFile);
        fileExists(masterName + " kinematic", masterKinematicFile);
        mtsIntuitiveResearchKitConsole::Arm * mtm
                = new mtsIntuitiveResearchKitConsole::Arm(masterName, io->GetName());
        mtm->ConfigurePID(masterPIDFile);
        mtm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_MTM,
                          masterKinematicFile, periodKinematics);
        console->AddArm(mtm);

        if (masterUDPIP != "" || masterUDPPort != 0) {
            if (masterUDPIP != "" && masterUDPPort != 0) {
                std::cout << "Adding UDPStream component for master " << masterName << " to " << masterUDPIP << ":" << masterUDPPort << std::endl;
                mtsIntuitiveResearchKitUDPStreamer * streamer =
                    new mtsIntuitiveResearchKitUDPStreamer(masterName + "UDP", periodUDP, masterUDPIP, masterUDPPort);
                componentManager->AddComponent(streamer);
                // connect to mtm interface to get cartesian position
                componentManager->Connect(streamer->GetName(), "Robot", mtm->Name(), "Robot");
                // connect to io to get clutch events
                componentManager->Connect(streamer->GetName(), "Clutch", "io", "CLUTCH");
                // connect to io to get coag events
                componentManager->Connect(streamer->GetName(), "Coag", "io", "COAG");
            } else {
                std::cerr << "Error for master arm " << masterName << ", you can provided UDP-IP w/o UDP-port" << std::endl;
                exit(-1);
            }
        }

        Json::Value jsonSlave = pairs[index]["slave"];
        std::string slaveName =  jsonSlave["name"].asString();
        std::string slavePIDFile = jsonSlave["pid"].asString();
        std::string slaveKinematicFile = jsonSlave["kinematic"].asString();

        fileExists(slaveName + " PID", slavePIDFile);
        fileExists(slaveName + " kinematic", slaveKinematicFile);
        mtsIntuitiveResearchKitConsole::Arm * psm
                = new mtsIntuitiveResearchKitConsole::Arm(slaveName, io->GetName());
        psm->ConfigurePID(slavePIDFile);
        psm->ConfigureArm(mtsIntuitiveResearchKitConsole::Arm::ARM_PSM,
                          slaveKinematicFile, periodKinematics);
        console->AddArm(psm);

        // PID Master GUI
        std::string masterPIDName = masterName + " PID";
        mtsPIDQtWidget * pidMasterGUI = new mtsPIDQtWidget(masterPIDName, 8);
        pidMasterGUI->Configure();
        componentManager->AddComponent(pidMasterGUI);
        componentManager->Connect(pidMasterGUI->GetName(), "Controller", mtm->PIDComponentName(), "Controller");
        tabWidget->addTab(pidMasterGUI, masterPIDName.c_str());

        // PID Slave GUI
        std::string slavePIDName = slaveName + " PID";
        mtsPIDQtWidget * pidSlaveGUI = new mtsPIDQtWidget(slavePIDName, 7);
        pidSlaveGUI->Configure();
        componentManager->AddComponent(pidSlaveGUI);
        componentManager->Connect(pidSlaveGUI->GetName(), "Controller", psm->PIDComponentName(), "Controller");
        tabWidget->addTab(pidSlaveGUI, slavePIDName.c_str());

        // Master GUI
        mtsIntuitiveResearchKitArmQtWidget * masterGUI = new mtsIntuitiveResearchKitArmQtWidget(mtm->Name() + "GUI");
        masterGUI->Configure();
        componentManager->AddComponent(masterGUI);
        tabWidget->addTab(masterGUI, mtm->Name().c_str());
        // connect masterGUI to master
        componentManager->Connect(masterGUI->GetName(), "Manipulator", mtm->Name(), "Robot");

        // Slave GUI
        mtsIntuitiveResearchKitArmQtWidget * slaveGUI = new mtsIntuitiveResearchKitArmQtWidget(psm->Name() + "GUI");
        slaveGUI->Configure();
        componentManager->AddComponent(slaveGUI);
        tabWidget->addTab(slaveGUI, psm->Name().c_str());
        // connect slaveGUI to slave
        componentManager->Connect(slaveGUI->GetName(), "Manipulator", psm->Name(), "Robot");

        // Teleoperation
        std::string teleName = masterName + "-" + slaveName;
        mtsTeleOperationQtWidget * teleGUI = new mtsTeleOperationQtWidget(teleName + "GUI");
        teleGUI->Configure();
        componentManager->AddComponent(teleGUI);
        tabWidget->addTab(teleGUI, teleName.c_str());
        mtsTeleOperation * tele = new mtsTeleOperation(teleName, periodTeleop);
        // Default orientation between master and slave
        vctMatRot3 master2slave;
        master2slave.Assign(-1.0, 0.0, 0.0,
                             0.0,-1.0, 0.0,
                             0.0, 0.0, 1.0);
        tele->SetRegistrationRotation(master2slave);
        componentManager->AddComponent(tele);
        // connect teleGUI to tele
        componentManager->Connect(teleGUI->GetName(), "TeleOperation", tele->GetName(), "Setting");

        componentManager->Connect(tele->GetName(), "Master", mtm->Name(), "Robot");
        componentManager->Connect(tele->GetName(), "Slave", psm->Name(), "Robot");
        componentManager->Connect(tele->GetName(), "Clutch", "io", "CLUTCH");
        componentManager->Connect(tele->GetName(), "OperatorPresent", operatorPresentComponent, operatorPresentInterface);
        componentManager->Connect(tele->GetName(), "CAM+", "io", "CAM+");
        componentManager->Connect(tele->GetName(), "CAM-", "io", "CAM-");
    }

    // configure data collection if needed
    if (options.IsSet("collection-config")) {
        // make sure the json config file exists
        fileExists("JSON data collection configuration", jsonCollectionConfigFile);

        mtsCollectorFactory * collectorFactory = new mtsCollectorFactory("collectors");
        collectorFactory->Configure(jsonCollectionConfigFile);
        componentManager->AddComponent(collectorFactory);
        collectorFactory->Connect();

        mtsCollectorQtWidget * collectorQtWidget = new mtsCollectorQtWidget();
        tabWidget->addTab(collectorQtWidget, "Collection");

        mtsCollectorQtFactory * collectorQtFactory = new mtsCollectorQtFactory("collectorsQt");
        collectorQtFactory->SetFactory("collectors");
        componentManager->AddComponent(collectorQtFactory);
        collectorQtFactory->Connect();
        collectorQtFactory->ConnectToWidget(collectorQtWidget);
    }


    ///////////////////////  ROS Bridge   ////////////////////////////////////////////

    //Note: the period of rosBridge was changed from 20 to 2 milliseconds, Pose changed to PoseStamped
    // Starting ROS-Bridge Here
    mtsROSBridge rosBridge("dvrk", 2 * cmn_ms, true, false);

    ///////////////////////  Pose, Joint, Robot State   ////////////////////

    rosBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
                "MTML", "GetPositionJoint", "/dvrk_mtml/joint_position_current");
    rosBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
                "MTMR", "GetPositionJoint", "/dvrk_mtmr/joint_position_current");
//    rosBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
//                "PSM1", "GetPositionJoint", "/dvrk_psm1/joint_position_current");
//    rosBridge.AddPublisherFromReadCommand<prmPositionJointGet, sensor_msgs::JointState>(
//                "PSM2", "GetPositionJoint", "/dvrk_psm2/joint_position_current");


//    const std::vector<std::string> jointNamesPSM1;

    std::string jointNamesPSM_array[] = {"two_outer_yaw_joint",
                                         "two_outer_pitch_joint_1",
                                         "two_outer_insertion_joint",
                                         "two_outer_roll_joint",
                                         "two_outer_wrist_pitch_joint",
                                         "two_outer_wrist_yaw_joint",
                                         "two_outer_wrist_open_angle_joint_1",
                                         "one_outer_yaw_joint",
                                         "one_outer_pitch_joint_1",
                                         "one_outer_insertion_joint",
                                         "one_outer_roll_joint",
                                         "one_outer_wrist_pitch_joint",
                                         "one_outer_wrist_yaw_joint",
                                         "one_outer_wrist_open_angle_joint_1"};
//    std::string jointNamesPSM_array[] = {'one_outer_yaw_joint', 'one_outer_pitch_joint_1', 'one_outer_pitch_joint_2', 'one_outer_pitch_joint_3', 'one_outer_pitch_joint_4', 'one_outer_pitch_joint_5', 'one_outer_insertion_joint', 'one_outer_roll_joint', 'one_outer_wrist_pitch_joint', 'one_outer_wrist_yaw_joint', 'one_outer_wrist_open_angle_joint_1', 'one_outer_wrist_open_angle_joint_2', 'two_outer_yaw_joint', 'two_outer_pitch_joint_1', 'two_outer_pitch_joint_2', 'two_outer_pitch_joint_3', 'two_outer_pitch_joint_4', 'two_outer_pitch_joint_5', 'two_outer_insertion_joint', 'two_outer_roll_joint', 'two_outer_wrist_pitch_joint', 'two_outer_wrist_yaw_joint', 'two_outer_wrist_open_angle_joint_1', 'two_outer_wrist_open_angle_joint_2'};
    std::vector< std::string > jointNamesPSM( jointNamesPSM_array, jointNamesPSM_array + ( sizeof ( jointNamesPSM_array ) /  sizeof ( std::string ) ) );
    rosBridge.AddJointPublisher("PSM1", "GetPositionJoint", "PSM2", "GetPositionJoint", jointNamesPSM);

    rosBridge.AddPublisherFromReadCommandStamped("PSM1", "GetPositionCartesian", "/dvrk_psm1/joint_position_cartesian", "/two_remote_center_link");
    rosBridge.AddPublisherFromReadCommandStamped("PSM2", "GetPositionCartesian", "/dvrk_psm2/joint_position_cartesian", "/one_remote_center_link");
    rosBridge.AddPublisherFromReadCommandStamped("MTML", "GetPositionCartesian", "/dvrk_mtml/joint_position_cartesian", "");
    rosBridge.AddPublisherFromReadCommandStamped("MTMR", "GetPositionCartesian", "/dvrk_mtmr/joint_position_cartesian", "");

    rosBridge.AddSubscriberToWriteCommand<std::string , std_msgs::String>(
                "PSM1", "SetRobotControlState", "/dvrk_psm1/set_robot_state");
    rosBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::PoseStamped>(
                "PSM1", "SetPositionCartesian", "/dvrk_psm1/set_cartesian_pose");

    rosBridge.AddSubscriberToWriteCommand<std::string , std_msgs::String>(
                "PSM2", "SetRobotControlState", "/dvrk_psm2/set_robot_state");
    rosBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::PoseStamped>(
                "PSM2", "SetPositionCartesian", "/dvrk_psm2/set_cartesian_pose");
    rosBridge.AddSubscriberToWriteCommand<std::string , std_msgs::String>(
                "MTML", "SetRobotControlState", "/dvrk_mtml/set_robot_state");
    rosBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::PoseStamped>(
                "MTML", "SetPositionCartesian", "/dvrk_mtml/set_cartesian_pose");
    rosBridge.AddSubscriberToWriteCommand<std::string , std_msgs::String>(
                "MTMR", "SetRobotControlState", "/dvrk_mtmr/set_robot_state");
    rosBridge.AddSubscriberToWriteCommand<prmPositionCartesianSet, geometry_msgs::PoseStamped>(
                "MTMR", "SetPositionCartesian", "/dvrk_mtmr/set_cartesian_pose");

    ///////////////////////  Gripper   ///////////////////////

    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "MTML", "GetGripperPosition", "/dvrk_mtml/gripper_position");
    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "MTMR", "GetGripperPosition", "/dvrk_mtmr/gripper_position");

    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "PSM1", "GetOpenAngle", "/dvrk_psm1/gripper_position");
    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "PSM2", "GetOpenAngle", "/dvrk_psm2/gripper_position");

    rosBridge.AddSubscriberToWriteCommand<double, std_msgs::Float32>(
                "PSM1", "SetOpenAngle", "/dvrk_psm1/set_gripper_position");
    rosBridge.AddSubscriberToWriteCommand<double, std_msgs::Float32>(
                "PSM2", "SetOpenAngle", "/dvrk_psm2/set_gripper_position");


    ///////////////////////////  Coag, Clutch & Scale //////////////////////////////////////

    rosBridge.AddPublisherFromEventWriteContinuous<prmEventButton, std_msgs::Bool>(
                "COAG", "Button", "/dvrk_pedal/coag");
    rosBridge.AddPublisherFromEventWriteContinuous<prmEventButton, std_msgs::Bool>(
                "CLUTCH", "Button", "/dvrk_pedal/clutch");
    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "SCALE1", "GetScale", "/dvrk_pedal/scale1");
    rosBridge.AddPublisherFromReadCommand<double, std_msgs::Float32>(
                "SCALE2", "GetScale", "/dvrk_pedal/scale2");

    componentManager->AddComponent(&rosBridge);
    componentManager->Connect(rosBridge.GetName(), "MTML", "MTML", "Robot");
    componentManager->Connect(rosBridge.GetName(), "MTMR", "MTMR", "Robot");
    componentManager->Connect(rosBridge.GetName(), "PSM1", "PSM1", "Robot");
    componentManager->Connect(rosBridge.GetName(), "PSM2", "PSM2", "Robot");
    componentManager->Connect(rosBridge.GetName(), "CLUTCH", "io", "CLUTCH");
    componentManager->Connect(rosBridge.GetName(), "COAG", operatorPresentComponent, operatorPresentInterface);
    componentManager->Connect(rosBridge.GetName(), "SCALE1", "MTMR-PSM1", "Scale");
    componentManager->Connect(rosBridge.GetName(), "SCALE2", "MTML-PSM2", "Scale");

    // show all widgets
    tabWidget->show();

    //-------------- create the components ------------------
    io->CreateAndWait(2.0 * cmn_s); // this will also create the pids as they are in same thread
    io->StartAndWait(2.0 * cmn_s);

    // start all other components
    componentManager->CreateAllAndWait(2.0 * cmn_s);
    componentManager->StartAllAndWait(2.0 * cmn_s);

    // QtApplication will run in main thread and return control
    // when exited.

    componentManager->KillAllAndWait(2.0 * cmn_s);
    componentManager->Cleanup();

    // delete dvgc robot
    delete io;
    delete robotWidgetFactory;

    // stop all logs
    cmnLogger::Kill();

    return 0;
}









