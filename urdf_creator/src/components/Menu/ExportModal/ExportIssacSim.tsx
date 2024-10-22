import type React from "react";
import "./Export.css";
import { useRef } from "react";
import { handleDownload } from "../../../utils/HandleDownload";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";

type Props = {
    onClose: () => void;
    scene: ThreeScene;
    projectTitle: string;
};

const ExportIssacSim: React.FC<Props> = ({ onClose, scene, projectTitle }) => {
    const handleURDFExport = () => {
        handleDownload(scene, "issacsimpackage", projectTitle);
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="export-title">
                    <h3>Export ROS2 Issac Sim Package</h3>
                </div>
                <div className="export-box">
                    <button
                        className="export-button"
                        onClick={() => {
                            handleURDFExport();
                            onClose();
                        }}
                        type="button"
                    >
                        Export Package
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Installation instructions: </h3>
                <p>
                    This package containes all assets for simulating a robot in
                    Issac Sim including a USD file that includes simulation
                    information for accuratly simulating your robot. Also
                    included are ROS2 launch files that will startup all
                    important ROS2 nodes for simulation.
                </p>
                <div className="steps-container">
                    <h3>Steps:</h3>
                    <ol>
                        <li>Unzip the downloaded package.</li>
                        <li>
                            Place the {projectTitle}_gazebo_package folder in
                            the src directory of your ROS{" "}
                            <a href="https://roboeverything.com">workspace</a>
                        </li>
                        <li>Return to the base of the ROS2 workspace </li>
                        <li>Build the package with the following command</li>
                        <div className="terminal-container">
                            <code>
                                colcon build --packages-select {projectTitle}
                                _description
                            </code>
                        </div>
                        <li>Source the workspace with the following command</li>
                        <div className="terminal-container">
                            <code>source install/setup.bash</code>
                        </div>
                        <li>Run the launch file using the command: </li>
                        <div className="terminal-container">
                            <code>
                                ros2 launch {projectTitle}_description{" "}
                                {projectTitle}.launch.py
                            </code>
                        </div>
                    </ol>
                    <p>
                        For more info on installing this into your ROS system,
                        following this{" "}
                        <a href="https://roboeverything.com">documentaion</a>
                    </p>
                </div>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img
                        src={`${import.meta.env.BASE_URL}/statics/folders_white.png`}
                        alt="Folder file structure"
                        className="urdfgraphic"
                    />
                </div>
                <div className="image-description">
                    <h3>File Structure:</h3>
                    <ul>
                        <li className="bullet-point">
                            Robot_description - A ROS package to be placed in
                            src folder of robot workspace
                        </li>
                        <li className="bullet-point">
                            robot.urdf - The robot description in xml format to
                            be used by ROS
                        </li>
                        <li className="bullet-point">
                            robot.launch - Launch file that will load the urdf
                            and load it into statepublisher that publishes
                            transformations between all the joints of the robot.
                            This file also launches the robot into rviz to
                            visualize its current state and gazebo which
                            simulates the robot.
                        </li>
                        <li className="bullet-point">
                            robot.sdf - This is the description of the robot for
                            simulation which can contain more info about the
                            links and joints such as friction
                        </li>
                        <li className="bullet-point">
                            robot.rviz - This is a file that configures the rviz
                            enviroment to visulize the robot description
                        </li>
                    </ul>
                </div>
            </div>
            <div className="description-container">
                <p>
                    To understand more on the robot_description file structure
                    and its use, refer to the{" "}
                    <a href="https://roboeverything.com">documentation</a>.
                </p>
            </div>
        </div>
    );
};

export default ExportIssacSim;
