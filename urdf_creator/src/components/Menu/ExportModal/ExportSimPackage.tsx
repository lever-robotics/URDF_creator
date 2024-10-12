import type React from "react";
import "./Export.css";
import { handleDownload } from "../../../utils/HandleDownload";
import type ThreeScene from "../../ThreeDisplay/ThreeScene";

type Props = {
    onClose: () => void;
    scene: ThreeScene;
    projectTitle: string;
};

const ExportSimPackage: React.FC<Props> = ({
    onClose,
    scene,
    projectTitle,
}) => {
    const handleURDFExport = () => {
        handleDownload(scene, "urdfpackage", projectTitle);
    };

    return (
        <div className="des-container">
            <div className="header">
                <div className="export-title">
                    <h3>Export a Robot Description package</h3>
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
                        Export ROS2 Package
                    </button>
                </div>
            </div>
            <div className="description-container">
                <h3>Description: </h3>
                <p>
                    This downloaded package contains a launch file that can be
                    used to launch the robot model in RViz and Gazebo. It
                    includes the necessary robot description files and
                    configuration for visualization and simulation. Place this
                    folder in the src folder of your ROS development{" "}
                    <a href="https://roboeverything.com">workspace</a>, build it
                    then run the example executable.
                </p>
                <div className="terminal-container">
                    <code>
                        ros2 launch {projectTitle}_description {projectTitle}
                        .launch.py
                    </code>
                </div>
            </div>
            <div className="image-description-container">
                <div className="image-container">
                    <img
                        src={`${process.env.PUBLIC_URL}/statics/folders_white.png`}
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

export default ExportSimPackage;
