import type React from "react";
import { useState } from "react";
import Project from "./Project";
import "./Project.css";

type Props = {
    handleProjectClick: (projectPath: string, title: string) => Promise<void>;
    onClose: () => void;
};

export type ProjectType = {
    title: string;
    description: string;
    image: string;
    width: string;
    height: string;
    path: string;
};

// h1 {
//   font-size: 3.2em;
//   line-height: 1.1;
// }

const ProjectDisplayer: React.FC<Props> = ({ handleProjectClick, onClose }) => {
    const [selectedProject, setSelectedProject] = useState(null);
    const [projects, setProjects]: [
        ProjectType[],
        (set: ProjectType[]) => void,
    ] = useState([
        {
            title: "Turtlebot3 Burger",
            description: "Turtlebot Designed by ROBOTIS",
            path: `${import.meta.env.BASE_URL}/statics/turtlebot3_burger.gltf`,
            image: `${import.meta.env.BASE_URL}/statics/turtlebot3_burger.png`,
            width: "100",
            height: "200",
        },
        {
            title: "Turtlebot3 Waffle",
            description: "Turtlebot Designed by ROBOTIS",
            path: `${import.meta.env.BASE_URL}/statics/turtlebot3_burger.gltf`,
            image: `${import.meta.env.BASE_URL}/statics/turtlebot3_waffle.png`,
            width: "100",
            height: "200",
        },
        {
            title: "Fun Robot",
            description: "Test out the functionality of ROS2",
            path: `${import.meta.env.BASE_URL}/statics/green_robot.gltf`,
            image: `${import.meta.env.BASE_URL}/statics/green_robot.png`,
            width: "200",
            height: "200",
        },
        // Add more projects
    ]);

    return (
        <>
            <h1>Project Manager</h1>
            <div className="project-displayer">
                {projects.map((project) => (
                    <Project
                        key={project.title}
                        project={project}
                        handleProjectClick={handleProjectClick}
                        onClose={onClose}
                    />
                ))}
            </div>
        </>
    );
};

export default ProjectDisplayer;
