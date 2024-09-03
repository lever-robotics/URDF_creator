import React, { useState } from "react";
import Project from './Project';
import './Project.css';

const ProjectDisplayer = ({ handleProjectClick, onClose }) => {
  const [selectedProject, setSelectedProject] = useState(null);
  const [projects, setProjects] = useState([
      { title: 'New Project', description: 'Create a new URDF Project', image:'/statics/PlusIcon.jpg', width: '200', height: '200' },
      { title: 'Turtlebot3 Burger', description: 'Turtlebot Designed by ROBOTIS', path: '/statics/turtlebot3_burger.gltf', image: '/statics/turtlebot3_burger.png', width: '200', height: '200' },
      { title: 'Fun Robot', description: 'Test out the functionality of ROS2', path: '/statics/green_robot.gltf', image: '/statics/green_robot.png', width: '200', height: '200' },
      { title: 'OldMain', description: 'Haha we go to college here', image:'/statics/oldmain.jpg', width: '200', height: '200' },
      // Add more projects
    ]);
    
      return (
        <>
          <h1>Project Manager</h1>
          <div className="project-displayer">
          {projects.map((project, index) => (
            <Project key={index} project={project} handleProjectClick={handleProjectClick} onClose={onClose}/>))}
          </div>
        </>
      );
};

export default ProjectDisplayer;