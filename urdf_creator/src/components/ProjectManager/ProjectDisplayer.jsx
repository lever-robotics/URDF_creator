import React, { useState } from "react";
import Project from './Project';

const ProjectDisplayer = ({ handleProjectClick }) => {
  const [selectedProject, setSelectedProject] = useState(null);
  const [projects, setProjects] = useState([
      { title: 'R2D2', description: 'A legend among bots', path: '/statics/R2D2.gltf', image: '/statics/r2d2.png', width: '200', height: '200' },
      { title: 'OldMain', description: 'Haha we go to college here', image:'/statics/oldmain.jpg', width: '200', height: '200' },
      // Add more projects
    ]);
    
      return (
        <div className="project-displayer">
          {projects.map((project, index) => (
            <Project key={index} project={project} handleProjectClick={handleProjectClick}/>))}
        </div>
      );
};

export default ProjectDisplayer;