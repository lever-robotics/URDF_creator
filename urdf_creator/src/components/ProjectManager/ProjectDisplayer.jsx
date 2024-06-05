import React, { useState } from "react";
import Project from './Project';

const ProjectDisplayer = () => {
  const [selectedProject, setSelectedProject] = useState(null);
  const [projects, setProjects] = useState([
      { title: 'SPOT', description: 'SPOT by Boston Dynamics', image: './ProjectStatics/...', width: '400', height: '400' },
      { title: 'OldMain', description: 'Haha we go to college here', image:'./statics/oldmain.jpg', width: '200', height: '200' },
      // Add more projects
    ]);


    const handleProjectClick = (project) => {
        setSelectedProject(project);
    }
    
      return (
        <div className="project-displayer">
          {projects.map((project, index) => (
            <Project key={index} project={project} />))}
        </div>
      );
};

export default ProjectDisplayer;