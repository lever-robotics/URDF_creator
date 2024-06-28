import React from 'react';

const Project = ({ project, handleProjectClick }) => {

  return (
    <div className="project" onClick={() => handleProjectClick(project.path, project.title)}>
      <div className="project-info">
        <p className="project-description">{project.description}</p>
      </div>
        <img className="project-image" src={process.env.PUBLIC_URL + project.image} alt="Project Image" width={project.width} height={project.height}/>
        <div className="project-title">{project.title}</div>
    </div>
  );
};

export default Project;