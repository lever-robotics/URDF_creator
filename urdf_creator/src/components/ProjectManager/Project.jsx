import React from 'react';

const Project = ({ project }) => {

  return (
    <div className="project">
      <div className="project-info">
        <p className="project-description">{project.description}</p>
      </div>
        <img className="project-image" src={project.image} alt="Project Image" width={project.width} height={project.height}/>
        <div className="project-title">{project.title}</div>
    </div>
  );
};

export default Project;