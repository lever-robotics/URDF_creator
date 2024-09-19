import React from "react";

import { ProjectType } from "./ProjectDisplayer";

type Props = {
    project: ProjectType,
    handleProjectClick: (path: string, title: string) => void,
    onClose: () => void
}

const Project: React.FC<Props> = ({ project, handleProjectClick, onClose }) => {
    const handleClick = () => {
        if (project?.path === undefined) {
            onClose();
        } else {
            handleProjectClick(project.path, project.title);
        }
    };

    return (
        <div className="project" onClick={handleClick}>
            <div className="project-info">
                <p className="project-description">{project.description}</p>
            </div>
            <img className="project-image" src={project.image} alt="Project Image" width={project.width} height={project.height} />
            <div className="project-title">{project.title}</div>
        </div>
    );
};

export default Project;
