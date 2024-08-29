import React, { useState } from 'react';

import './exportDisplayer.css';

const ExportDisplayer = ({ onClose }) => {

    const [content, setContent] = useState("");

    const exportOptions = [
        { label: "URDF", action: () => {}, content: "The URDF file"},
        { label: "Robot Package", action: () => {}, content: "Download the whole Robot Package necessary for ROS2"},
        { label: "GLTF", action: () => {}, content: "Save a current copy of this project to work on later in a GLTF file format"}
    ]

    return (
        <>
            <h2 className="title">Export Options</h2>
            <div className="export-displayer">
                <ul className="export-list">
                    {exportOptions.map((item, index) => (
                        <ExportOption index={index} item={item} setContent={setContent}/>
                    ))}
                </ul>
                <div className="content">
                    {content}
                </div>
            </div>
        </>
        
    );
};

const ExportOption = ({ item, setContent }) => {

    return (
        <li className="export-option" onMouseEnter={() => setContent(item.content)} onMouseLeave={() => setContent("")}>
            <span className="expot-option-span" onClick={item.action}>
                {item.label}
            </span>
        </li>
    )
}

export default ExportDisplayer;