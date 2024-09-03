import React, { useState } from "react";
import ExportURDFPackage from "./ExportURDFPackage";
import ExportGLTF from "./ExportGLTF";
import ExportURDF from "./ExportURDF";

import "./exportDisplayer.css";

const ExportDisplayer = ({ onClose, getBaseLink, projectTitle, getScene, stateFunctions }) => {
    const [selectedIndex, setSelectedIndex] = useState(1);
    const [content, setContent] = useState(<ExportURDFPackage onClose={onClose} getScene={getScene} projectTitle={projectTitle} />);

    const exportOptions = [
        { label: "URDF", content: <ExportURDF onClose={onClose} getScene={getScene} projectTitle={projectTitle} /> },
        { label: "Robot Package", content: <ExportURDFPackage onClose={onClose} getScene={getScene} projectTitle={projectTitle} /> },
        { label: "GLTF", content: <ExportGLTF onClose={onClose} getBaseLink={getBaseLink} projectTitle={projectTitle} stateFunctions={stateFunctions} /> },
    ];

    return (
        <>
            <h2 className="title">Export Options</h2>
            <div className="import-displayer">
                <div className="import-menu-modal">
                    <ul className="menu-list">
                        {exportOptions.map((item, index) => (
                            <ExportOption key={index} index={index} item={item} setContent={setContent} selectedIndex={selectedIndex} setSelectedIndex={setSelectedIndex} />
                        ))}
                    </ul>
                </div>
                {content}
            </div>
        </>
    );
};

const ExportOption = ({ item, index, setContent, selectedIndex, setSelectedIndex }) => {
    const handleClick = () => {
        setContent(item.content);
        setSelectedIndex(index);
    };

    return (
        <li className={`menu-item ${selectedIndex === index ? "menu-selected" : ""}`} onClick={handleClick}>
            {item.label}
        </li>
    );
};

export default ExportDisplayer;
