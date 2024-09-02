import React, { useState } from 'react';
import ExportURDFPackage from './ExportURDFPackage';

import './exportDisplayer.css';

const ExportDisplayer = ({ onClose, getBaseLink, projectTitle }) => {
    const [selectedIndex, setSelectedIndex] = useState(null);
    const [content, setContent] = useState("");

    const exportOptions = [
        { label: "URDF", content: "The URD"},
        { label: "Robot Package", content: <ExportURDFPackage onClose={onClose} getBaseLink={getBaseLink} projectTitle={projectTitle} /> },
        { label: "GLTF", content: "Save a current copy of this project to work on later in a GLTF file format"}
    ]

    return (
        <>
            <h2 className="title">Export Options</h2>
            <div className="import-displayer">
                <div className='import-menu-modal'>                
                    <ul className="menu-list">
                    {exportOptions.map((item, index) => (
                        <ExportOption 
                        key={index}
                        index={index} 
                        item={item} 
                        setContent={setContent} 
                        selectedIndex={selectedIndex}
                        setSelectedIndex={setSelectedIndex}
                    />
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
        <li 
            className={`menu-item ${selectedIndex === index ? 'menu-selected' : ''}`} 
            onClick={handleClick}
        >
            {item.label}
        </li>
    )
}

export default ExportDisplayer;