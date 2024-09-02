import React, { useState, useRef } from 'react';
import urdfObjectManager from '../../../Models/urdfObjectManager';
import STLImport from './STLImport';
import GLTFImport from './GLTFImport';
import GltfFilesGrid from '../Import/ImportSensor';
import ReactGA from "react-ga4";

import './importDisplayer.css';
import '../../../FunctionalComponents/MenuModal.css';

const ImportDisplayer = ({ onImportClose, loadScene }) => {
    const [content, setContent] = useState("");
    const [selectedIndex, setSelectedIndex] = useState(null);

    const importOptions = [
        { label: "STL", content: <STLImport onClose={onImportClose} /> },
        { label: "GLTF", content: <GLTFImport onClose={onImportClose} loadScene={loadScene} />},
        { label: "Robot Sensor", action: () => {}, content: <GltfFilesGrid /> },
        // { label: "Sensors", content: },
        // { label: "Link", content: },
        // { label: "URDF", content: },
    ];

    return (
        <>
            <h2 className="title">Import Options</h2>
            <div className="import-displayer">
                <div className='stl-menu-modal'>                
                    <ul className="stl-menu-list">
                    {importOptions.map((item, index) => (
                        <ImportOption 
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

const ImportOption = ({ item, index, setContent, selectedIndex, setSelectedIndex }) => {

    const handleClick = () => {
        setContent(item.content);
        setSelectedIndex(index);
    };

    return (
        <li 
            className={`stl-menu-item ${selectedIndex === index ? 'stl-menu-selected' : ''}`} 
            onClick={handleClick}
        >
            {item.label}
        </li>
    )
}

export default ImportDisplayer;